#include "flight_computer.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CM5_SYNC_1 0xA5u
#define CM5_SYNC_2 0x5Au
#define CM5_MSG_STATE 0x01u
#define CM5_MSG_ASCENT_MODE 0x10u
#define CM5_MSG_PRO_NAV_MODE 0x11u
#define CM5_MSG_NMPC_MODE 0x12u
#define CM5_MSG_FIN_RESPONSE 0x81u

static uint32_t fc_now_ms(const fc_flight_computer_t *computer)
{
    if (computer != NULL && computer->board.time != NULL && computer->board.time->millis != NULL)
    {
        return computer->board.time->millis(computer->board.time->context);
    }

    return (computer != NULL) ? (computer->ukf_state.timestamp_ms + 10u) : 0u;
}

static float fc_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static fc_vec3_t fc_vec3_add(fc_vec3_t a, fc_vec3_t b)
{
    fc_vec3_t result = {a.x + b.x, a.y + b.y, a.z + b.z};
    return result;
}

static fc_vec3_t fc_vec3_scale(fc_vec3_t a, float scale)
{
    fc_vec3_t result = {a.x * scale, a.y * scale, a.z * scale};
    return result;
}

static float fc_vec3_length_xy(fc_vec3_t a)
{
    return sqrtf((a.x * a.x) + (a.y * a.y));
}

static fc_quat_t fc_quat_normalize(fc_quat_t q)
{
    float norm = sqrtf((q.w * q.w) + (q.x * q.x) + (q.y * q.y) + (q.z * q.z));

    if (norm > 0.0001f)
    {
        q.w /= norm;
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
    }
    else
    {
        q.w = 1.0f;
        q.x = 0.0f;
        q.y = 0.0f;
        q.z = 0.0f;
    }

    return q;
}

static void fc_quat_to_euler(const fc_quat_t *q, float *roll_rad, float *pitch_rad, float *yaw_rad)
{
    const float sinr_cosp = 2.0f * ((q->w * q->x) + (q->y * q->z));
    const float cosr_cosp = 1.0f - (2.0f * ((q->x * q->x) + (q->y * q->y)));
    const float sinp = 2.0f * ((q->w * q->y) - (q->z * q->x));
    const float siny_cosp = 2.0f * ((q->w * q->z) + (q->x * q->y));
    const float cosy_cosp = 1.0f - (2.0f * ((q->y * q->y) + (q->z * q->z)));

    if (roll_rad != NULL)
    {
        *roll_rad = atan2f(sinr_cosp, cosr_cosp);
    }
    if (pitch_rad != NULL)
    {
        *pitch_rad = (fabsf(sinp) >= 1.0f) ? copysignf(1.57079632679f, sinp) : asinf(sinp);
    }
    if (yaw_rad != NULL)
    {
        *yaw_rad = atan2f(siny_cosp, cosy_cosp);
    }
}

static uint16_t fc_crc16_ccitt(const uint8_t *buffer, size_t len)
{
    uint16_t crc = 0xFFFFu;
    size_t index;
    uint8_t bit;

    for (index = 0u; index < len; ++index)
    {
        crc ^= (uint16_t)buffer[index] << 8;
        for (bit = 0u; bit < 8u; ++bit)
        {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }

    return crc;
}

static fc_status_t fc_send_cm5_frame(fc_flight_computer_t *computer,
                                     uint8_t message_id,
                                     const uint8_t *payload,
                                     uint8_t payload_len)
{
    uint8_t frame[128] = {0};
    uint16_t crc;
    size_t total_len;

    if (computer == NULL || computer->board.cm5_uart == NULL || computer->board.cm5_uart->write == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    total_len = (size_t)payload_len + 6u;
    if (total_len > sizeof(frame))
    {
        return FC_STATUS_INVALID_ARG;
    }

    frame[0] = CM5_SYNC_1;
    frame[1] = CM5_SYNC_2;
    frame[2] = message_id;
    frame[3] = payload_len;
    if (payload_len > 0u && payload != NULL)
    {
        memcpy(&frame[4], payload, payload_len);
    }

    crc = fc_crc16_ccitt(&frame[2], (size_t)payload_len + 2u);
    frame[4u + payload_len] = (uint8_t)(crc & 0xFFu);
    frame[5u + payload_len] = (uint8_t)(crc >> 8);
    return computer->board.cm5_uart->write(computer->board.cm5_uart->context, frame, total_len);
}

static bool fc_parse_cm5_frame(const uint8_t *buffer,
                               size_t len,
                               uint8_t *message_id,
                               const uint8_t **payload,
                               uint8_t *payload_len)
{
    uint16_t received_crc;
    uint16_t computed_crc;
    uint8_t length;

    if (buffer == NULL || len < 6u)
    {
        return false;
    }

    if (buffer[0] != CM5_SYNC_1 || buffer[1] != CM5_SYNC_2)
    {
        return false;
    }

    length = buffer[3];
    if (len < (size_t)length + 6u)
    {
        return false;
    }

    received_crc = (uint16_t)(((uint16_t)buffer[5u + length] << 8) | buffer[4u + length]);
    computed_crc = fc_crc16_ccitt(&buffer[2], (size_t)length + 2u);
    if (received_crc != computed_crc)
    {
        return false;
    }

    *message_id = buffer[2];
    *payload_len = length;
    *payload = &buffer[4];
    return true;
}

static fc_status_t fc_send_fin_command_uart(fc_flight_computer_t *computer, const fc_fin_command_t *command)
{
    char packet[96];
    int len;

    if (computer == NULL || command == NULL || computer->board.fin_uart == NULL ||
        computer->board.fin_uart->write == NULL)
    {
        return FC_STATUS_UNSUPPORTED;
    }

    len = snprintf(packet,
                   sizeof(packet),
                   "$FIN,%.2f,%.2f,%.2f,%.2f\n",
                   command->fin1_deg,
                   command->fin2_deg,
                   command->fin3_deg,
                   command->fin4_deg);
    if (len <= 0)
    {
        return FC_STATUS_ERROR;
    }

    return computer->board.fin_uart->write(computer->board.fin_uart->context,
                                           (const uint8_t *)packet,
                                           (size_t)len);
}

static fc_status_t fc_update_navigation_suite(fc_flight_computer_t *computer, bool include_gps)
{
    float lidar_range_m = 0.0f;
    mmwave_measurement_t mmwave_measurement = {0};

    if (readBaroPressure(computer) == FC_STATUS_OK)
    {
        /* Baro is optional in a single step as long as the previous sample is still valid. */
    }
    (void)readOrientationRaw1(computer);
    (void)readOrientationRaw2(computer);

    if (computer->lidar.uart != NULL && lidar_uart_poll(&computer->lidar, &lidar_range_m) == FC_STATUS_OK)
    {
        computer->lidar_range_m = lidar_range_m;
    }

    if (computer->mmwave.initialized && mmwave_doppler_read(&computer->mmwave, &mmwave_measurement) ==
                                            FC_STATUS_OK && mmwave_measurement.valid)
    {
        computer->ukf_state.velocity_mps.z = -mmwave_measurement.radial_velocity_mps;
    }

    if (include_gps)
    {
        (void)ReadGPS(computer);
    }

    return UKFState(computer, fc_now_ms(computer));
}

static bool fc_detect_launch(const fc_flight_computer_t *computer)
{
    return computer != NULL && computer->ukf_state.acceleration_mps2.z >
                                   (FC_LAUNCH_ACCEL_THRESHOLD_G * FC_ONE_G_MPS2);
}

static bool fc_detect_burnout(fc_flight_computer_t *computer, uint32_t now_ms)
{
    const float accel_threshold = FC_BURNOUT_ACCEL_THRESHOLD_G * FC_ONE_G_MPS2;
    float dt_ms;

    if (computer == NULL)
    {
        return false;
    }

    dt_ms = (computer->ukf_state.timestamp_ms > 0u) ? (float)(now_ms - computer->ukf_state.timestamp_ms) : 10.0f;
    if (dt_ms <= 0.0f)
    {
        dt_ms = 10.0f;
    }
    if (computer->ukf_state.acceleration_mps2.z < accel_threshold)
    {
        computer->burnout_timer_ms += dt_ms;
    }
    else
    {
        computer->burnout_timer_ms = 0.0f;
    }

    return computer->burnout_timer_ms >= (float)FC_BURNOUT_CONFIRM_MS;
}

static bool fc_detect_apogee(const fc_flight_computer_t *computer)
{
    return computer != NULL && computer->ukf_state.altitude_agl_m >= FC_APOGEE_MIN_ALTITUDE_M &&
           computer->ukf_state.velocity_mps.z <= 0.0f;
}

static bool fc_detect_landed(const fc_flight_computer_t *computer)
{
    if (computer == NULL)
    {
        return false;
    }

    if (computer->lidar.valid && computer->lidar_range_m <= 0.20f)
    {
        return true;
    }

    return computer->ukf_state.altitude_agl_m <= 0.20f;
}

static fc_status_t fc_transition_state(fc_flight_computer_t *computer,
                                       fc_flight_state_t new_state,
                                       uint32_t now_ms)
{
    if (computer == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    computer->state = new_state;
    computer->state_entry_ms = now_ms;
    computer->burnout_timer_ms = 0.0f;

    if (new_state == FLIGHT_STATE_ARMED_RTL)
    {
        return Pyro_On(computer);
    }

    if (new_state == FLIGHT_STATE_SUICIDE_BURN && !computer->second_stage_ignited)
    {
        computer->second_stage_ignited = (Pyro_Fire2(computer) == FC_STATUS_OK);
    }

    if (new_state == FLIGHT_STATE_LANDED || new_state == FLIGHT_STATE_ABORT)
    {
        return Pyro_Off(computer);
    }

    return FC_STATUS_OK;
}

fc_status_t readBaroPressure(fc_flight_computer_t *computer)
{
    if (computer == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    computer->baro_sample.timestamp_ms = fc_now_ms(computer);
    return ms5611_read(&computer->baro, &computer->baro_sample);
}

fc_status_t readOrientationRaw1(fc_flight_computer_t *computer)
{
    if (computer == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    computer->imu1_sample.timestamp_ms = fc_now_ms(computer);
    return bno085_read_raw(&computer->imu1, &computer->imu1_sample);
}

fc_status_t readOrientationRaw2(fc_flight_computer_t *computer)
{
    if (computer == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    computer->imu2_sample.timestamp_ms = fc_now_ms(computer);
    return icm42688p_read_raw(&computer->imu2, &computer->imu2_sample);
}

fc_status_t ReadGPS(fc_flight_computer_t *computer)
{
    fc_status_t status;

    if (computer == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    status = zed_f9p_poll(&computer->gps, &computer->gps_fix);
    if (status == FC_STATUS_OK)
    {
        computer->gps_fix.valid = true;
    }
    return status;
}

fc_status_t UKFState(fc_flight_computer_t *computer, uint32_t now_ms)
{
    fc_vec3_t accel_sum = {0.0f, 0.0f, 0.0f};
    uint8_t accel_count = 0u;
    float dt_s = 0.0f;
    float baro_agl_m = computer->baro_sample.altitude_m - computer->ground_baro_altitude_m;

    if (computer == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    /*
     * This is the estimator integration point for your full UKF.
     * For now it keeps the state machine alive with deterministic propagation
     * and measurement blending across the baro, IMUs, GPS, lidar, and Doppler.
     */
    if (computer->ukf_state.timestamp_ms != 0u && now_ms > computer->ukf_state.timestamp_ms)
    {
        dt_s = (float)(now_ms - computer->ukf_state.timestamp_ms) / 1000.0f;
    }

    if (computer->imu1_sample.valid)
    {
        accel_sum = fc_vec3_add(accel_sum, computer->imu1_sample.accel_mps2);
        ++accel_count;
        if (computer->imu1_sample.attitude_valid)
        {
            computer->ukf_state.attitude = fc_quat_normalize(computer->imu1_sample.attitude);
        }
    }

    if (computer->imu2_sample.valid)
    {
        accel_sum = fc_vec3_add(accel_sum, computer->imu2_sample.accel_mps2);
        ++accel_count;
        if (!computer->imu1_sample.attitude_valid && !computer->imu2_sample.attitude_valid)
        {
            computer->ukf_state.attitude = fc_quat_normalize(computer->ukf_state.attitude);
        }
    }

    if (accel_count > 0u)
    {
        computer->ukf_state.acceleration_mps2 = fc_vec3_scale(accel_sum, 1.0f / (float)accel_count);
    }

    if (computer->gps_fix.valid)
    {
        computer->ukf_state.position_m = computer->gps_fix.position_m;
        computer->ukf_state.velocity_mps = computer->gps_fix.velocity_mps;
        computer->ukf_state.gps_valid = true;
    }
    else if (dt_s > 0.0f)
    {
        computer->ukf_state.velocity_mps =
            fc_vec3_add(computer->ukf_state.velocity_mps, fc_vec3_scale(computer->ukf_state.acceleration_mps2, dt_s));
        computer->ukf_state.position_m =
            fc_vec3_add(computer->ukf_state.position_m, fc_vec3_scale(computer->ukf_state.velocity_mps, dt_s));
    }

    if (computer->baro_sample.valid)
    {
        computer->ukf_state.altitude_agl_m = baro_agl_m;
        computer->ukf_state.position_m.z = baro_agl_m;
        computer->ukf_state.pressure_pa = computer->baro_sample.pressure_pa;
    }

    if (computer->lidar.valid && computer->lidar_range_m > 0.0f)
    {
        computer->ukf_state.altitude_agl_m = computer->lidar_range_m;
        computer->ukf_state.position_m.z = computer->lidar_range_m;
    }

    computer->ukf_state.timestamp_ms = now_ms;
    return FC_STATUS_OK;
}

fc_status_t TransmitStateCm5(fc_flight_computer_t *computer)
{
    uint8_t payload[80] = {0};
    size_t offset = 0u;
    const float state_as_float = (float)computer->state;

    if (computer == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    memcpy(&payload[offset], &computer->ukf_state.timestamp_ms, sizeof(computer->ukf_state.timestamp_ms));
    offset += sizeof(computer->ukf_state.timestamp_ms);
    memcpy(&payload[offset], &state_as_float, sizeof(state_as_float));
    offset += sizeof(state_as_float);
    memcpy(&payload[offset], &computer->ukf_state.position_m, sizeof(computer->ukf_state.position_m));
    offset += sizeof(computer->ukf_state.position_m);
    memcpy(&payload[offset], &computer->ukf_state.velocity_mps, sizeof(computer->ukf_state.velocity_mps));
    offset += sizeof(computer->ukf_state.velocity_mps);
    memcpy(&payload[offset], &computer->ukf_state.acceleration_mps2, sizeof(computer->ukf_state.acceleration_mps2));
    offset += sizeof(computer->ukf_state.acceleration_mps2);
    memcpy(&payload[offset], &computer->ukf_state.attitude, sizeof(computer->ukf_state.attitude));
    offset += sizeof(computer->ukf_state.attitude);
    memcpy(&payload[offset], &computer->ukf_state.altitude_agl_m, sizeof(computer->ukf_state.altitude_agl_m));
    offset += sizeof(computer->ukf_state.altitude_agl_m);

    return fc_send_cm5_frame(computer, CM5_MSG_STATE, payload, (uint8_t)offset);
}

fc_status_t CostFunction1(fc_flight_computer_t *computer)
{
    uint8_t payload = 1u;

    if (computer == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    return fc_send_cm5_frame(computer, CM5_MSG_ASCENT_MODE, &payload, 1u);
}

fc_status_t recieveCM5(fc_flight_computer_t *computer)
{
    uint8_t buffer[64] = {0};
    size_t received_len = 0u;
    const uint8_t *payload = NULL;
    uint8_t payload_len = 0u;
    uint8_t message_id = 0u;

    if (computer == NULL || computer->board.cm5_uart == NULL || computer->board.cm5_uart->read == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    if (computer->board.cm5_uart->read(computer->board.cm5_uart->context,
                                       buffer,
                                       sizeof(buffer),
                                       0u,
                                       &received_len) != FC_STATUS_OK ||
        received_len == 0u)
    {
        return FC_STATUS_NOT_READY;
    }

    if (!fc_parse_cm5_frame(buffer, received_len, &message_id, &payload, &payload_len))
    {
        return FC_STATUS_CRC_ERROR;
    }

    if (message_id == CM5_MSG_FIN_RESPONSE && payload_len >= 20u)
    {
        memcpy(&computer->cm5_command.generation, &payload[0], sizeof(computer->cm5_command.generation));
        memcpy(&computer->cm5_command.fin_command.fin1_deg, &payload[4], sizeof(float));
        memcpy(&computer->cm5_command.fin_command.fin2_deg, &payload[8], sizeof(float));
        memcpy(&computer->cm5_command.fin_command.fin3_deg, &payload[12], sizeof(float));
        memcpy(&computer->cm5_command.fin_command.fin4_deg, &payload[16], sizeof(float));
        computer->cm5_command.fin_command.valid = true;
        computer->cm5_command.valid = true;
        computer->last_cm5_rx_ms = fc_now_ms(computer);
        return FC_STATUS_OK;
    }

    return FC_STATUS_NOT_READY;
}

fc_status_t InnerLQRFIN(fc_flight_computer_t *computer)
{
    const float smoothing = 0.25f;

    if (computer == NULL || !computer->cm5_command.valid || !computer->cm5_command.fin_command.valid)
    {
        return FC_STATUS_NOT_READY;
    }

    computer->applied_fin_command.fin1_deg +=
        (computer->cm5_command.fin_command.fin1_deg - computer->applied_fin_command.fin1_deg) * smoothing;
    computer->applied_fin_command.fin2_deg +=
        (computer->cm5_command.fin_command.fin2_deg - computer->applied_fin_command.fin2_deg) * smoothing;
    computer->applied_fin_command.fin3_deg +=
        (computer->cm5_command.fin_command.fin3_deg - computer->applied_fin_command.fin3_deg) * smoothing;
    computer->applied_fin_command.fin4_deg +=
        (computer->cm5_command.fin_command.fin4_deg - computer->applied_fin_command.fin4_deg) * smoothing;
    computer->applied_fin_command.valid = true;
    return fc_send_fin_command_uart(computer, &computer->applied_fin_command);
}

fc_status_t LQRTVC(fc_flight_computer_t *computer)
{
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;
    float target_pitch_rad = 0.0f;
    float target_yaw_rad = 0.0f;
    float pitch_cmd;
    float yaw_cmd;
    fc_vec3_t waypoint_vector;

    if (computer == NULL || computer->board.pwm == NULL || computer->board.pwm->set_normalized == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    fc_quat_to_euler(&computer->ukf_state.attitude, &roll_rad, &pitch_rad, &yaw_rad);

    if (computer->state == FLIGHT_STATE_SUICIDE_BURN)
    {
        waypoint_vector = ComputeVectortoWaypoint(computer);
        target_pitch_rad = fc_clampf((-waypoint_vector.y * 0.005f) - (computer->ukf_state.velocity_mps.y * 0.03f),
                                     -0.35f,
                                     0.35f);
        target_yaw_rad = fc_clampf((waypoint_vector.x * 0.005f) - (computer->ukf_state.velocity_mps.x * 0.03f),
                                   -0.35f,
                                   0.35f);
    }

    pitch_cmd = fc_clampf(0.5f + ((target_pitch_rad - pitch_rad) * 0.8f) - (computer->imu2_sample.gyro_rps.y * 0.05f),
                          0.0f,
                          1.0f);
    yaw_cmd = fc_clampf(0.5f + ((target_yaw_rad - yaw_rad) * 0.8f) - (computer->imu2_sample.gyro_rps.z * 0.05f),
                        0.0f,
                        1.0f);

    computer->tvc_command.pitch_cmd_rad = target_pitch_rad;
    computer->tvc_command.yaw_cmd_rad = target_yaw_rad;
    computer->tvc_command.valid = true;

    (void)computer->board.pwm->set_normalized(computer->board.pwm->context, 1u, pitch_cmd);
    (void)computer->board.pwm->set_normalized(computer->board.pwm->context, 2u, 1.0f - pitch_cmd);
    (void)computer->board.pwm->set_normalized(computer->board.pwm->context, 3u, yaw_cmd);
    (void)computer->board.pwm->set_normalized(computer->board.pwm->context, 4u, 1.0f - yaw_cmd);
    return FC_STATUS_OK;
}

fc_status_t Pyro_On(fc_flight_computer_t *computer)
{
    if (computer == NULL || computer->board.pyro == NULL || computer->board.pyro->set_armed == NULL ||
        computer->board.pyro->continuity_ok == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    computer->pyro_status.continuity_ok = computer->board.pyro->continuity_ok(computer->board.pyro->context);
    if (!computer->pyro_status.continuity_ok)
    {
        computer->pyro_status.armed = false;
        return FC_STATUS_NOT_READY;
    }

    if (computer->board.pyro->set_armed(computer->board.pyro->context, true) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    computer->pyro_status.armed = true;
    return FC_STATUS_OK;
}

fc_status_t Pyro_Off(fc_flight_computer_t *computer)
{
    if (computer == NULL || computer->board.pyro == NULL || computer->board.pyro->set_armed == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    computer->pyro_status.armed = false;
    return computer->board.pyro->set_armed(computer->board.pyro->context, false);
}

fc_status_t Pyro_Fire(fc_flight_computer_t *computer)
{
    if (computer == NULL || computer->board.pyro == NULL || computer->board.pyro->fire_channel == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    return computer->board.pyro->fire_channel(computer->board.pyro->context, 1u);
}

fc_status_t Pyro_Fire2(fc_flight_computer_t *computer)
{
    if (computer == NULL || computer->board.pyro == NULL || computer->board.pyro->fire_channel == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    return computer->board.pyro->fire_channel(computer->board.pyro->context, 2u);
}

fc_status_t LoraTransmit(fc_flight_computer_t *computer)
{
    char telemetry[160];
    int len;

    if (computer == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    len = snprintf(telemetry,
                   sizeof(telemetry),
                   "S:%u,T:%lu,ALT:%.1f,X:%.1f,Y:%.1f,VZ:%.1f,LAT:%.7f,LON:%.7f",
                   (unsigned int)computer->state,
                   (unsigned long)computer->ukf_state.timestamp_ms,
                   computer->ukf_state.altitude_agl_m,
                   computer->ukf_state.position_m.x,
                   computer->ukf_state.position_m.y,
                   computer->ukf_state.velocity_mps.z,
                   computer->gps_fix.latitude_deg,
                   computer->gps_fix.longitude_deg);
    if (len <= 0)
    {
        return FC_STATUS_ERROR;
    }

    return lora_radio_transmit(&computer->lora, (const uint8_t *)telemetry, (size_t)len);
}

fc_status_t Flashwrite(fc_flight_computer_t *computer)
{
    fc_log_record_t record;
    uint32_t write_address;
    uint32_t page_offset;

    if (computer == NULL || !computer->flash.initialized)
    {
        return FC_STATUS_INVALID_ARG;
    }

    record.timestamp_ms = computer->ukf_state.timestamp_ms;
    record.state = computer->state;
    record.ukf_state = computer->ukf_state;
    record.gps_fix = computer->gps_fix;

    write_address = computer->flash.next_log_address;
    page_offset = write_address % FC_FLASH_LOG_PAGE_SIZE;

    if ((page_offset + sizeof(record)) > FC_FLASH_LOG_PAGE_SIZE)
    {
        write_address += FC_FLASH_LOG_PAGE_SIZE - page_offset;
    }

    if ((write_address % FC_FLASH_LOG_SECTOR_SIZE) == 0u)
    {
        (void)w25q128_erase_sector(&computer->flash, write_address);
    }

    if (w25q128_page_program(&computer->flash, write_address, (const uint8_t *)&record, sizeof(record)) !=
        FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    computer->flash.next_log_address = write_address + (uint32_t)sizeof(record);
    return FC_STATUS_OK;
}

fc_status_t ProportionalNavigationCM5(fc_flight_computer_t *computer)
{
    uint8_t payload[16] = {0};
    fc_vec3_t vector;

    if (computer == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    vector = ComputeVectortoWaypoint(computer);
    memcpy(&payload[0], &vector.x, sizeof(float));
    memcpy(&payload[4], &vector.y, sizeof(float));
    memcpy(&payload[8], &vector.z, sizeof(float));
    memcpy(&payload[12], &computer->ukf_state.altitude_agl_m, sizeof(float));
    return fc_send_cm5_frame(computer, CM5_MSG_PRO_NAV_MODE, payload, sizeof(payload));
}

fc_status_t NMPCCM5(fc_flight_computer_t *computer)
{
    uint8_t payload[20] = {0};
    fc_vec3_t vector;

    if (computer == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    vector = ComputeVectortoWaypoint(computer);
    memcpy(&payload[0], &vector.x, sizeof(float));
    memcpy(&payload[4], &vector.y, sizeof(float));
    memcpy(&payload[8], &vector.z, sizeof(float));
    memcpy(&payload[12], &computer->ukf_state.velocity_mps.z, sizeof(float));
    memcpy(&payload[16], &computer->suicide_burn_height_m, sizeof(float));
    return fc_send_cm5_frame(computer, CM5_MSG_NMPC_MODE, payload, sizeof(payload));
}

float ComputeSuicideBurn(fc_flight_computer_t *computer)
{
    float vertical_speed_down_mps;
    float net_deceleration_mps2;
    float burn_height_m;

    if (computer == NULL)
    {
        return 0.0f;
    }

    vertical_speed_down_mps = fmaxf(0.0f, -computer->ukf_state.velocity_mps.z);
    net_deceleration_mps2 = fmaxf(1.0f, FC_SECOND_STAGE_MAX_DECEL_MPS2 - FC_ONE_G_MPS2);
    burn_height_m =
        ((vertical_speed_down_mps * vertical_speed_down_mps) / (2.0f * net_deceleration_mps2)) +
        (vertical_speed_down_mps * FC_IGNITION_LATENCY_S) + FC_SUICIDE_BURN_SAFETY_MARGIN_M;
    computer->suicide_burn_height_m = burn_height_m;
    return burn_height_m;
}

fc_vec3_t ComputeVectortoWaypoint(const fc_flight_computer_t *computer)
{
    fc_vec3_t vector = {0.0f, 0.0f, 0.0f};

    if (computer != NULL)
    {
        vector.x = computer->waypoint.x_m - computer->ukf_state.position_m.x;
        vector.y = computer->waypoint.y_m - computer->ukf_state.position_m.y;
        vector.z = -computer->ukf_state.altitude_agl_m;
    }

    return vector;
}

bool ComputeLandingPossibility(fc_flight_computer_t *computer)
{
    fc_vec3_t vector;
    float crossrange_m;
    bool nav_ok;
    bool cm5_ok;
    bool enough_altitude;

    if (computer == NULL)
    {
        return false;
    }

    vector = ComputeVectortoWaypoint(computer);
    crossrange_m = fc_vec3_length_xy(vector);
    nav_ok = computer->gps_fix.valid &&
             (computer->gps_fix.rtk_fix || computer->gps_fix.satellites >= FC_MIN_RTK_SATELLITES);
    cm5_ok = (computer->last_cm5_rx_ms == 0u) ||
             ((fc_now_ms(computer) - computer->last_cm5_rx_ms) <= FC_CM5_LINK_TIMEOUT_MS);
    enough_altitude = computer->ukf_state.altitude_agl_m > 10.0f;

    computer->ukf_state.landing_possible = nav_ok && cm5_ok && enough_altitude &&
                                           (crossrange_m <= FC_MAX_LANDING_CROSSRANGE_M);
    return computer->ukf_state.landing_possible;
}

fc_status_t FlightComputer_Init(fc_flight_computer_t *computer, const fc_board_config_t *board)
{
    if (computer == NULL || board == NULL || board->baro_i2c == NULL || board->imu1_i2c == NULL ||
        board->imu2_spi == NULL || board->lidar_uart == NULL || board->fdcan == NULL ||
        board->gps_uart == NULL || board->rpi_uart == NULL || board->cm5_uart == NULL ||
        board->pwm == NULL || board->lora_spi == NULL || board->flash_spi == NULL ||
        board->mmwave_spi == NULL || board->pyro == NULL || board->time == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    memset(computer, 0, sizeof(*computer));
    computer->board = *board;
    computer->waypoint.x_m = FC_DEFAULT_WAYPOINT_X_M;
    computer->waypoint.y_m = FC_DEFAULT_WAYPOINT_Y_M;
    computer->ukf_state.attitude.w = 1.0f;

    if (ms5611_init(&computer->baro, board->baro_i2c, FC_MS5611_I2C_ADDRESS) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (bno085_init(&computer->imu1, board->imu1_i2c, FC_BNO085_I2C_ADDRESS) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (icm42688p_init(&computer->imu2, board->imu2_spi) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lidar_uart_init(&computer->lidar, board->lidar_uart) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (zed_f9p_init(&computer->gps, board->gps_uart) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_radio_init(&computer->lora, board->lora_spi, 915000000u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (w25q128_init(&computer->flash, board->flash_spi) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (mmwave_doppler_init(&computer->mmwave, board->mmwave_spi) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    if (readBaroPressure(computer) == FC_STATUS_OK)
    {
        computer->ground_baro_altitude_m = computer->baro_sample.altitude_m;
    }

    computer->state = FLIGHT_STATE_STANDBY;
    computer->state_entry_ms = fc_now_ms(computer);
    return FC_STATUS_OK;
}

fc_status_t FlightComputer_Step(fc_flight_computer_t *computer)
{
    uint32_t now_ms;
    uint8_t lora_payload[64] = {0};
    size_t lora_len = 0u;
    fc_lora_command_t command = {FC_LORA_CMD_NONE, false};

    if (computer == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    now_ms = fc_now_ms(computer);

    if (lora_radio_receive(&computer->lora, lora_payload, sizeof(lora_payload), &lora_len) == FC_STATUS_OK)
    {
        command = lora_radio_decode_command(lora_payload, lora_len);
        if (command.valid && command.id == FC_LORA_CMD_ABORT)
        {
            (void)fc_transition_state(computer, FLIGHT_STATE_ABORT, now_ms);
        }
        if (command.valid && command.id == FC_LORA_CMD_DISARM)
        {
            (void)fc_transition_state(computer, FLIGHT_STATE_STANDBY, now_ms);
            (void)Pyro_Off(computer);
        }
    }

    switch (computer->state)
    {
        case FLIGHT_STATE_STANDBY:
            if (command.valid && command.id == FC_LORA_CMD_ARM)
            {
                if (fc_transition_state(computer, FLIGHT_STATE_ARMED_RTL, now_ms) != FC_STATUS_OK)
                {
                    (void)fc_transition_state(computer, FLIGHT_STATE_ABORT, now_ms);
                }
            }
            break;

        case FLIGHT_STATE_ARMED_RTL:
            (void)fc_update_navigation_suite(computer, false);
            (void)TransmitStateCm5(computer);
            (void)CostFunction1(computer);
            (void)recieveCM5(computer);
            (void)LoraTransmit(computer);
            (void)Flashwrite(computer);

            if (fc_detect_launch(computer))
            {
                computer->launch_detected = true;
                (void)fc_transition_state(computer, FLIGHT_STATE_BURN, now_ms);
            }
            break;

        case FLIGHT_STATE_BURN:
            (void)fc_update_navigation_suite(computer, true);
            (void)LQRTVC(computer);
            (void)LoraTransmit(computer);
            (void)Flashwrite(computer);

            if (fc_detect_burnout(computer, now_ms))
            {
                (void)fc_transition_state(computer, FLIGHT_STATE_COAST, now_ms);
            }
            break;

        case FLIGHT_STATE_COAST:
            (void)fc_update_navigation_suite(computer, true);
            (void)TransmitStateCm5(computer);
            (void)ProportionalNavigationCM5(computer);
            (void)recieveCM5(computer);
            (void)InnerLQRFIN(computer);
            (void)LoraTransmit(computer);
            (void)Flashwrite(computer);

            if (fc_detect_apogee(computer))
            {
                (void)fc_transition_state(computer, FLIGHT_STATE_APOGEE, now_ms);
            }
            break;

        case FLIGHT_STATE_APOGEE:
            (void)fc_update_navigation_suite(computer, true);
            if (!ComputeLandingPossibility(computer))
            {
                (void)Pyro_Fire(computer);
                (void)fc_transition_state(computer, FLIGHT_STATE_ABORT, now_ms);
                break;
            }

            if (computer->ukf_state.velocity_mps.z < 0.0f)
            {
                (void)fc_transition_state(computer, FLIGHT_STATE_DESCENT, now_ms);
            }
            break;

        case FLIGHT_STATE_DESCENT:
            (void)fc_update_navigation_suite(computer, true);
            (void)TransmitStateCm5(computer);
            (void)NMPCCM5(computer);
            (void)recieveCM5(computer);
            (void)InnerLQRFIN(computer);
            (void)ComputeSuicideBurn(computer);
            (void)LoraTransmit(computer);
            (void)Flashwrite(computer);

            if (computer->ukf_state.altitude_agl_m <=
                (computer->suicide_burn_height_m + FC_SUICIDE_BURN_ALTITUDE_OFFSET_M))
            {
                (void)fc_transition_state(computer, FLIGHT_STATE_SUICIDE_BURN, now_ms);
            }
            break;

        case FLIGHT_STATE_SUICIDE_BURN:
            (void)fc_update_navigation_suite(computer, true);
            (void)LQRTVC(computer);
            (void)LoraTransmit(computer);
            (void)Flashwrite(computer);

            if (fc_detect_landed(computer))
            {
                (void)fc_transition_state(computer, FLIGHT_STATE_LANDED, now_ms);
            }
            break;

        case FLIGHT_STATE_LANDED:
            (void)Pyro_Off(computer);
            (void)LoraTransmit(computer);
            (void)Flashwrite(computer);
            break;

        case FLIGHT_STATE_ABORT:
            (void)Pyro_Off(computer);
            (void)LoraTransmit(computer);
            (void)Flashwrite(computer);
            break;

        default:
            return FC_STATUS_UNSUPPORTED;
    }

    return FC_STATUS_OK;
}
