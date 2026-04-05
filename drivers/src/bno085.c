#include "bno085.h"

#include <string.h>

#define BNO085_CHANNEL_CONTROL 2u
#define BNO085_CHANNEL_REPORTS 3u
#define BNO085_REPORT_BASE_TIMESTAMP 0xFBu
#define BNO085_REPORT_SET_FEATURE_COMMAND 0xFDu

static float bno085_q_to_float(int16_t fixed_point, uint8_t q_point)
{
    return (float)fixed_point / (float)(1u << q_point);
}

static fc_status_t bno085_send_packet(bno085_t *device,
                                      uint8_t channel,
                                      const uint8_t *payload,
                                      uint16_t payload_len)
{
    uint8_t frame[32] = {0};
    uint16_t total_len;

    if (device == NULL || device->bus == NULL || device->bus->write == NULL || payload_len > 28u)
    {
        return FC_STATUS_INVALID_ARG;
    }

    total_len = (uint16_t)(payload_len + 4u);
    frame[0] = (uint8_t)(total_len & 0xFFu);
    frame[1] = (uint8_t)((total_len >> 8) & 0x7Fu);
    frame[2] = channel;
    frame[3] = device->sequence[channel]++;
    memcpy(&frame[4], payload, payload_len);

    return device->bus->write(device->bus->context, device->address, frame, total_len);
}

static fc_status_t bno085_read_packet(bno085_t *device,
                                      uint8_t *channel,
                                      uint8_t *payload,
                                      uint16_t *payload_len)
{
    uint8_t header[4] = {0};
    uint16_t total_len;
    fc_status_t status;

    if (device == NULL || channel == NULL || payload == NULL || payload_len == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    if (device->bus == NULL || device->bus->read == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    status = device->bus->read(device->bus->context, device->address, header, sizeof(header));
    if (status != FC_STATUS_OK)
    {
        return status;
    }

    total_len = (uint16_t)(((uint16_t)(header[1] & 0x7Fu) << 8) | header[0]);
    if (total_len <= 4u || total_len > 132u)
    {
        return FC_STATUS_NOT_READY;
    }

    *channel = header[2];
    *payload_len = (uint16_t)(total_len - 4u);
    return device->bus->read(device->bus->context, device->address, payload, *payload_len);
}

static int16_t bno085_le16(const uint8_t *buffer)
{
    return (int16_t)(((uint16_t)buffer[1] << 8) | buffer[0]);
}

static void bno085_parse_sensor_report(bno085_t *device,
                                       const uint8_t *payload,
                                       uint16_t payload_len)
{
    uint8_t sensor_id;

    if (payload_len < 19u || payload[0] != BNO085_REPORT_BASE_TIMESTAMP)
    {
        return;
    }

    sensor_id = payload[5];
    switch (sensor_id)
    {
        case BNO085_SENSOR_ACCELEROMETER:
            device->last_sample.accel_mps2.x = bno085_q_to_float(bno085_le16(&payload[9]), 8u);
            device->last_sample.accel_mps2.y = bno085_q_to_float(bno085_le16(&payload[11]), 8u);
            device->last_sample.accel_mps2.z = bno085_q_to_float(bno085_le16(&payload[13]), 8u);
            device->last_sample.valid = true;
            break;

        case BNO085_SENSOR_GYROSCOPE_CALIBRATED:
            device->last_sample.gyro_rps.x = bno085_q_to_float(bno085_le16(&payload[9]), 9u);
            device->last_sample.gyro_rps.y = bno085_q_to_float(bno085_le16(&payload[11]), 9u);
            device->last_sample.gyro_rps.z = bno085_q_to_float(bno085_le16(&payload[13]), 9u);
            device->last_sample.valid = true;
            break;

        case BNO085_SENSOR_ROTATION_VECTOR:
            device->last_sample.attitude.x = bno085_q_to_float(bno085_le16(&payload[9]), 14u);
            device->last_sample.attitude.y = bno085_q_to_float(bno085_le16(&payload[11]), 14u);
            device->last_sample.attitude.z = bno085_q_to_float(bno085_le16(&payload[13]), 14u);
            device->last_sample.attitude.w = bno085_q_to_float(bno085_le16(&payload[15]), 14u);
            device->last_sample.attitude_valid = true;
            device->last_sample.valid = true;
            break;

        default:
            break;
    }
}

fc_status_t bno085_enable_report(bno085_t *device,
                                 bno085_sensor_id_t sensor_id,
                                 uint32_t interval_us)
{
    uint8_t payload[17] = {0};

    if (device == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    payload[0] = BNO085_REPORT_SET_FEATURE_COMMAND;
    payload[1] = (uint8_t)sensor_id;
    payload[5] = (uint8_t)(interval_us & 0xFFu);
    payload[6] = (uint8_t)((interval_us >> 8) & 0xFFu);
    payload[7] = (uint8_t)((interval_us >> 16) & 0xFFu);
    payload[8] = (uint8_t)((interval_us >> 24) & 0xFFu);

    return bno085_send_packet(device, BNO085_CHANNEL_CONTROL, payload, sizeof(payload));
}

fc_status_t bno085_init(bno085_t *device, fc_i2c_bus_t *bus, uint8_t address)
{
    if (device == NULL || bus == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    memset(device, 0, sizeof(*device));
    device->bus = bus;
    device->address = address;

    if (bno085_enable_report(device, BNO085_SENSOR_ACCELEROMETER, 10000u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (bno085_enable_report(device, BNO085_SENSOR_GYROSCOPE_CALIBRATED, 10000u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (bno085_enable_report(device, BNO085_SENSOR_ROTATION_VECTOR, 10000u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    device->initialized = true;
    return FC_STATUS_OK;
}

fc_status_t bno085_read_raw(bno085_t *device, fc_imu_sample_t *sample)
{
    uint8_t channel = 0u;
    uint8_t payload[128] = {0};
    uint16_t payload_len = 0u;
    fc_status_t status;

    if (device == NULL || sample == NULL || !device->initialized)
    {
        return FC_STATUS_INVALID_ARG;
    }

    status = bno085_read_packet(device, &channel, payload, &payload_len);
    if (status != FC_STATUS_OK)
    {
        return status;
    }

    if (channel == BNO085_CHANNEL_REPORTS)
    {
        bno085_parse_sensor_report(device, payload, payload_len);
    }

    *sample = device->last_sample;
    return sample->valid ? FC_STATUS_OK : FC_STATUS_NOT_READY;
}
