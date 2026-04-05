#include "zed_f9p.h"

#include <math.h>
#include <string.h>

#define UBX_SYNC_CHAR_1 0xB5u
#define UBX_SYNC_CHAR_2 0x62u
#define UBX_CLASS_CFG 0x06u
#define UBX_ID_CFG_MSG 0x01u
#define UBX_ID_CFG_RATE 0x08u
#define UBX_CLASS_NAV 0x01u
#define UBX_ID_NAV_PVT 0x07u

enum
{
    ZED_STATE_SYNC_1 = 0,
    ZED_STATE_SYNC_2,
    ZED_STATE_CLASS,
    ZED_STATE_ID,
    ZED_STATE_LENGTH_1,
    ZED_STATE_LENGTH_2,
    ZED_STATE_PAYLOAD,
    ZED_STATE_CHECKSUM_A,
    ZED_STATE_CHECKSUM_B
};

static void zed_f9p_checksum_step(uint8_t *checksum_a, uint8_t *checksum_b, uint8_t value)
{
    *checksum_a = (uint8_t)(*checksum_a + value);
    *checksum_b = (uint8_t)(*checksum_b + *checksum_a);
}

static uint32_t zed_f9p_u32(const uint8_t *buffer)
{
    return ((uint32_t)buffer[3] << 24) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[1] << 8) |
           buffer[0];
}

static int32_t zed_f9p_i32(const uint8_t *buffer)
{
    return (int32_t)zed_f9p_u32(buffer);
}

static void zed_f9p_update_local_position(zed_f9p_t *device, fc_gps_fix_t *fix)
{
    const double earth_radius_m = 6378137.0;
    double d_lat_rad;
    double d_lon_rad;
    double mean_lat_rad;

    if (!device->origin_set)
    {
        device->origin_latitude_deg = fix->latitude_deg;
        device->origin_longitude_deg = fix->longitude_deg;
        device->origin_altitude_m = fix->altitude_msl_m;
        device->origin_set = true;
    }

    d_lat_rad = (fix->latitude_deg - device->origin_latitude_deg) * (3.14159265358979323846 / 180.0);
    d_lon_rad = (fix->longitude_deg - device->origin_longitude_deg) * (3.14159265358979323846 / 180.0);
    mean_lat_rad =
        ((fix->latitude_deg + device->origin_latitude_deg) * 0.5) * (3.14159265358979323846 / 180.0);

    fix->position_m.x = (float)(d_lon_rad * cos(mean_lat_rad) * earth_radius_m);
    fix->position_m.y = (float)(d_lat_rad * earth_radius_m);
    fix->position_m.z = fix->altitude_msl_m - device->origin_altitude_m;
}

static void zed_f9p_parse_nav_pvt(zed_f9p_t *device)
{
    fc_gps_fix_t *fix = &device->last_fix;
    uint8_t fix_type = device->payload[20];
    uint8_t flags = device->payload[21];
    uint8_t carrier_solution = (uint8_t)((flags >> 6) & 0x03u);

    memset(fix, 0, sizeof(*fix));
    fix->latitude_deg = (double)zed_f9p_i32(&device->payload[28]) * 1.0e-7;
    fix->longitude_deg = (double)zed_f9p_i32(&device->payload[24]) * 1.0e-7;
    fix->altitude_msl_m = (float)zed_f9p_i32(&device->payload[36]) / 1000.0f;
    fix->velocity_mps.y = (float)zed_f9p_i32(&device->payload[48]) / 1000.0f;
    fix->velocity_mps.x = (float)zed_f9p_i32(&device->payload[52]) / 1000.0f;
    fix->velocity_mps.z = -(float)zed_f9p_i32(&device->payload[56]) / 1000.0f;
    fix->satellites = device->payload[23];
    fix->valid = (fix_type >= 3u) && ((flags & 0x01u) != 0u);
    fix->rtk_fix = (carrier_solution == 2u);

    if (fix->valid)
    {
        zed_f9p_update_local_position(device, fix);
    }
}

static fc_status_t zed_f9p_send_frame(zed_f9p_t *device,
                                      uint8_t message_class,
                                      uint8_t message_id,
                                      const uint8_t *payload,
                                      uint16_t payload_length)
{
    uint8_t frame[64] = {0};
    uint8_t checksum_a = 0u;
    uint8_t checksum_b = 0u;
    uint16_t index = 0u;
    uint16_t write_len;

    if (device == NULL || device->uart == NULL || device->uart->write == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    if ((size_t)payload_length + 8u > sizeof(frame))
    {
        return FC_STATUS_INVALID_ARG;
    }

    frame[0] = UBX_SYNC_CHAR_1;
    frame[1] = UBX_SYNC_CHAR_2;
    frame[2] = message_class;
    frame[3] = message_id;
    frame[4] = (uint8_t)(payload_length & 0xFFu);
    frame[5] = (uint8_t)((payload_length >> 8) & 0xFFu);

    zed_f9p_checksum_step(&checksum_a, &checksum_b, frame[2]);
    zed_f9p_checksum_step(&checksum_a, &checksum_b, frame[3]);
    zed_f9p_checksum_step(&checksum_a, &checksum_b, frame[4]);
    zed_f9p_checksum_step(&checksum_a, &checksum_b, frame[5]);

    for (index = 0u; index < payload_length; ++index)
    {
        frame[6u + index] = payload[index];
        zed_f9p_checksum_step(&checksum_a, &checksum_b, payload[index]);
    }

    write_len = (uint16_t)(payload_length + 8u);
    frame[write_len - 2u] = checksum_a;
    frame[write_len - 1u] = checksum_b;

    return device->uart->write(device->uart->context, frame, write_len);
}

fc_status_t zed_f9p_set_rate_15hz(zed_f9p_t *device)
{
    uint8_t rate_payload[6] = {66u, 0u, 1u, 0u, 1u, 0u};
    uint8_t msg_payload[8] = {UBX_CLASS_NAV, UBX_ID_NAV_PVT, 1u, 1u, 1u, 1u, 1u, 0u};

    if (device == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    if (zed_f9p_send_frame(device, UBX_CLASS_CFG, UBX_ID_CFG_RATE, rate_payload, sizeof(rate_payload)) !=
        FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    return zed_f9p_send_frame(device, UBX_CLASS_CFG, UBX_ID_CFG_MSG, msg_payload, sizeof(msg_payload));
}

fc_status_t zed_f9p_init(zed_f9p_t *device, fc_uart_bus_t *uart)
{
    if (device == NULL || uart == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    memset(device, 0, sizeof(*device));
    device->uart = uart;

    if (zed_f9p_set_rate_15hz(device) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    device->initialized = true;
    return FC_STATUS_OK;
}

fc_status_t zed_f9p_poll(zed_f9p_t *device, fc_gps_fix_t *fix)
{
    uint8_t byte = 0u;
    size_t received_len = 0u;
    fc_status_t status = FC_STATUS_NOT_READY;

    if (device == NULL || !device->initialized || device->uart == NULL || device->uart->read == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    while (device->uart->read(device->uart->context, &byte, 1u, 0u, &received_len) == FC_STATUS_OK &&
           received_len == 1u)
    {
        switch (device->parser_state)
        {
            case ZED_STATE_SYNC_1:
                if (byte == UBX_SYNC_CHAR_1)
                {
                    device->parser_state = ZED_STATE_SYNC_2;
                }
                break;

            case ZED_STATE_SYNC_2:
                device->parser_state = (byte == UBX_SYNC_CHAR_2) ? ZED_STATE_CLASS : ZED_STATE_SYNC_1;
                break;

            case ZED_STATE_CLASS:
                device->message_class = byte;
                device->checksum_a = 0u;
                device->checksum_b = 0u;
                zed_f9p_checksum_step(&device->checksum_a, &device->checksum_b, byte);
                device->parser_state = ZED_STATE_ID;
                break;

            case ZED_STATE_ID:
                device->message_id = byte;
                zed_f9p_checksum_step(&device->checksum_a, &device->checksum_b, byte);
                device->parser_state = ZED_STATE_LENGTH_1;
                break;

            case ZED_STATE_LENGTH_1:
                device->payload_length = byte;
                zed_f9p_checksum_step(&device->checksum_a, &device->checksum_b, byte);
                device->parser_state = ZED_STATE_LENGTH_2;
                break;

            case ZED_STATE_LENGTH_2:
                device->payload_length |= (uint16_t)((uint16_t)byte << 8);
                zed_f9p_checksum_step(&device->checksum_a, &device->checksum_b, byte);
                device->payload_index = 0u;
                if (device->payload_length > sizeof(device->payload))
                {
                    device->parser_state = ZED_STATE_SYNC_1;
                }
                else
                {
                    device->parser_state =
                        (device->payload_length == 0u) ? ZED_STATE_CHECKSUM_A : ZED_STATE_PAYLOAD;
                }
                break;

            case ZED_STATE_PAYLOAD:
                device->payload[device->payload_index++] = byte;
                zed_f9p_checksum_step(&device->checksum_a, &device->checksum_b, byte);
                if (device->payload_index >= device->payload_length)
                {
                    device->parser_state = ZED_STATE_CHECKSUM_A;
                }
                break;

            case ZED_STATE_CHECKSUM_A:
                if (byte == device->checksum_a)
                {
                    device->parser_state = ZED_STATE_CHECKSUM_B;
                }
                else
                {
                    device->parser_state = ZED_STATE_SYNC_1;
                }
                break;

            case ZED_STATE_CHECKSUM_B:
                if (byte == device->checksum_b && device->message_class == UBX_CLASS_NAV &&
                    device->message_id == UBX_ID_NAV_PVT && device->payload_length >= 92u)
                {
                    zed_f9p_parse_nav_pvt(device);
                    status = FC_STATUS_OK;
                }
                device->parser_state = ZED_STATE_SYNC_1;
                break;

            default:
                device->parser_state = ZED_STATE_SYNC_1;
                break;
        }
    }

    if (status == FC_STATUS_OK && fix != NULL)
    {
        *fix = device->last_fix;
    }

    return status;
}
