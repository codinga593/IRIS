#ifndef ZED_F9P_H
#define ZED_F9P_H

#include "flight_types.h"
#include "platform_bus.h"

typedef struct
{
    fc_uart_bus_t *uart;
    fc_gps_fix_t last_fix;
    uint8_t parser_state;
    uint8_t message_class;
    uint8_t message_id;
    uint16_t payload_length;
    uint16_t payload_index;
    uint8_t payload[128];
    uint8_t checksum_a;
    uint8_t checksum_b;
    bool origin_set;
    double origin_latitude_deg;
    double origin_longitude_deg;
    float origin_altitude_m;
    bool initialized;
} zed_f9p_t;

fc_status_t zed_f9p_init(zed_f9p_t *device, fc_uart_bus_t *uart);
fc_status_t zed_f9p_set_rate_15hz(zed_f9p_t *device);
fc_status_t zed_f9p_poll(zed_f9p_t *device, fc_gps_fix_t *fix);

#endif
