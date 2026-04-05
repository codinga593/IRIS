#ifndef LIDAR_UART_H
#define LIDAR_UART_H

#include "platform_bus.h"

typedef struct
{
    fc_uart_bus_t *uart;
    char line_buffer[32];
    size_t line_length;
    float last_range_m;
    bool valid;
} lidar_uart_t;

fc_status_t lidar_uart_init(lidar_uart_t *device, fc_uart_bus_t *uart);
fc_status_t lidar_uart_poll(lidar_uart_t *device, float *range_m);

#endif
