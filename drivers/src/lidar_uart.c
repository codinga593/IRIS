#include "lidar_uart.h"

#include <stdlib.h>
#include <string.h>

fc_status_t lidar_uart_init(lidar_uart_t *device, fc_uart_bus_t *uart)
{
    if (device == NULL || uart == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    memset(device, 0, sizeof(*device));
    device->uart = uart;
    return FC_STATUS_OK;
}

fc_status_t lidar_uart_poll(lidar_uart_t *device, float *range_m)
{
    uint8_t buffer[16] = {0};
    size_t received_len = 0u;
    size_t index = 0u;

    if (device == NULL || device->uart == NULL || device->uart->read == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    if (device->uart->read(device->uart->context, buffer, sizeof(buffer), 0u, &received_len) !=
            FC_STATUS_OK ||
        received_len == 0u)
    {
        return FC_STATUS_NOT_READY;
    }

    for (index = 0u; index < received_len; ++index)
    {
        char ch = (char)buffer[index];

        if (ch == '\r' || ch == '\n')
        {
            if (device->line_length > 0u)
            {
                float value;

                device->line_buffer[device->line_length] = '\0';
                value = strtof(device->line_buffer, NULL);
                device->last_range_m = (value > 100.0f) ? (value / 1000.0f) : value;
                device->line_length = 0u;
                device->valid = true;

                if (range_m != NULL)
                {
                    *range_m = device->last_range_m;
                }
                return FC_STATUS_OK;
            }
        }
        else if (device->line_length < (sizeof(device->line_buffer) - 1u))
        {
            device->line_buffer[device->line_length++] = ch;
        }
    }

    return FC_STATUS_NOT_READY;
}
