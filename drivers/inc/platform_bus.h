#ifndef PLATFORM_BUS_H
#define PLATFORM_BUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    FC_STATUS_OK = 0,
    FC_STATUS_ERROR,
    FC_STATUS_TIMEOUT,
    FC_STATUS_INVALID_ARG,
    FC_STATUS_NOT_READY,
    FC_STATUS_CRC_ERROR,
    FC_STATUS_UNSUPPORTED
} fc_status_t;

typedef struct
{
    void *context;
    fc_status_t (*write)(void *context, uint8_t address, const uint8_t *tx, size_t tx_len);
    fc_status_t (*read)(void *context, uint8_t address, uint8_t *rx, size_t rx_len);
    fc_status_t (*write_read)(void *context,
                              uint8_t address,
                              const uint8_t *tx,
                              size_t tx_len,
                              uint8_t *rx,
                              size_t rx_len);
    void (*delay_ms)(void *context, uint32_t delay_ms);
} fc_i2c_bus_t;

typedef struct
{
    void *context;
    fc_status_t (*select)(void *context, bool selected);
    fc_status_t (*transfer)(void *context, const uint8_t *tx, uint8_t *rx, size_t len);
    void (*delay_ms)(void *context, uint32_t delay_ms);
} fc_spi_bus_t;

typedef struct
{
    void *context;
    fc_status_t (*write)(void *context, const uint8_t *tx, size_t len);
    fc_status_t (*read)(void *context,
                        uint8_t *rx,
                        size_t len,
                        uint32_t timeout_ms,
                        size_t *received_len);
} fc_uart_bus_t;

typedef struct
{
    uint32_t id;
    bool extended_id;
    uint8_t data[64];
    size_t data_len;
} fc_can_frame_t;

typedef struct
{
    void *context;
    fc_status_t (*send)(void *context, const fc_can_frame_t *frame);
} fc_can_bus_t;

typedef struct
{
    void *context;
    fc_status_t (*set_normalized)(void *context, uint8_t channel, float duty_0_to_1);
} fc_pwm_bus_t;

typedef struct
{
    void *context;
    uint32_t (*millis)(void *context);
} fc_time_source_t;

typedef struct
{
    void *context;
    fc_status_t (*set_armed)(void *context, bool armed);
    bool (*continuity_ok)(void *context);
    fc_status_t (*fire_channel)(void *context, uint8_t channel);
} fc_pyro_interface_t;

#endif
