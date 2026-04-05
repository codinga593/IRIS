#include "flight_computer.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Replace these mock transports with your actual STM32 HAL bindings.
 * They exist so the project has a complete entry point and host-side syntax checks.
 */

static fc_status_t mock_i2c_write(void *context, uint8_t address, const uint8_t *tx, size_t tx_len)
{
    (void)context;
    (void)address;
    (void)tx;
    (void)tx_len;
    return FC_STATUS_OK;
}

static fc_status_t mock_i2c_read(void *context, uint8_t address, uint8_t *rx, size_t rx_len)
{
    size_t index;
    (void)context;
    (void)address;

    for (index = 0u; index < rx_len; ++index)
    {
        rx[index] = 0u;
    }
    return FC_STATUS_OK;
}

static fc_status_t mock_i2c_write_read(void *context,
                                       uint8_t address,
                                       const uint8_t *tx,
                                       size_t tx_len,
                                       uint8_t *rx,
                                       size_t rx_len)
{
    (void)tx;
    (void)tx_len;
    return mock_i2c_read(context, address, rx, rx_len);
}

static void mock_delay_ms(void *context, uint32_t delay_ms)
{
    (void)context;
    (void)delay_ms;
}

static fc_status_t mock_spi_select(void *context, bool selected)
{
    (void)context;
    (void)selected;
    return FC_STATUS_OK;
}

static fc_status_t mock_spi_transfer(void *context, const uint8_t *tx, uint8_t *rx, size_t len)
{
    size_t index;
    (void)context;
    (void)tx;

    if (rx != NULL)
    {
        for (index = 0u; index < len; ++index)
        {
            rx[index] = 0u;
        }
    }
    return FC_STATUS_OK;
}

static fc_status_t mock_uart_write(void *context, const uint8_t *tx, size_t len)
{
    (void)context;
    (void)tx;
    (void)len;
    return FC_STATUS_OK;
}

static fc_status_t mock_uart_read(void *context,
                                  uint8_t *rx,
                                  size_t len,
                                  uint32_t timeout_ms,
                                  size_t *received_len)
{
    size_t index;
    (void)context;
    (void)timeout_ms;

    for (index = 0u; index < len; ++index)
    {
        rx[index] = 0u;
    }

    if (received_len != NULL)
    {
        *received_len = 0u;
    }
    return FC_STATUS_OK;
}

static fc_status_t mock_can_send(void *context, const fc_can_frame_t *frame)
{
    (void)context;
    (void)frame;
    return FC_STATUS_OK;
}

static fc_status_t mock_pwm_set(void *context, uint8_t channel, float duty_0_to_1)
{
    (void)context;
    (void)channel;
    (void)duty_0_to_1;
    return FC_STATUS_OK;
}

static uint32_t mock_millis(void *context)
{
    static uint32_t time_ms = 0u;
    (void)context;
    time_ms += 10u;
    return time_ms;
}

static fc_status_t mock_pyro_set_armed(void *context, bool armed)
{
    (void)context;
    (void)armed;
    return FC_STATUS_OK;
}

static bool mock_pyro_continuity_ok(void *context)
{
    (void)context;
    return true;
}

static fc_status_t mock_pyro_fire_channel(void *context, uint8_t channel)
{
    (void)context;
    (void)channel;
    return FC_STATUS_OK;
}

int main(void)
{
    fc_i2c_bus_t i2c_bus = {0};
    fc_spi_bus_t spi_bus = {0};
    fc_uart_bus_t uart_bus = {0};
    fc_can_bus_t can_bus = {0};
    fc_pwm_bus_t pwm_bus = {0};
    fc_time_source_t time_source = {0};
    fc_pyro_interface_t pyro = {0};
    fc_board_config_t board = {0};
    fc_flight_computer_t computer = {0};

    i2c_bus.write = mock_i2c_write;
    i2c_bus.read = mock_i2c_read;
    i2c_bus.write_read = mock_i2c_write_read;
    i2c_bus.delay_ms = mock_delay_ms;

    spi_bus.select = mock_spi_select;
    spi_bus.transfer = mock_spi_transfer;
    spi_bus.delay_ms = mock_delay_ms;

    uart_bus.write = mock_uart_write;
    uart_bus.read = mock_uart_read;

    can_bus.send = mock_can_send;
    pwm_bus.set_normalized = mock_pwm_set;
    time_source.millis = mock_millis;
    pyro.set_armed = mock_pyro_set_armed;
    pyro.continuity_ok = mock_pyro_continuity_ok;
    pyro.fire_channel = mock_pyro_fire_channel;

    board.baro_i2c = &i2c_bus;
    board.imu1_i2c = &i2c_bus;
    board.imu2_spi = &spi_bus;
    board.lidar_uart = &uart_bus;
    board.fdcan = &can_bus;
    board.gps_uart = &uart_bus;
    board.rpi_uart = &uart_bus;
    board.cm5_uart = &uart_bus;
    board.fin_uart = &uart_bus;
    board.pwm = &pwm_bus;
    board.lora_spi = &spi_bus;
    board.flash_spi = &spi_bus;
    board.mmwave_spi = &spi_bus;
    board.pyro = &pyro;
    board.time = &time_source;

    if (FlightComputer_Init(&computer, &board) == FC_STATUS_OK)
    {
        (void)FlightComputer_Step(&computer);
    }

    return 0;
}
