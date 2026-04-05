#ifndef LORA_RADIO_H
#define LORA_RADIO_H

#include "flight_types.h"
#include "platform_bus.h"

typedef struct
{
    fc_spi_bus_t *bus;
    uint32_t frequency_hz;
    bool initialized;
} lora_radio_t;

fc_status_t lora_radio_init(lora_radio_t *device, fc_spi_bus_t *bus, uint32_t frequency_hz);
fc_status_t lora_radio_transmit(lora_radio_t *device, const uint8_t *payload, size_t len);
fc_status_t lora_radio_receive(lora_radio_t *device, uint8_t *payload, size_t max_len, size_t *actual_len);
fc_lora_command_t lora_radio_decode_command(const uint8_t *payload, size_t len);

#endif
