#ifndef W25Q128_H
#define W25Q128_H

#include "platform_bus.h"

typedef struct
{
    fc_spi_bus_t *bus;
    uint32_t next_log_address;
    bool initialized;
} w25q128_t;

fc_status_t w25q128_init(w25q128_t *device, fc_spi_bus_t *bus);
fc_status_t w25q128_read(w25q128_t *device, uint32_t address, uint8_t *buffer, size_t len);
fc_status_t w25q128_erase_sector(w25q128_t *device, uint32_t address);
fc_status_t w25q128_page_program(w25q128_t *device,
                                 uint32_t address,
                                 const uint8_t *buffer,
                                 size_t len);

#endif
