#include "w25q128.h"

#include <string.h>

#define W25Q128_CMD_JEDEC_ID 0x9Fu
#define W25Q128_CMD_READ_DATA 0x03u
#define W25Q128_CMD_WRITE_ENABLE 0x06u
#define W25Q128_CMD_SECTOR_ERASE_4K 0x20u
#define W25Q128_CMD_PAGE_PROGRAM 0x02u
#define W25Q128_CMD_READ_STATUS_1 0x05u

static fc_status_t w25q128_select(w25q128_t *device, bool selected)
{
    if (device == NULL || device->bus == NULL || device->bus->select == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    return device->bus->select(device->bus->context, selected);
}

static fc_status_t w25q128_transfer(w25q128_t *device,
                                    const uint8_t *tx,
                                    uint8_t *rx,
                                    size_t len)
{
    fc_status_t status;

    if (device == NULL || device->bus == NULL || device->bus->transfer == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    status = w25q128_select(device, true);
    if (status != FC_STATUS_OK)
    {
        return status;
    }

    status = device->bus->transfer(device->bus->context, tx, rx, len);
    (void)w25q128_select(device, false);
    return status;
}

static void w25q128_delay(w25q128_t *device, uint32_t delay_ms)
{
    if (device != NULL && device->bus != NULL && device->bus->delay_ms != NULL)
    {
        device->bus->delay_ms(device->bus->context, delay_ms);
    }
}

static fc_status_t w25q128_write_enable(w25q128_t *device)
{
    uint8_t command = W25Q128_CMD_WRITE_ENABLE;
    return w25q128_transfer(device, &command, NULL, 1u);
}

static fc_status_t w25q128_read_status_1(w25q128_t *device, uint8_t *status_reg)
{
    uint8_t tx[2] = {W25Q128_CMD_READ_STATUS_1, 0u};
    uint8_t rx[2] = {0};
    fc_status_t status = w25q128_transfer(device, tx, rx, sizeof(tx));

    if (status == FC_STATUS_OK && status_reg != NULL)
    {
        *status_reg = rx[1];
    }

    return status;
}

static fc_status_t w25q128_wait_busy(w25q128_t *device)
{
    uint8_t status_reg = 0u;
    uint32_t retries = 100u;

    while (retries-- > 0u)
    {
        if (w25q128_read_status_1(device, &status_reg) != FC_STATUS_OK)
        {
            return FC_STATUS_ERROR;
        }

        if ((status_reg & 0x01u) == 0u)
        {
            return FC_STATUS_OK;
        }

        w25q128_delay(device, 2u);
    }

    return FC_STATUS_TIMEOUT;
}

fc_status_t w25q128_init(w25q128_t *device, fc_spi_bus_t *bus)
{
    uint8_t tx[4] = {W25Q128_CMD_JEDEC_ID, 0u, 0u, 0u};
    uint8_t rx[4] = {0};

    if (device == NULL || bus == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    memset(device, 0, sizeof(*device));
    device->bus = bus;

    if (w25q128_transfer(device, tx, rx, sizeof(tx)) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    if (!(rx[1] == 0xEFu && rx[2] == 0x40u && rx[3] == 0x18u))
    {
        return FC_STATUS_NOT_READY;
    }

    device->initialized = true;
    return FC_STATUS_OK;
}

fc_status_t w25q128_read(w25q128_t *device, uint32_t address, uint8_t *buffer, size_t len)
{
    uint8_t header[4] = {
        W25Q128_CMD_READ_DATA,
        (uint8_t)((address >> 16) & 0xFFu),
        (uint8_t)((address >> 8) & 0xFFu),
        (uint8_t)(address & 0xFFu)};
    fc_status_t status;
    size_t index;

    if (device == NULL || buffer == NULL || !device->initialized)
    {
        return FC_STATUS_INVALID_ARG;
    }

    status = w25q128_select(device, true);
    if (status != FC_STATUS_OK)
    {
        return status;
    }

    status = device->bus->transfer(device->bus->context, header, NULL, sizeof(header));
    if (status == FC_STATUS_OK)
    {
        uint8_t zeros[256] = {0};
        uint8_t rx[256] = {0};

        if (len > sizeof(zeros))
        {
            (void)w25q128_select(device, false);
            return FC_STATUS_INVALID_ARG;
        }

        status = device->bus->transfer(device->bus->context, zeros, rx, len);
        if (status == FC_STATUS_OK)
        {
            for (index = 0u; index < len; ++index)
            {
                buffer[index] = rx[index];
            }
        }
    }

    (void)w25q128_select(device, false);
    return status;
}

fc_status_t w25q128_erase_sector(w25q128_t *device, uint32_t address)
{
    uint8_t command[4] = {W25Q128_CMD_SECTOR_ERASE_4K,
                          (uint8_t)((address >> 16) & 0xFFu),
                          (uint8_t)((address >> 8) & 0xFFu),
                          (uint8_t)(address & 0xFFu)};

    if (device == NULL || !device->initialized)
    {
        return FC_STATUS_INVALID_ARG;
    }

    if (w25q128_write_enable(device) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    if (w25q128_transfer(device, command, NULL, sizeof(command)) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    return w25q128_wait_busy(device);
}

fc_status_t w25q128_page_program(w25q128_t *device,
                                 uint32_t address,
                                 const uint8_t *buffer,
                                 size_t len)
{
    uint8_t header[4] = {W25Q128_CMD_PAGE_PROGRAM,
                         (uint8_t)((address >> 16) & 0xFFu),
                         (uint8_t)((address >> 8) & 0xFFu),
                         (uint8_t)(address & 0xFFu)};
    fc_status_t status;

    if (device == NULL || buffer == NULL || !device->initialized || len == 0u || len > 256u)
    {
        return FC_STATUS_INVALID_ARG;
    }

    if (w25q128_write_enable(device) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    status = w25q128_select(device, true);
    if (status != FC_STATUS_OK)
    {
        return status;
    }

    status = device->bus->transfer(device->bus->context, header, NULL, sizeof(header));
    if (status == FC_STATUS_OK)
    {
        status = device->bus->transfer(device->bus->context, buffer, NULL, len);
    }

    (void)w25q128_select(device, false);
    if (status != FC_STATUS_OK)
    {
        return status;
    }

    return w25q128_wait_busy(device);
}
