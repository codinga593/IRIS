#include "lora_radio.h"

#include <string.h>

#define LORA_REG_FIFO 0x00u
#define LORA_REG_OP_MODE 0x01u
#define LORA_REG_FRF_MSB 0x06u
#define LORA_REG_FRF_MID 0x07u
#define LORA_REG_FRF_LSB 0x08u
#define LORA_REG_PA_CONFIG 0x09u
#define LORA_REG_FIFO_ADDR_PTR 0x0Du
#define LORA_REG_FIFO_TX_BASE_ADDR 0x0Eu
#define LORA_REG_FIFO_RX_BASE_ADDR 0x0Fu
#define LORA_REG_FIFO_RX_CURRENT_ADDR 0x10u
#define LORA_REG_IRQ_FLAGS 0x12u
#define LORA_REG_RX_NB_BYTES 0x13u
#define LORA_REG_MODEM_CONFIG1 0x1Du
#define LORA_REG_MODEM_CONFIG2 0x1Eu
#define LORA_REG_PREAMBLE_MSB 0x20u
#define LORA_REG_PREAMBLE_LSB 0x21u
#define LORA_REG_PAYLOAD_LENGTH 0x22u
#define LORA_REG_MODEM_CONFIG3 0x26u
#define LORA_REG_SYNC_WORD 0x39u
#define LORA_REG_DIO_MAPPING1 0x40u

#define LORA_MODE_SLEEP 0x80u
#define LORA_MODE_STANDBY 0x81u
#define LORA_MODE_TX 0x83u
#define LORA_MODE_RX_CONTINUOUS 0x85u

static fc_status_t lora_select(lora_radio_t *device, bool selected)
{
    if (device == NULL || device->bus == NULL || device->bus->select == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    return device->bus->select(device->bus->context, selected);
}

static fc_status_t lora_transfer(lora_radio_t *device, const uint8_t *tx, uint8_t *rx, size_t len)
{
    fc_status_t status;

    if (device == NULL || device->bus == NULL || device->bus->transfer == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    status = lora_select(device, true);
    if (status != FC_STATUS_OK)
    {
        return status;
    }

    status = device->bus->transfer(device->bus->context, tx, rx, len);
    (void)lora_select(device, false);
    return status;
}

static void lora_delay(lora_radio_t *device, uint32_t delay_ms)
{
    if (device != NULL && device->bus != NULL && device->bus->delay_ms != NULL)
    {
        device->bus->delay_ms(device->bus->context, delay_ms);
    }
}

static fc_status_t lora_write_reg(lora_radio_t *device, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg | 0x80u, value};
    return lora_transfer(device, tx, NULL, sizeof(tx));
}

static fc_status_t lora_read_reg(lora_radio_t *device, uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = {reg & 0x7Fu, 0u};
    uint8_t rx[2] = {0};
    fc_status_t status = lora_transfer(device, tx, rx, sizeof(tx));

    if (status == FC_STATUS_OK && value != NULL)
    {
        *value = rx[1];
    }

    return status;
}

static fc_status_t lora_write_fifo(lora_radio_t *device, const uint8_t *payload, size_t len)
{
    fc_status_t status;

    if (payload == NULL || len > 255u)
    {
        return FC_STATUS_INVALID_ARG;
    }

    status = lora_select(device, true);
    if (status != FC_STATUS_OK)
    {
        return status;
    }

    {
        uint8_t reg = LORA_REG_FIFO | 0x80u;
        status = device->bus->transfer(device->bus->context, &reg, NULL, 1u);
    }

    if (status == FC_STATUS_OK)
    {
        status = device->bus->transfer(device->bus->context, payload, NULL, len);
    }

    (void)lora_select(device, false);
    return status;
}

static fc_status_t lora_read_fifo(lora_radio_t *device, uint8_t *payload, size_t len)
{
    fc_status_t status;

    if (payload == NULL || len > 255u)
    {
        return FC_STATUS_INVALID_ARG;
    }

    status = lora_select(device, true);
    if (status != FC_STATUS_OK)
    {
        return status;
    }

    {
        uint8_t reg = LORA_REG_FIFO & 0x7Fu;
        status = device->bus->transfer(device->bus->context, &reg, NULL, 1u);
    }

    if (status == FC_STATUS_OK)
    {
        uint8_t zeros[255] = {0};
        status = device->bus->transfer(device->bus->context, zeros, payload, len);
    }

    (void)lora_select(device, false);
    return status;
}

static fc_status_t lora_set_frequency(lora_radio_t *device, uint32_t frequency_hz)
{
    uint64_t frf = ((uint64_t)frequency_hz << 19) / 32000000ull;

    if (lora_write_reg(device, LORA_REG_FRF_MSB, (uint8_t)((frf >> 16) & 0xFFu)) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_FRF_MID, (uint8_t)((frf >> 8) & 0xFFu)) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    return lora_write_reg(device, LORA_REG_FRF_LSB, (uint8_t)(frf & 0xFFu));
}

fc_status_t lora_radio_init(lora_radio_t *device, fc_spi_bus_t *bus, uint32_t frequency_hz)
{
    if (device == NULL || bus == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    memset(device, 0, sizeof(*device));
    device->bus = bus;
    device->frequency_hz = frequency_hz;

    if (lora_write_reg(device, LORA_REG_OP_MODE, LORA_MODE_SLEEP) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    lora_delay(device, 5u);

    if (lora_set_frequency(device, frequency_hz) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_FIFO_TX_BASE_ADDR, 0x00u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_FIFO_RX_BASE_ADDR, 0x00u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_MODEM_CONFIG1, 0x72u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_MODEM_CONFIG2, 0x74u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_MODEM_CONFIG3, 0x04u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_PREAMBLE_MSB, 0x00u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_PREAMBLE_LSB, 0x08u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_SYNC_WORD, 0x12u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_PA_CONFIG, 0x8Fu) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_DIO_MAPPING1, 0x00u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_OP_MODE, LORA_MODE_RX_CONTINUOUS) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    device->initialized = true;
    return FC_STATUS_OK;
}

fc_status_t lora_radio_transmit(lora_radio_t *device, const uint8_t *payload, size_t len)
{
    uint8_t irq_flags = 0u;
    uint32_t retries = 20u;

    if (device == NULL || payload == NULL || len == 0u || len > 255u || !device->initialized)
    {
        return FC_STATUS_INVALID_ARG;
    }

    if (lora_write_reg(device, LORA_REG_OP_MODE, LORA_MODE_STANDBY) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_FIFO_ADDR_PTR, 0x00u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_fifo(device, payload, len) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_PAYLOAD_LENGTH, (uint8_t)len) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_IRQ_FLAGS, 0xFFu) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_OP_MODE, LORA_MODE_TX) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    while (retries-- > 0u)
    {
        if (lora_read_reg(device, LORA_REG_IRQ_FLAGS, &irq_flags) != FC_STATUS_OK)
        {
            return FC_STATUS_ERROR;
        }

        if ((irq_flags & 0x08u) != 0u)
        {
            break;
        }

        lora_delay(device, 2u);
    }

    if (lora_write_reg(device, LORA_REG_IRQ_FLAGS, 0xFFu) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    return lora_write_reg(device, LORA_REG_OP_MODE, LORA_MODE_RX_CONTINUOUS);
}

fc_status_t lora_radio_receive(lora_radio_t *device, uint8_t *payload, size_t max_len, size_t *actual_len)
{
    uint8_t irq_flags = 0u;
    uint8_t bytes = 0u;
    uint8_t current_addr = 0u;

    if (actual_len != NULL)
    {
        *actual_len = 0u;
    }

    if (device == NULL || payload == NULL || max_len == 0u || !device->initialized)
    {
        return FC_STATUS_INVALID_ARG;
    }

    if (lora_read_reg(device, LORA_REG_IRQ_FLAGS, &irq_flags) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    if ((irq_flags & 0x40u) == 0u)
    {
        return FC_STATUS_NOT_READY;
    }

    if ((irq_flags & 0x20u) != 0u)
    {
        (void)lora_write_reg(device, LORA_REG_IRQ_FLAGS, 0xFFu);
        return FC_STATUS_CRC_ERROR;
    }

    if (lora_read_reg(device, LORA_REG_RX_NB_BYTES, &bytes) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_read_reg(device, LORA_REG_FIFO_RX_CURRENT_ADDR, &current_addr) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (lora_write_reg(device, LORA_REG_FIFO_ADDR_PTR, current_addr) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if ((size_t)bytes > max_len)
    {
        bytes = (uint8_t)max_len;
    }
    if (lora_read_fifo(device, payload, bytes) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    if (actual_len != NULL)
    {
        *actual_len = bytes;
    }

    return lora_write_reg(device, LORA_REG_IRQ_FLAGS, 0xFFu);
}

fc_lora_command_t lora_radio_decode_command(const uint8_t *payload, size_t len)
{
    fc_lora_command_t command = {FC_LORA_CMD_NONE, false};

    if (payload == NULL || len == 0u)
    {
        return command;
    }

    if (len >= 3u && memcmp(payload, "ARM", 3u) == 0)
    {
        command.id = FC_LORA_CMD_ARM;
        command.valid = true;
    }
    else if (len >= 6u && memcmp(payload, "DISARM", 6u) == 0)
    {
        command.id = FC_LORA_CMD_DISARM;
        command.valid = true;
    }
    else if (len >= 5u && memcmp(payload, "ABORT", 5u) == 0)
    {
        command.id = FC_LORA_CMD_ABORT;
        command.valid = true;
    }

    return command;
}
