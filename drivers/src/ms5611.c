#include "ms5611.h"

#include <math.h>
#include <string.h>

#define MS5611_CMD_RESET 0x1Eu
#define MS5611_CMD_ADC_READ 0x00u
#define MS5611_CMD_CONVERT_D1_4096 0x48u
#define MS5611_CMD_CONVERT_D2_4096 0x58u
#define MS5611_CMD_PROM_READ_BASE 0xA0u

static fc_status_t ms5611_read_bytes(ms5611_t *device,
                                     const uint8_t *tx,
                                     size_t tx_len,
                                     uint8_t *rx,
                                     size_t rx_len)
{
    if (device == NULL || device->bus == NULL || device->bus->write_read == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    return device->bus->write_read(device->bus->context, device->address, tx, tx_len, rx, rx_len);
}

static fc_status_t ms5611_write_byte(ms5611_t *device, uint8_t value)
{
    if (device == NULL || device->bus == NULL || device->bus->write == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    return device->bus->write(device->bus->context, device->address, &value, 1u);
}

static void ms5611_delay(ms5611_t *device, uint32_t delay_ms)
{
    if (device != NULL && device->bus != NULL && device->bus->delay_ms != NULL)
    {
        device->bus->delay_ms(device->bus->context, delay_ms);
    }
}

static fc_status_t ms5611_read_adc(ms5611_t *device, uint32_t *value)
{
    uint8_t command = MS5611_CMD_ADC_READ;
    uint8_t raw[3] = {0};
    fc_status_t status = ms5611_read_bytes(device, &command, 1u, raw, sizeof(raw));

    if (status != FC_STATUS_OK)
    {
        return status;
    }

    *value = ((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | raw[2];
    return FC_STATUS_OK;
}

fc_status_t ms5611_init(ms5611_t *device, fc_i2c_bus_t *bus, uint8_t address)
{
    uint8_t command = 0u;
    uint8_t raw[2] = {0};
    size_t index = 0u;

    if (device == NULL || bus == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    memset(device, 0, sizeof(*device));
    device->bus = bus;
    device->address = address;

    if (ms5611_write_byte(device, MS5611_CMD_RESET) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    ms5611_delay(device, 5u);

    for (index = 0u; index < 8u; ++index)
    {
        command = (uint8_t)(MS5611_CMD_PROM_READ_BASE + (uint8_t)(index * 2u));
        if (ms5611_read_bytes(device, &command, 1u, raw, sizeof(raw)) != FC_STATUS_OK)
        {
            return FC_STATUS_ERROR;
        }

        device->prom[index] = (uint16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    }

    device->initialized = true;
    return FC_STATUS_OK;
}

fc_status_t ms5611_read(ms5611_t *device, fc_baro_sample_t *sample)
{
    uint32_t d1 = 0u;
    uint32_t d2 = 0u;
    int64_t d_t = 0;
    int64_t temperature = 0;
    int64_t offset = 0;
    int64_t sensitivity = 0;
    int64_t temperature2 = 0;
    int64_t offset2 = 0;
    int64_t sensitivity2 = 0;
    int64_t pressure = 0;

    if (device == NULL || sample == NULL || !device->initialized)
    {
        return FC_STATUS_INVALID_ARG;
    }

    if (ms5611_write_byte(device, MS5611_CMD_CONVERT_D1_4096) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    ms5611_delay(device, 10u);
    if (ms5611_read_adc(device, &d1) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    if (ms5611_write_byte(device, MS5611_CMD_CONVERT_D2_4096) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    ms5611_delay(device, 10u);
    if (ms5611_read_adc(device, &d2) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    d_t = (int64_t)d2 - ((int64_t)device->prom[5] << 8);
    temperature = 2000 + ((d_t * (int64_t)device->prom[6]) >> 23);
    offset = ((int64_t)device->prom[2] << 16) + (((int64_t)device->prom[4] * d_t) >> 7);
    sensitivity = ((int64_t)device->prom[1] << 15) + (((int64_t)device->prom[3] * d_t) >> 8);

    if (temperature < 2000)
    {
        temperature2 = (d_t * d_t) >> 31;
        offset2 = (5 * (temperature - 2000) * (temperature - 2000)) >> 1;
        sensitivity2 = (5 * (temperature - 2000) * (temperature - 2000)) >> 2;

        if (temperature < -1500)
        {
            offset2 += 7 * (temperature + 1500) * (temperature + 1500);
            sensitivity2 += (11 * (temperature + 1500) * (temperature + 1500)) >> 1;
        }

        temperature -= temperature2;
        offset -= offset2;
        sensitivity -= sensitivity2;
    }

    pressure = (((((int64_t)d1 * sensitivity) >> 21) - offset) >> 15);

    sample->pressure_pa = (float)pressure * 100.0f;
    sample->temperature_c = (float)temperature / 100.0f;
    sample->altitude_m = 44330.0f * (1.0f - powf(sample->pressure_pa / 101325.0f, 0.19029495f));
    sample->valid = true;
    return FC_STATUS_OK;
}
