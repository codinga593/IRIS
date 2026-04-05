#include "icm42688p.h"

#include <math.h>
#include <string.h>

#define ICM42688P_REG_DEVICE_CONFIG 0x11u
#define ICM42688P_REG_TEMP_DATA1 0x1Du
#define ICM42688P_REG_PWR_MGMT0 0x4Eu
#define ICM42688P_REG_GYRO_CONFIG0 0x4Fu
#define ICM42688P_REG_ACCEL_CONFIG0 0x50u
#define ICM42688P_REG_WHO_AM_I 0x75u

#define ICM42688P_WHO_AM_I_VALUE 0x47u
#define DEG_TO_RAD 0.01745329251994329577f

static fc_status_t icm42688p_select(icm42688p_t *device, bool selected)
{
    if (device == NULL || device->bus == NULL || device->bus->select == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    return device->bus->select(device->bus->context, selected);
}

static fc_status_t icm42688p_transfer(icm42688p_t *device,
                                      const uint8_t *tx,
                                      uint8_t *rx,
                                      size_t len)
{
    fc_status_t status;

    if (device == NULL || device->bus == NULL || device->bus->transfer == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    status = icm42688p_select(device, true);
    if (status != FC_STATUS_OK)
    {
        return status;
    }

    status = device->bus->transfer(device->bus->context, tx, rx, len);
    (void)icm42688p_select(device, false);
    return status;
}

static void icm42688p_delay(icm42688p_t *device, uint32_t delay_ms)
{
    if (device != NULL && device->bus != NULL && device->bus->delay_ms != NULL)
    {
        device->bus->delay_ms(device->bus->context, delay_ms);
    }
}

static fc_status_t icm42688p_write_reg(icm42688p_t *device, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg & 0x7Fu, value};
    return icm42688p_transfer(device, tx, NULL, sizeof(tx));
}

static fc_status_t icm42688p_read_reg(icm42688p_t *device, uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = {(uint8_t)(reg | 0x80u), 0u};
    uint8_t rx[2] = {0};
    fc_status_t status = icm42688p_transfer(device, tx, rx, sizeof(tx));

    if (status == FC_STATUS_OK && value != NULL)
    {
        *value = rx[1];
    }

    return status;
}

static fc_status_t icm42688p_read_block(icm42688p_t *device, uint8_t reg, uint8_t *buffer, size_t len)
{
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};
    size_t index = 0u;
    fc_status_t status;

    if (buffer == NULL || len == 0u || (len + 1u) > sizeof(tx))
    {
        return FC_STATUS_INVALID_ARG;
    }

    tx[0] = (uint8_t)(reg | 0x80u);
    status = icm42688p_transfer(device, tx, rx, len + 1u);
    if (status != FC_STATUS_OK)
    {
        return status;
    }

    for (index = 0u; index < len; ++index)
    {
        buffer[index] = rx[index + 1u];
    }

    return FC_STATUS_OK;
}

static int16_t icm42688p_be16(const uint8_t *buffer)
{
    return (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
}

fc_status_t icm42688p_init(icm42688p_t *device, fc_spi_bus_t *bus)
{
    uint8_t who_am_i = 0u;

    if (device == NULL || bus == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    memset(device, 0, sizeof(*device));
    device->bus = bus;

    if (icm42688p_read_reg(device, ICM42688P_REG_WHO_AM_I, &who_am_i) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    if (who_am_i != ICM42688P_WHO_AM_I_VALUE)
    {
        return FC_STATUS_NOT_READY;
    }

    if (icm42688p_write_reg(device, ICM42688P_REG_DEVICE_CONFIG, 0x01u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    icm42688p_delay(device, 5u);

    if (icm42688p_write_reg(device, ICM42688P_REG_PWR_MGMT0, 0x0Fu) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (icm42688p_write_reg(device, ICM42688P_REG_GYRO_CONFIG0, 0x06u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }
    if (icm42688p_write_reg(device, ICM42688P_REG_ACCEL_CONFIG0, 0x06u) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    device->initialized = true;
    return FC_STATUS_OK;
}

fc_status_t icm42688p_read_raw(icm42688p_t *device, fc_imu_sample_t *sample)
{
    uint8_t raw[14] = {0};
    int16_t raw_ax;
    int16_t raw_ay;
    int16_t raw_az;
    int16_t raw_gx;
    int16_t raw_gy;
    int16_t raw_gz;

    if (device == NULL || sample == NULL || !device->initialized)
    {
        return FC_STATUS_INVALID_ARG;
    }

    if (icm42688p_read_block(device, ICM42688P_REG_TEMP_DATA1, raw, sizeof(raw)) != FC_STATUS_OK)
    {
        return FC_STATUS_ERROR;
    }

    raw_ax = icm42688p_be16(&raw[2]);
    raw_ay = icm42688p_be16(&raw[4]);
    raw_az = icm42688p_be16(&raw[6]);
    raw_gx = icm42688p_be16(&raw[8]);
    raw_gy = icm42688p_be16(&raw[10]);
    raw_gz = icm42688p_be16(&raw[12]);

    sample->accel_mps2.x = ((float)raw_ax / 2048.0f) * 9.80665f;
    sample->accel_mps2.y = ((float)raw_ay / 2048.0f) * 9.80665f;
    sample->accel_mps2.z = ((float)raw_az / 2048.0f) * 9.80665f;
    sample->gyro_rps.x = ((float)raw_gx / 16.4f) * DEG_TO_RAD;
    sample->gyro_rps.y = ((float)raw_gy / 16.4f) * DEG_TO_RAD;
    sample->gyro_rps.z = ((float)raw_gz / 16.4f) * DEG_TO_RAD;
    sample->attitude_valid = false;
    sample->valid = true;
    device->last_sample = *sample;
    return FC_STATUS_OK;
}
