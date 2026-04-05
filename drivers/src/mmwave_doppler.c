#include "mmwave_doppler.h"

#include <string.h>

/*
 * The mmWave module was not specified beyond "doppler on SPI Bus 4", so this
 * driver keeps the transport contract clean and uses a compact placeholder frame.
 * Replace the command and byte decoding below with the actual module register map.
 */

fc_status_t mmwave_doppler_init(mmwave_doppler_t *device, fc_spi_bus_t *bus)
{
    if (device == NULL || bus == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    memset(device, 0, sizeof(*device));
    device->bus = bus;
    device->initialized = true;
    return FC_STATUS_OK;
}

fc_status_t mmwave_doppler_read(mmwave_doppler_t *device, mmwave_measurement_t *measurement)
{
    uint8_t tx[8] = {0xAAu, 0x55u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};
    uint8_t rx[8] = {0};
    int16_t raw_velocity;
    uint16_t raw_range;
    fc_status_t status;

    if (device == NULL || measurement == NULL || !device->initialized || device->bus == NULL ||
        device->bus->select == NULL || device->bus->transfer == NULL)
    {
        return FC_STATUS_INVALID_ARG;
    }

    status = device->bus->select(device->bus->context, true);
    if (status != FC_STATUS_OK)
    {
        return status;
    }

    status = device->bus->transfer(device->bus->context, tx, rx, sizeof(tx));
    (void)device->bus->select(device->bus->context, false);
    if (status != FC_STATUS_OK)
    {
        return status;
    }

    raw_velocity = (int16_t)(((uint16_t)rx[2] << 8) | rx[3]);
    raw_range = (uint16_t)(((uint16_t)rx[4] << 8) | rx[5]);

    measurement->radial_velocity_mps = (float)raw_velocity / 100.0f;
    measurement->range_m = (float)raw_range / 100.0f;
    measurement->valid = true;
    device->last_measurement = *measurement;
    return FC_STATUS_OK;
}
