#ifndef MMWAVE_DOPPLER_H
#define MMWAVE_DOPPLER_H

#include "platform_bus.h"

typedef struct
{
    float radial_velocity_mps;
    float range_m;
    bool valid;
} mmwave_measurement_t;

typedef struct
{
    fc_spi_bus_t *bus;
    mmwave_measurement_t last_measurement;
    bool initialized;
} mmwave_doppler_t;

fc_status_t mmwave_doppler_init(mmwave_doppler_t *device, fc_spi_bus_t *bus);
fc_status_t mmwave_doppler_read(mmwave_doppler_t *device, mmwave_measurement_t *measurement);

#endif
