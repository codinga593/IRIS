#ifndef ICM42688P_H
#define ICM42688P_H

#include "flight_types.h"
#include "platform_bus.h"

typedef struct
{
    fc_spi_bus_t *bus;
    fc_imu_sample_t last_sample;
    bool initialized;
} icm42688p_t;

fc_status_t icm42688p_init(icm42688p_t *device, fc_spi_bus_t *bus);
fc_status_t icm42688p_read_raw(icm42688p_t *device, fc_imu_sample_t *sample);

#endif
