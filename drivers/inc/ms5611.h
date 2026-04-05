#ifndef MS5611_H
#define MS5611_H

#include "flight_types.h"
#include "platform_bus.h"

typedef struct
{
    fc_i2c_bus_t *bus;
    uint8_t address;
    uint16_t prom[8];
    bool initialized;
} ms5611_t;

fc_status_t ms5611_init(ms5611_t *device, fc_i2c_bus_t *bus, uint8_t address);
fc_status_t ms5611_read(ms5611_t *device, fc_baro_sample_t *sample);

#endif
