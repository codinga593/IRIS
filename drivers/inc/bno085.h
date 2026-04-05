#ifndef BNO085_H
#define BNO085_H

#include "flight_types.h"
#include "platform_bus.h"

typedef enum
{
    BNO085_SENSOR_ACCELEROMETER = 0x01,
    BNO085_SENSOR_GYROSCOPE_CALIBRATED = 0x02,
    BNO085_SENSOR_ROTATION_VECTOR = 0x05
} bno085_sensor_id_t;

typedef struct
{
    fc_i2c_bus_t *bus;
    uint8_t address;
    uint8_t sequence[6];
    fc_imu_sample_t last_sample;
    bool initialized;
} bno085_t;

fc_status_t bno085_init(bno085_t *device, fc_i2c_bus_t *bus, uint8_t address);
fc_status_t bno085_enable_report(bno085_t *device,
                                 bno085_sensor_id_t sensor_id,
                                 uint32_t interval_us);
fc_status_t bno085_read_raw(bno085_t *device, fc_imu_sample_t *sample);

#endif
