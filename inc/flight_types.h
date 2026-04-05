#ifndef FLIGHT_TYPES_H
#define FLIGHT_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float x;
    float y;
    float z;
} fc_vec3_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} fc_quat_t;

typedef struct
{
    uint32_t timestamp_ms;
    float pressure_pa;
    float temperature_c;
    float altitude_m;
    bool valid;
} fc_baro_sample_t;

typedef struct
{
    uint32_t timestamp_ms;
    fc_vec3_t accel_mps2;
    fc_vec3_t gyro_rps;
    fc_quat_t attitude;
    bool attitude_valid;
    bool valid;
} fc_imu_sample_t;

typedef struct
{
    bool valid;
    bool rtk_fix;
    double latitude_deg;
    double longitude_deg;
    float altitude_msl_m;
    fc_vec3_t position_m;
    fc_vec3_t velocity_mps;
    uint8_t satellites;
} fc_gps_fix_t;

typedef struct
{
    float x_m;
    float y_m;
} fc_waypoint_t;

typedef struct
{
    uint32_t timestamp_ms;
    fc_vec3_t position_m;
    fc_vec3_t velocity_mps;
    fc_vec3_t acceleration_mps2;
    fc_quat_t attitude;
    float pressure_pa;
    float altitude_agl_m;
    bool gps_valid;
    bool landing_possible;
} fc_ukf_state_t;

typedef struct
{
    float fin1_deg;
    float fin2_deg;
    float fin3_deg;
    float fin4_deg;
    bool valid;
} fc_fin_command_t;

typedef struct
{
    float pitch_cmd_rad;
    float yaw_cmd_rad;
    bool valid;
} fc_tvc_command_t;

typedef struct
{
    bool armed;
    bool continuity_ok;
} fc_pyro_status_t;

typedef enum
{
    FC_LORA_CMD_NONE = 0,
    FC_LORA_CMD_ARM,
    FC_LORA_CMD_DISARM,
    FC_LORA_CMD_ABORT
} fc_lora_command_id_t;

typedef struct
{
    fc_lora_command_id_t id;
    bool valid;
} fc_lora_command_t;

typedef struct
{
    uint32_t generation;
    fc_fin_command_t fin_command;
    bool valid;
} fc_cm5_command_t;

typedef enum
{
    FLIGHT_STATE_STANDBY = 0,
    FLIGHT_STATE_ARMED_RTL,
    FLIGHT_STATE_BURN,
    FLIGHT_STATE_COAST,
    FLIGHT_STATE_APOGEE,
    FLIGHT_STATE_DESCENT,
    FLIGHT_STATE_SUICIDE_BURN,
    FLIGHT_STATE_LANDED,
    FLIGHT_STATE_ABORT
} fc_flight_state_t;

typedef struct
{
    uint32_t timestamp_ms;
    fc_flight_state_t state;
    fc_ukf_state_t ukf_state;
    fc_gps_fix_t gps_fix;
} fc_log_record_t;

#endif
