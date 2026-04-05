#ifndef FLIGHT_COMPUTER_H
#define FLIGHT_COMPUTER_H

#include "bno085.h"
#include "flight_config.h"
#include "flight_types.h"
#include "icm42688p.h"
#include "lidar_uart.h"
#include "lora_radio.h"
#include "mmwave_doppler.h"
#include "ms5611.h"
#include "platform_bus.h"
#include "w25q128.h"
#include "zed_f9p.h"

typedef struct
{
    fc_i2c_bus_t *baro_i2c;
    fc_i2c_bus_t *imu1_i2c;
    fc_spi_bus_t *imu2_spi;
    fc_uart_bus_t *lidar_uart;
    fc_can_bus_t *fdcan;
    fc_uart_bus_t *gps_uart;
    fc_uart_bus_t *rpi_uart;
    fc_uart_bus_t *cm5_uart;
    fc_uart_bus_t *fin_uart;
    fc_pwm_bus_t *pwm;
    fc_spi_bus_t *lora_spi;
    fc_spi_bus_t *flash_spi;
    fc_spi_bus_t *mmwave_spi;
    fc_pyro_interface_t *pyro;
    fc_time_source_t *time;
} fc_board_config_t;

typedef struct
{
    fc_board_config_t board;
    ms5611_t baro;
    bno085_t imu1;
    icm42688p_t imu2;
    lidar_uart_t lidar;
    zed_f9p_t gps;
    lora_radio_t lora;
    w25q128_t flash;
    mmwave_doppler_t mmwave;
    fc_flight_state_t state;
    fc_waypoint_t waypoint;
    fc_baro_sample_t baro_sample;
    fc_imu_sample_t imu1_sample;
    fc_imu_sample_t imu2_sample;
    fc_gps_fix_t gps_fix;
    fc_ukf_state_t ukf_state;
    fc_cm5_command_t cm5_command;
    fc_fin_command_t applied_fin_command;
    fc_tvc_command_t tvc_command;
    fc_pyro_status_t pyro_status;
    float lidar_range_m;
    float ground_baro_altitude_m;
    float suicide_burn_height_m;
    float burnout_timer_ms;
    uint32_t last_cm5_rx_ms;
    uint32_t state_entry_ms;
    bool launch_detected;
    bool second_stage_ignited;
} fc_flight_computer_t;

fc_status_t FlightComputer_Init(fc_flight_computer_t *computer, const fc_board_config_t *board);
fc_status_t FlightComputer_Step(fc_flight_computer_t *computer);

fc_status_t readBaroPressure(fc_flight_computer_t *computer);
fc_status_t readOrientationRaw1(fc_flight_computer_t *computer);
fc_status_t readOrientationRaw2(fc_flight_computer_t *computer);
fc_status_t ReadGPS(fc_flight_computer_t *computer);
fc_status_t UKFState(fc_flight_computer_t *computer, uint32_t now_ms);
fc_status_t TransmitStateCm5(fc_flight_computer_t *computer);
fc_status_t recieveCM5(fc_flight_computer_t *computer);
fc_status_t CostFunction1(fc_flight_computer_t *computer);
fc_status_t InnerLQRFIN(fc_flight_computer_t *computer);
fc_status_t LQRTVC(fc_flight_computer_t *computer);
fc_status_t Pyro_On(fc_flight_computer_t *computer);
fc_status_t Pyro_Off(fc_flight_computer_t *computer);
fc_status_t Pyro_Fire(fc_flight_computer_t *computer);
fc_status_t Pyro_Fire2(fc_flight_computer_t *computer);
fc_status_t LoraTransmit(fc_flight_computer_t *computer);
fc_status_t Flashwrite(fc_flight_computer_t *computer);
fc_status_t ProportionalNavigationCM5(fc_flight_computer_t *computer);
fc_status_t NMPCCM5(fc_flight_computer_t *computer);
float ComputeSuicideBurn(fc_flight_computer_t *computer);
fc_vec3_t ComputeVectortoWaypoint(const fc_flight_computer_t *computer);
bool ComputeLandingPossibility(fc_flight_computer_t *computer);

#endif
