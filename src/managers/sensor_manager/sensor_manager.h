#ifndef __SENSOR_MANAGER_H__
#define __SENSOR_MANAGER_H__

#include <stdint.h>

struct sensor_manager_instance {
  // Sensor readings
  double imu_accel[3];
  double imu_gyro[3];
  double high_g_accel[3];
  double mag_reading[3];
  float temp;
  float press;

  // GPS data
  int32_t lat;
  int32_t lon;
  int32_t height;
  int32_t vel_ned[3];
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t num_svs;
  float gps_cov_ned[6];

  // Flight history
  int32_t highest_gps_alt;
  float lowest_press;

  // Hardware peripherals
  void *htim;
  void *hrtc;

  // Time
  uint32_t system_time;
};

enum sensor_manager_err {
  SENSOR_MANAGER_ERR_OK,
};

// Sensor manager functions
enum sensor_manager_err
sensor_manager_init(struct sensor_manager_instance *sensor_manager);
enum sensor_manager_err
sensor_manager_update(struct sensor_manager_instance *sensor_manager);

// Sensor hardware initialization functions
void magnetometer_init();
void high_g_imu_init();
void imu_init();
void barometer_init();
void gps_init();

uint8_t gps_process();

#endif /* __SENSOR_MANAGER_H__ */