#include <stdint.h>

#include "iam20680ht.h"

enum iam20680ht_err iam20680ht_initialize(struct iam20680ht_device *device) {
  // Soft reset - 4.18.1
  enum iam20680ht_err ret = iam20680ht_soft_reset(device);
  if(ret == IAM20680HT_ERR_OK)
  {
    // Output data rate selection - 4.18.4
    uint8_t gyro_config = device->gyro_fs | device->gyro_dlpf_en;
    uint8_t accel_config = device->accel_fs;
    iam20680ht_write_byte(device, IAM20680HT_REG_GYRO_CONFIG, gyro_config);
    iam20680ht_write_byte(device, IAM20680HT_REG_ACCEL_CONFIG, accel_config);

    // Filter frequency selection - 4.18.6
    uint8_t gyro_fsr_filter = device->gyro_fsr_filter;
    uint8_t accel_fsr_filter = device->accel_fsr_filter;
    iam20680ht_write_byte(device, IAM20680HT_REG_ACCEL_CONFIG_2, gyro_fsr_filter);
    iam20680ht_write_byte(device, IAM20680HT_REG_CONFIG, gyro_fsr_filter);
    // Power mode selection - 4.18.17?? Specifies nothing, let's hope we start in Mode 6
  }
  
  return ret;
}

enum iam20680ht_err iam20680ht_soft_reset(struct iam20680ht_device *device) {
  enum iam20680ht_err ret = IAM20680HT_ERR_OK;
  // Soft reset
  iam20680ht_write_byte(device, IAM20680HT_REG_PWR_MGMT_1, 0x81);
  iam20680ht_delay(1);

  // Digital interface access test
  uint8_t sensor_id;
  iam20680ht_read_byte(device, IAM20680HT_REG_WHO_AM_I, &sensor_id);

  if (sensor_id != 0xFA) {
    ret = IAM20680HT_BAD_SOFT_RESET;
  }

  return ret;
}

enum iam20680ht_err iam20680ht_get_accel_sensitivity(struct iam20680ht_device *device, double *sensitvity) {
  switch (device->accel_fs) {
    case IAM20680HT_ACCEL_FS_2G:
      *sensitvity = 16384.f;
      break;
    case IAM20680HT_ACCEL_FS_4G:
      *sensitvity = 8192.f;
      break; 
    case IAM20680HT_ACCEL_FS_8G:
      *sensitvity = 4096.f;
      break; 
    case IAM20680HT_ACCEL_FS_16G:
      *sensitvity = 2048.f;
      break;
  }

  return IAM20680HT_ERR_OK;
}

enum iam20680ht_err iam20680ht_get_gyro_sensitivity(struct iam20680ht_device *device, double *sensitvity) {
  // TODO: make sure these are interpreted as a float
  switch (device->gyro_fs) {
    case IAM20680HT_GYRO_FS_250DPS:
      *sensitvity = 131;
      break;
    case IAM20680HT_GYRO_FS_500DPS:
      *sensitvity = 65.5;
      break; 
    case IAM20680HT_GYRO_FS_1000DPS:
      *sensitvity = 32.8;
      break; 
    case IAM20680HT_ACCEL_FS_16G:
      *sensitvity = 16.4;
      break;
  }

  return IAM20680HT_ERR_OK;
}

enum iam20680ht_err iam20680ht_read_accel(struct iam20680ht_device *device, double *accel_reading) {
  uint8_t rx[6];
  double sensitivity = 0;

  iam20680ht_read_byte(device, IAM20680HT_REG_ACCEL_XOUT_H, &rx[0]);
  iam20680ht_read_byte(device, IAM20680HT_REG_ACCEL_XOUT_L, &rx[1]);
  iam20680ht_read_byte(device, IAM20680HT_REG_ACCEL_YOUT_H, &rx[2]);
  iam20680ht_read_byte(device, IAM20680HT_REG_ACCEL_YOUT_L, &rx[3]);
  iam20680ht_read_byte(device, IAM20680HT_REG_ACCEL_ZOUT_H, &rx[4]);
  iam20680ht_read_byte(device, IAM20680HT_REG_ACCEL_ZOUT_L, &rx[5]);

//  iam20680ht_read_block(device, IAM20680HT_REG_ACCEL_XOUT_H, 6, rx);

  int16_t x = (rx[0] << 8) | rx[1];
  int16_t y = (rx[2] << 8) | rx[3];
  int16_t z = (rx[4] << 8) | rx[5];

  iam20680ht_get_accel_sensitivity(device, &sensitivity);

  accel_reading[0] = (double)x/sensitivity;
  accel_reading[1] = (double)y/sensitivity;
  accel_reading[2] = (double)z/sensitivity;

  return IAM20680HT_ERR_OK;
}

enum iam20680ht_err iam20680ht_read_gyro(struct iam20680ht_device *device, double *gyro_reading) {
  uint8_t rx[6];
  double sensitivity = 0;

  iam20680ht_read_block(device, IAM20680HT_REG_GYRO_XOUT_H, 6, rx);

  int16_t x = (rx[0] << 8) | rx[1];
  int16_t y = (rx[2] << 8) | rx[3];
  int16_t z = (rx[4] << 8) | rx[5];

  iam20680ht_get_gyro_sensitivity(device, &sensitivity);

  gyro_reading[0] = (double)x/sensitivity;
  gyro_reading[1] = (double)y/sensitivity;
  gyro_reading[2] = (double)z/sensitivity;

  return IAM20680HT_ERR_OK;
}

__attribute__((weak)) enum iam20680ht_err 
iam20680ht_delay(uint16_t delay_ms) {
  return IAM20680HT_ERR_OK;
}

__attribute__((weak)) enum iam20680ht_err
iam20680ht_write_byte(struct iam20680ht_device *device, uint8_t addr,
                      uint8_t byte) {
  return IAM20680HT_ERR_OK;
}

__attribute__((weak)) enum iam20680ht_err
iam20680ht_read_byte(struct iam20680ht_device *device, uint8_t addr,
                     uint8_t *byte) {
  return IAM20680HT_ERR_OK;
}

__attribute__((weak)) enum iam20680ht_err
iam20680ht_write_block(struct iam20680ht_device *device, uint8_t start_reg,
                       uint8_t bytes, uint8_t *data) {
  return IAM20680HT_ERR_OK;
}

__attribute__((weak)) enum iam20680ht_err
iam20680ht_read_block(struct iam20680ht_device *device, uint8_t start_reg,
                      uint8_t bytes, uint8_t *data) {
  return IAM20680HT_ERR_OK;
}