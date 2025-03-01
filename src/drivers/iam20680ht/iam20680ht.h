/* Driver for LIS3MDL magnetometer
   Minimal driver for STMicroelectronics LIS3MDL magnetometer
   Datasheet: https://www.st.com/resource/en/datasheet/lis3mdl.pdf
*/

#ifndef __IAM20680HT_H__
#define __IAM20680HT_H__

#include <stdint.h>

/* IAM20680HT Register Map */
#define IAM20680HT_REG_SELF_TEST_X_GYRO  0x00
#define IAM20680HT_REG_SELF_TEST_Y_GYRO  0x01
#define IAM20680HT_REG_SELF_TEST_Z_GYRO  0x02
#define IAM20680HT_REG_SELF_TEST_X_ACCEL 0x0D
#define IAM20680HT_REG_SELF_TEST_Y_ACCEL 0x0E
#define IAM20680HT_REG_SELF_TEST_Z_ACCEL 0x0F

#define IAM20680HT_REG_XG_OFFS_USRH 0x13
#define IAM20680HT_REG_XG_OFFS_USRL 0x14
#define IAM20680HT_REG_YG_OFFS_USRH 0x15
#define IAM20680HT_REG_YG_OFFS_USRL 0x16
#define IAM20680HT_REG_ZG_OFFS_USRH 0x17
#define IAM20680HT_REG_ZG_OFFS_USRL 0x18

#define IAM20680HT_REG_SMPLRT_DIV     0x19
#define IAM20680HT_REG_CONFIG         0x1A
#define IAM20680HT_REG_GYRO_CONFIG    0x1B
#define IAM20680HT_REG_ACCEL_CONFIG   0x1C
#define IAM20680HT_REG_ACCEL_CONFIG_2 0x1D
#define IAM20680HT_REG_LP_MODE_CFG    0x1E
#define IAM20680HT_REG_ACCEL_WOM_THR  0x1F
#define IAM20680HT_REG_FIFO_EN        0x23
#define IAM20680HT_REG_FSYNC_INT      0x36
#define IAM20680HT_REG_INT_PIN_CFG    0x37
#define IAM20680HT_REG_INT_ENABLE     0x38
#define IAM20680HT_REG_INT_STATUS     0x3A

#define IAM20680HT_REG_ACCEL_XOUT_H 0x3B
#define IAM20680HT_REG_ACCEL_XOUT_L 0x3C
#define IAM20680HT_REG_ACCEL_YOUT_H 0x3D
#define IAM20680HT_REG_ACCEL_YOUT_L 0x3E
#define IAM20680HT_REG_ACCEL_ZOUT_H 0x3F
#define IAM20680HT_REG_ACCEL_ZOUT_L 0x40

#define IAM20680HT_REG_TEMP_OUT_H 0x41
#define IAM20680HT_REG_TEMP_OUT_L 0x42

#define IAM20680HT_REG_GYRO_XOUT_H 0x43
#define IAM20680HT_REG_GYRO_XOUT_L 0x44
#define IAM20680HT_REG_GYRO_YOUT_H 0x45
#define IAM20680HT_REG_GYRO_YOUT_L 0x46
#define IAM20680HT_REG_GYRO_ZOUT_H 0x47
#define IAM20680HT_REG_GYRO_ZOUT_L 0x48

#define IAM20680HT_REG_SIGNAL_PATH_RESET 0x68
#define IAM20680HT_REG_ACCEL_INTEL_CTRL  0x69
#define IAM20680HT_REG_USER_CTRL         0x6A
#define IAM20680HT_REG_PWR_MGMT_1        0x6B
#define IAM20680HT_REG_PWR_MGMT_2        0x6C
#define IAM20680HT_REG_FIFO_COUNTH       0x72
#define IAM20680HT_REG_FIFO_COUNTL       0x73
#define IAM20680HT_REG_FIFO_R_W          0x74

#define IAM20680HT_REG_WHO_AM_I 0x75

#define IAM20680HT_REG_XA_OFFSET_H 0x77
#define IAM20680HT_REG_XA_OFFSET_L 0x78
#define IAM20680HT_REG_YA_OFFSET_H 0x7A
#define IAM20680HT_REG_YA_OFFSET_L 0x7B
#define IAM20680HT_REG_ZA_OFFSET_H 0x7D
#define IAM20680HT_REG_ZA_OFFSET_L 0x7E

/* GYRO_CONFIG (0x1B) configuration map */
#define IAM20680HT_GYRO_FS_250DPS  0x00
#define IAM20680HT_GYRO_FS_500DPS  0x08
#define IAM20680HT_GYRO_FS_1000DPS 0x10
#define IAM20680HT_GYRO_FS_2000DPS 0x18
#define IAM20680HT_GYRO_DLPF_EN    0x00
#define IAM20680HT_GYRO_DLPF_DIS   0x01

/* ACCEL_CONFIG (0x1C) configuration map */
#define IAM20680HT_ACCEL_FS_2G  0x00
#define IAM20680HT_ACCEL_FS_4G  0x08
#define IAM20680HT_ACCEL_FS_8G  0x10
#define IAM20680HT_ACCEL_FS_16G 0x18

/* ACCEL_CONFIG_2 (0x1D) configuration map */
#define IAM20680HT_ACCEL_DLPF_EN  0x00
#define IAM20680HT_ACCEL_DLPF_DIS 0x08

/* ACCEL_CONFIG_2 */

enum iam20680ht_err {
  IAM20680HT_ERR_OK,
  IAM20680HT_ERR_GENERAL,
  IAM20680HT_BAD_SOFT_RESET,
  IAM20680HT_BAD_SELF_TEST
};

struct iam20680ht_device {
  uint8_t gyro_fs;
  uint8_t accel_fs;
  uint8_t gyro_dlpf_en;
  uint8_t accel_dlpf_en;
  uint8_t accel_fsr_filter;
  uint8_t gyro_fsr_filter;

  void *hspi;
  void *cs_gpio_port;
  uint16_t cs_gpio_pin;
};

enum iam20680ht_err iam20680ht_soft_reset(struct iam20680ht_device *device);

enum iam20680ht_err iam20680ht_initialize(struct iam20680ht_device *device);

enum iam20680ht_err iam20680ht_get_accel_sensitivity(struct iam20680ht_device *device, double *sensitvity);

enum iam20680ht_err iam20680ht_get_gyro_sensitivity(struct iam20680ht_device *device, double *sensitivity);

enum iam20680ht_err iam20680ht_read_accel(struct iam20680ht_device *device, double *accel_reading);

enum iam20680ht_err iam20680ht_read_gyro(struct iam20680ht_device *device, double *gyro_reading);

enum iam20680ht_err iam20680ht_delay(uint16_t delay_ms);

enum iam20680ht_err iam20680ht_write_byte(struct iam20680ht_device *device,
                                          uint8_t addr, uint8_t byte);

enum iam20680ht_err iam20680ht_read_byte(struct iam20680ht_device *device,
                                         uint8_t addr, uint8_t *byte);

enum iam20680ht_err iam20680ht_write_block(struct iam20680ht_device *device,
                                        uint8_t start_reg, uint8_t bytes,
                                        uint8_t *data);

enum iam20680ht_err iam20680ht_read_block(struct iam20680ht_device *device,
                                       uint8_t start_reg, uint8_t bytes,
                                       uint8_t *data);

#endif /* __IAM20680HT_H__ */
