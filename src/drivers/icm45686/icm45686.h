/* Driver for ICM-45686 IMU
  Minimal driver for TDK ICM-45686 inertial measurement unit. 
  Datasheet: https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf
  User Guide: https://invensense.tdk.com/wp-content/uploads/2024/07/AN-000478_ICM-45605-ICM-45686-User-Guide.pdf
*/

#ifndef __ICM45686_H__
#define __ICM45686_H__

#include <stdint.h>

/* ICM-45686 Register Map */
// User Bank 0
#define ICM45686_REG_ACCEL_DATA_X1_UI   0x00
#define ICM45686_REG_ACCEL_DATA_X0_UI   0x01
#define ICM45686_REG_ACCEL_DATA_Y1_UI   0x02
#define ICM45686_REG_ACCEL_DATA_Y0_UI   0x03
#define ICM45686_REG_ACCEL_DATA_Z1_UI   0x04
#define ICM45686_REG_ACCEL_DATA_Z0_UI   0x05

#define ICM45686_REG_GYRO_DATA_X1_UI    0x06
#define ICM45686_REG_GYRO_DATA_X0_UI    0x07
#define ICM45686_REG_GYRO_DATA_Y1_UI    0x08
#define ICM45686_REG_GYRO_DATA_Y0_UI    0x09
#define ICM45686_REG_GYRO_DATA_Z1_UI    0x0A
#define ICM45686_REG_GYRO_DATA_Z0_UI    

#define ICM45686_REG_TEMP_DATA1_UI      0x0C
#define ICM45686_REG_TEMP_DATA0_UI      0x0D

#define ICM45686_REG_TMST_FSYNCH        0x0E
#define ICM45686_REG_TMST_FSYNCL        0x0F

#define ICM45686_REG_PWR_MGMT0          0x10
#define ICM45686_REG_ACCEL_CONFIG0      0x1B 
#define ICM45686_REG_GYRO_CONFIG0       0x1C

#define ICM45686_REG_WHO_AM_I           0x72

#define ICM45686_REG_MISC2              0x7F

/* PWR_MGMT0 (0x10) configuration map */
// Gyro (bits 3:2)
#define ICM45686_GYRO_MODE_OFF    (0b00 << 2)
#define ICM45686_GYRO_MODE_STBY   (0b01 << 2)
#define ICM45686_GYRO_MODE_LP     (0b10 << 2)
#define ICM45686_GYRO_MODE_LN     (0b11 << 2)

// Accel (bits 1:0)
#define ICM45686_ACCEL_MODE_OFF    (0b00)
#define ICM45686_ACCEL_MODE_LP     (0b10)
#define ICM45686_ACCEL_MODE_LN     (0b11)

/* ACCEL_CONFIG0 (0x1B) configuration map */
// Full scale range (bits 6:4)
#define ICM45686_ACCEL_FS_32G     (0b000 << 4)
#define ICM45686_ACCEL_FS_16G     (0b001 << 4)
#define ICM45686_ACCEL_FS_8G      (0b010 << 4)
#define ICM45686_ACCEL_FS_4G      (0b011 << 4)
#define ICM45686_ACCEL_FS_2G      (0b100 << 4)

// Output data rate (bits 3:0)
#define ICM45686_ACCEL_ODR_1_5625HZ   0b1111
#define ICM45686_ACCEL_ODR_3_125HZ    0b1110
#define ICM45686_ACCEL_ODR_6_25HZ     0b1101
#define ICM45686_ACCEL_ODR_12_5HZ     0b1100
#define ICM45686_ACCEL_ODR_25HZ       0b1011
#define ICM45686_ACCEL_ODR_50HZ       0b1010
#define ICM45686_ACCEL_ODR_100HZ      0b1001
#define ICM45686_ACCEL_ODR_200HZ      0b1000
#define ICM45686_ACCEL_ODR_400HZ      0b0111
#define ICM45686_ACCEL_ODR_800HZ      0b0110
#define ICM45686_ACCEL_ODR_1600HZ     0b0101
#define ICM45686_ACCEL_ODR_3200HZ     0b0100
#define ICM45686_ACCEL_ODR_6400HZ     0b0011

/* GYRO_CONFIG0 (0x1C) configuration map */
// Full scale range (bits 7:4)
#define ICM45686_GYRO_FS_4000DPS    (0b0000 << 4)
#define ICM45686_GYRO_FS_2000DPS    (0b0001 << 4)
#define ICM45686_GYRO_FS_1000DPS    (0b0010 << 4)
#define ICM45686_GYRO_FS_500DPS     (0b0011 << 4)
#define ICM45686_GYRO_FS_250DPS     (0b0100 << 4)
#define ICM45686_GYRO_FS_125        (0b0101 << 4)
#define ICM45686_GYRO_FS_62_5DPS    (0b0110 << 4)
#define ICM45686_GYRO_FS_31_25DPS   (0b0111 << 4)
#define ICM45686_GYRO_FS_15_625DPS  (0b1000 << 4)

// Output data rate (bits 3:0)
#define ICM45686_GYRO_ODR_1_5625HZ   0b1111
#define ICM45686_GYRO_ODR_3_125HZ    0b1110
#define ICM45686_GYRO_ODR_6_25HZ     0b1101
#define ICM45686_GYRO_ODR_12_5HZ     0b1100
#define ICM45686_GYRO_ODR_25HZ       0b1011
#define ICM45686_GYRO_ODR_50HZ       0b1010
#define ICM45686_GYRO_ODR_100HZ      0b1001
#define ICM45686_GYRO_ODR_200HZ      0b1000
#define ICM45686_GYRO_ODR_400HZ      0b0111
#define ICM45686_GYRO_ODR_800HZ      0b0110
#define ICM45686_GYRO_ODR_1600HZ     0b0101
#define ICM45686_GYRO_ODR_3200HZ     0b0100
#define ICM45686_GYRO_ODR_6400HZ     0b0011

/* MISC */
#define ICM45686_SOFT_RESET 0x02

enum icm45686_err {
  ICM45686_ERR_OK,
  ICM45686_ERR_VALUE
};

struct icm45686_device {
  uint8_t gyro_pwr_mode;
  uint8_t accel_pwr_mode; 

  uint8_t gyro_fs;
  uint8_t gyro_odr;

  uint8_t accel_fs;
  uint8_t accel_odr;

  void *hspi;
  void *cs_gpio_port;
  uint16_t cs_gpio_pin;
};

enum icm45686_err icm45686_soft_reset(struct icm45686_device *device);

enum icm45686_err icm45686_read_whoami(struct icm45686_device *device, uint8_t *whoami);

enum icm45686_err icm45686_init(struct icm45686_device *device);

enum icm45686_err icm45686_read_accel(struct icm45686_device *device, double *accel_reading);

enum icm45686_err icm45686_read_gyro(struct icm45686_device *device, double *gyro_reading);

enum icm45686_err icm45686_read_temp(struct icm45686_device *device, double *temp);

enum icm45686_err icm45686_read_byte(struct icm45686_device *device, uint8_t addr, uint8_t *byte);

enum icm45686_err icm45686_write_byte(struct icm45686_device *device, uint8_t addr, uint8_t byte);

enum icm45686_err icm45686_delay_ms(uint16_t delay);

#endif /* __ICM45686_H__ */
