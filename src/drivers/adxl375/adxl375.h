/* Driver for ADXL375 accelerometer 
  Minimal driver for AD ADXL375 accelerometer device 
  Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL375.pdf
*/

#ifndef __ADXL375__
#define __ADXL375__

#include <stdint.h>

/* ADXL375 Register Map */
#define ADXL375_REG_DEVID            0x00

#define ADXL375_REG_THRESH_SHOCK     0x1D
#define ADXL375_REG_DUR              0x21
#define ADXL375_REG_LATENT           0x22
#define ADXL375_REG_WINDOW           0x23
#define ADXL375_REG_SHOCK_AXES       0x2A
#define ADXL375_REG_ACT_SHOCK_STATUS 0x2B

#define ADXL375_REG_OFSX             0x1E
#define ADXL375_REG_OFSY             0x1F
#define ADXL375_REG_OFSZ             0x20

#define ADXL375_REG_THRESH_ACT       0x24
#define ADXL375_REG_THRESH_INACT     0x25
#define ADXL375_REG_TIME_INACT       0x26
#define ADXL375_REG_ACT_INACT_CTL    0x27

#define ADXL375_REG_BW_RATE          0x2C
#define ADXL375_REG_POWER_CTL        0x2D

#define ADXL375_REG_INT_ENABLE       0x2E
#define ADXL375_REG_INT_MAP          0x2F
#define ADXL375_REG_INT_SOURCE       0x30

#define ADXL375_REG_DATA_FORMAT      0x31
#define ADXL375_REG_DATAX0           0x32
#define ADXL375_REG_DATAX1           0x33
#define ADXL375_REG_DATAY0           0x34
#define ADXL375_REG_DATAY1           0x35
#define ADXL375_REG_DATAZ0           0x36
#define ADXL375_REG_DATAZ1           0x37

#define ADXL375_REG_FIFO_CTL         0x38
#define ADXL375_REG_FIFO_STATUS      0x39

enum adxl375_err {
  ADXL375_ERR_OK, 
  ADXL375_ERR_HAL, 
  ADXL375_ERR_GENERAL,
};

struct adxl375_device {
  void *hspi;
  void *cs_gpio_port; 
  uint16_t cs_gpio_pin;
};

enum adxl375_err adxl375_init(struct adxl375_device *device);
enum adxl375_err adxl375_read_byte(struct adxl375_device *device, uint8_t reg, uint8_t *data);
enum adxl375_err adxl375_write_byte(struct adxl375_device *device, uint8_t reg, uint8_t data);

#endif /* __ADXL375__ */