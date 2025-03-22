/* Driver for BMP581 barometer
    Minimal driver for Bosch BMP581 barometer
    Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf
*/

#ifndef __BMP581_H__
#define __BMP581_H__

#include <stdint.h>

/* LIS3MDL Register Map */
#define BMP581_REG_ASIC_ID          0x01
#define BMP581_REG_ASIC_REV_ID      0x02
#define BMP581_REG_ASIC_STATUS      0x11

#define BMP581_REG_TEMP_XLSB        0x1D
#define BMP581_REG_TEMP_LSB         0x1E
#define BMP581_REG_TEMP_MSB         0x1F

#define BMP581_REG_PRESS_XLSB       0x20
#define BMP581_REG_PRESS_LSB        0x21
#define BMP581_REG_PRESS_MSB        0x22

#define BMP581_REG_STATUS           0x28
#define BMP581_REG_CMD              0x7E

/* Device ID */
#define BMP581_ADDR_SDO_HIGH    0x47
#define BMP581_ADDR_SDO_LOW     0x46

enum bmp581_err {
    BMP581_ERR_OK,
    BMP581_ERR_HAL,
};

struct bmp581_device {
    uint8_t i2c_addr;

    void *hi2c;
};

enum bmp581_err bmp581_init(struct bmp581_device *device);
enum bmp581_err bmp581_write_byte(struct bmp581_device *device, uint8_t reg, uint8_t data);
enum bmp581_err bmp581_read_byte(struct bmp581_device *device, uint8_t reg, uint8_t *data);

#endif /* __BMP581_H__*/