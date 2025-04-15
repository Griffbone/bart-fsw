/* Driver for PCA9563 IO expander
   Minimal driver for TI PCA9563 IO expander 
   Datasheet: https://www.ti.com/lit/ds/symlink/pca9536.pdf?ts=1744612603579&ref_url=https%253A%252F%252Fwww.google.com%252F
*/

#ifndef __PCA9563_H__
#define __PCA9563_H__

#include <stdint.h>

#define PCA9563_I2C_ADDR 0x41

// Register Map
#define PCA9563_REG_INPUT_PORT         0x00
#define PCA9563_REG_OUTPUT_PORT        0x01
#define PCA9564_REG_POLARITY_INVERSION 0x02
#define PCA9563_REG_CONFIG             0x03

// Pin Map
#define PCA9563_PIN_P0  0x01
#define PCA9563_PIN_P1  0x02
#define PCA9563_PIN_P2  0x04
#define PCA9563_PIN_P3  0x08

// Pin Value Map
#define PCA9563_PIN_HIGH   0x01
#define PCA9563_PIN_LOW    0x00

// Pin Mode Map 
#define PCA9563_PIN_OUTPUT 0x00
#define PCA9563_PIN_INPUT  0x01

// Input port 
//    incoming logic level of the pins 
//    only acts on a read operation 
// Output port 
//    outgoing logic levels of the pins 
//    no effects on pins defined as inputs 
//    reads reflect value in output selection, not actual pin value
//    bit set to one --> pin has logic level one 
//    bit set to zero --> pin has logic level zero 
// Polarity inversion 
//    configures polarity inversion of IO pins 
//    bit set to one --> polarity is inverted
//    bit set to zero --> polarity normal
// Config 
//    configures direction of IO pins 
//    bit set to one --> enable as input 
//    bit set to zero --> enable as output 

typedef enum pca9563_err {
   PCA9563_ERR_OK = 0,
   PCA9563_ERR_HAL,
   PCA9563_ERR_UNKNOWN_MODE,
   PCA9563_ERR_UNKNOWN_VALUE, 
} pca9563_err_t;

typedef struct pca9563_device {
   void *hi2c;
   uint32_t timeout;
} pca9563_device_t;

pca9563_err_t pca9563_set_pin_mode(pca9563_device_t *device, uint8_t pin, uint8_t mode);
pca9563_err_t pca9563_set_pin(pca9563_device_t *device, uint8_t pin, uint8_t value); 
pca9563_err_t pca9563_read_pin(pca9563_device_t *device, uint8_t pin, uint8_t *value);

// Read-address the device and read a single byte return
pca9563_err_t pca9563_read_byte(pca9563_device_t *device, uint8_t *byte);

// Write-address the device and write a single byte to the device
pca9563_err_t pca9563_write_byte(pca9563_device_t *device, uint8_t byte);

#endif /* __PCA9563_H__ */