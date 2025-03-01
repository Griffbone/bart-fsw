#include "lis3mdl.h"

/**
 * @brief initialize the LIS3MDL magnetometer
 *
 * This function initializes the LIS3MDL magnetometer based on settings in the
 * device structure. The device is then powered up in continuous-conversion
 * mode.
 * @param device pointer to a lis3mdl_device structure
 * @warning ensure that all device parameters are filled out before initializing
 * the device
 */
enum lis3mdl_err lis3mdl_initialize(struct lis3mdl_device *device) {
  lis3mdl_write_byte(device, LIS3MDL_REG_CTRL3, LIS3MDL_CONTINUOUS_CONVERSION);

  uint8_t ctrl_reg_1 =
      device->temp_enable | device->data_rate | device->self_test;
  lis3mdl_write_byte(device, LIS3MDL_REG_CTRL1, ctrl_reg_1);

  uint8_t ctrl_reg_2 = device->full_scale;
  lis3mdl_write_byte(device, LIS3MDL_REG_CTRL2, ctrl_reg_2);

  return LIS3MDL_ERR_OK;
}

/**
 * @brief get the sensitivity of the LIS3MDL device
 *
 * This function determines the sensitivity of the LIS3MDL based on the setting
 * contained in the device structure
 * @param device pointer to a lis3mdl_device structure
 * @param sensitivity pointer to a double to store the determined sensitivity
 * @warning this function determines the sensitivity based off the settings in
 * the device structure. If these settings do not match what is actually
 * contained in the LIS3MDL_REG_CTRL2 register the sensitivty may be inaccurate.
 */
enum lis3mdl_err lis3mdl_get_sensitivity(struct lis3mdl_device *device,
                                         double *sensitivity) {
  switch (device->full_scale) {
  case LIS3MDL_FS_4Gauss:
    *sensitivity = 6842.0f;
    break;
  case LIS3MDL_FS_8Gauss:
    *sensitivity = 3421.0f;
    break;
  case LIS3MDL_FS_12Gauss:
    *sensitivity = 2281.0f;
    break;
  case LIS3MDL_FS_16Gauss:
    *sensitivity = 1711.0f;
    break;
  }

  return LIS3MDL_ERR_OK;
}

/**
 * @brief read magnetic field vector from LIS3MDL device
 *
 * This functon reads the magnetic field vector from the LIS3MDL device,
 * converts the value to double precision, and stores the vector in a provided
 * array. Magnetic field is expressed in Gauss.
 * @param device pointer to a lis3mdl_device structure
 * @param mag_reading pointer to a 3-element double array to store the magnetic
 * field reading in Gauss
 * @warning no error checking is performed. Make sure to allocate an appropriate
 * array for all inputs
 */
enum lis3mdl_err lis3mdl_read_mag(struct lis3mdl_device *device,
                                  double *mag_reading) {
  uint8_t mag_read_buf[6];
  double sensitivity = 0;

  lis3mdl_read_block(device, LIS3MDL_REG_OUT_X_L, 6, mag_read_buf);

  int16_t x_reading = (mag_read_buf[1] << 8) | mag_read_buf[0];
  int16_t y_reading = (mag_read_buf[3] << 8) | mag_read_buf[2];
  int16_t z_reading = (mag_read_buf[5] << 8) | mag_read_buf[4];

  lis3mdl_get_sensitivity(device, &sensitivity);

  mag_reading[0] = (double)x_reading / sensitivity;
  mag_reading[1] = (double)y_reading / sensitivity;
  mag_reading[2] = (double)z_reading / sensitivity;

  return LIS3MDL_ERR_OK;
}

/**
 * @brief read temperature from LIS3MDL device
 *
 * This functon reads the temperature sensor on the LIS3MDL device and converts
 * it to double precision in degC.
 * @param device pointer to a lis3mdl_device structure
 * @param temp pointer to a double to store the measured temperature in degC
 * @warning this functon only works if LIS3MDL_TEMP_EN has been written to
 * LIS3MDL_REG_CTRL1 during the initialization step
 */
enum lis3mdl_err lis3mdl_read_temp(struct lis3mdl_device *device,
                                   double *temp) {
  uint8_t temp_read_buf[2];

  lis3mdl_read_block(device, LIS3MDL_REG_TEMP_OUT_L, 2, temp_read_buf);

  int16_t temp_reading = (temp_read_buf[1] << 8) | temp_read_buf[0];
  *temp = (double)temp_reading / 8.0f + 25.0f;

  return LIS3MDL_ERR_OK;
}

__attribute__((weak)) enum lis3mdl_err
lis3mdl_write_byte(struct lis3mdl_device *device, uint8_t reg, uint8_t data) {
  return LIS3MDL_ERR_OK;
}

__attribute__((weak)) enum lis3mdl_err
lis3mdl_read_byte(struct lis3mdl_device *device, uint8_t reg, uint8_t *data) {
  return LIS3MDL_ERR_OK;
}

__attribute__((weak)) enum lis3mdl_err
lis3mdl_write_block(struct lis3mdl_device *device, uint8_t start_reg,
                    uint8_t bytes, uint8_t *data) {
  return LIS3MDL_ERR_OK;
}

__attribute__((weak)) enum lis3mdl_err
lis3mdl_read_block(struct lis3mdl_device *device, uint8_t start_reg,
                   uint8_t bytes, uint8_t *data) {
  return LIS3MDL_ERR_OK;
}