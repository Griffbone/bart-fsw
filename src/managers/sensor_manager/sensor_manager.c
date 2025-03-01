#include "sensor_manager.h"

#include "drivers/iam20680ht/iam20680ht.h"
#include "drivers/kx134/kx134.h"
#include "drivers/lis3mdl/lis3mdl.h"
#include "drivers/ms5611/ms5611.h"
#include "drivers/ublox_gnss/ublox_gnss.h"

#include "cli/cli.h"
#include "ring_buffer/ring_buffer.h"

#include "stm32h7xx_hal.h"
#include "system/rtc.h"
#include "system/spi.h"
#include "system/tim.h"
#include "system/usart.h"

extern struct cli_handle cli;

// Sensor hardware structures
struct lis3mdl_device mag;
struct kx134_device high_g_imu;
struct iam20680ht_device imu;
struct ms5611_device baro;
struct ublox_gnss_device gps;

// GPS data structures
struct ublox_gnss_nav_posllh nav_posllh;
struct ublox_gnss_nav_pvt nav_pvt;
struct ublox_gnss_nav_cov nav_cov;
struct ublox_gnss_nav_timeutc nav_timeutc;
uint8_t first_fix = 1;

// Set up GPS ring buffer
extern struct ring_buffer uart7_rx_rb;
uint8_t uart7_rx_rb_data[1024];
uint32_t bytes_to_read;

/**
 * @brief initialize the sensor manager
 *
 * This function initializes the sensor manager structure. It initializes each
 * of the sensors on the ADCS board according to flight config (hard-coded in
 * the init functions in this file) and then sets the lowest recorded pressure
 * to 1e9 and highest recorded GPS altitude to 0 so they are overwritten by real
 * values from the sensors.
 * @param sensor_manager pointer to a sensor manager structure
 * @return sensor manager error enum
 */
enum sensor_manager_err
sensor_manager_init(struct sensor_manager_instance *sensor_manager) {
  // Initialize sensors
  magnetometer_init();
  high_g_imu_init();
  gps_init();

  // Set highest and lowest values to extrema
  sensor_manager->lowest_press = 1e9;
  sensor_manager->highest_gps_alt = 0;

  return SENSOR_MANAGER_ERR_OK;
}

/**
 * @brief update the sensor manager
 *
 * This function updates the sensor manager. It first sees if there is new GPS
 * data in the UART ring buffer, if there is it copies that data to the sensor
 * manager struct. Then it polls all of the on-board sensors and records that
 * data to the sensor manager struct. It also manages recording of the maximum
 * altitude and lowest pressure fields.
 * @param sensor_manager
 * @return enum sensor_manager_err
 */
enum sensor_manager_err
sensor_manager_update(struct sensor_manager_instance *sensor_manager) {
  // Check for and handle new GPS data
  if (gps_process()) {
    sensor_manager->num_svs = nav_pvt.num_sv;

    if (sensor_manager->num_svs >= 4) {
      sensor_manager->lat = nav_posllh.lat;
      sensor_manager->lon = nav_posllh.lon;
      sensor_manager->height = nav_posllh.height;

      sensor_manager->vel_ned[0] = nav_pvt.vel_n;
      sensor_manager->vel_ned[1] = nav_pvt.vel_e;
      sensor_manager->vel_ned[2] = nav_pvt.vel_d;

      sensor_manager->year = nav_timeutc.year;
      sensor_manager->month = nav_timeutc.month;
      sensor_manager->day = nav_timeutc.day;
      sensor_manager->hour = nav_timeutc.hour;
      sensor_manager->min = nav_timeutc.min;
      sensor_manager->sec = nav_timeutc.sec;

      sensor_manager->gps_cov_ned[0] = nav_cov.pos_cov_nn;
      sensor_manager->gps_cov_ned[1] = nav_cov.pos_cov_ne;
      sensor_manager->gps_cov_ned[2] = nav_cov.pos_cov_nd;
      sensor_manager->gps_cov_ned[3] = nav_cov.pos_cov_ee;
      sensor_manager->gps_cov_ned[4] = nav_cov.pos_cov_ed;
      sensor_manager->gps_cov_ned[5] = nav_cov.pos_cov_dd;

      if (first_fix == 1) {
        RTC_TimeTypeDef rtc_time;
        rtc_time.Hours = sensor_manager->hour;
        rtc_time.Minutes = sensor_manager->min;
        rtc_time.Seconds = sensor_manager->sec;
        rtc_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        rtc_time.StoreOperation = RTC_STOREOPERATION_RESET;
        HAL_RTC_SetTime((RTC_HandleTypeDef *)sensor_manager->hrtc, &rtc_time,
                        RTC_FORMAT_BIN);

        RTC_DateTypeDef rtc_date;
        rtc_date.Date = sensor_manager->day;
        rtc_date.Month = sensor_manager->month;
        rtc_date.Year = sensor_manager->year;
        HAL_RTC_SetDate((RTC_HandleTypeDef *)sensor_manager->hrtc, &rtc_date,
                        RTC_FORMAT_BIN);

        first_fix = 0;
      }
    }
  }

  // Poll sensors
  lis3mdl_read_mag(&mag, sensor_manager->mag_reading);
  kx134_read_accel(&high_g_imu, sensor_manager->high_g_accel);

  // Check limits and re-save lowest pressure/highest GPS altitude
  if (sensor_manager->highest_gps_alt < sensor_manager->height) {
    sensor_manager->highest_gps_alt = sensor_manager->height;
  }

  if (sensor_manager->lowest_press > sensor_manager->press) {
    sensor_manager->lowest_press = sensor_manager->press;
  }

  // Get system time
  sensor_manager->system_time =
      __HAL_TIM_GET_COUNTER((TIM_HandleTypeDef *)sensor_manager->htim);

  return SENSOR_MANAGER_ERR_OK;
}

// Sensor hardware initialization functions
void magnetometer_init() {
  mag.hspi = &hspi1;
  mag.cs_gpio_port = GPIOG;
  mag.cs_gpio_pin = GPIO_PIN_13;
  mag.temp_enable = LIS3MDL_TEMP_EN;
  mag.data_rate = LIS3MDL_HP_300Hz;
  mag.self_test = LIS3MDL_SELF_TEST_DIS;
  mag.full_scale = LIS3MDL_FS_4Gauss;
  mag.z_axis_mode = LIS3MDL_Z_HP;

  lis3mdl_initialize(&mag);
}

void high_g_imu_init() {
  high_g_imu.hspi = &hspi3;
  high_g_imu.cs_gpio_port = GPIOD;
  high_g_imu.cs_gpio_pin = GPIO_PIN_1;

  kx134_init(&high_g_imu);
}

void imu_init() {
  // TODO: confirm this with actual hardware implementation
  imu.hspi = &hspi1;
  imu.cs_gpio_port = GPIOG;
  imu.cs_gpio_pin = GPIO_PIN_14;

  imu.gyro_fs = IAM20680HT_GYRO_FS_2000DPS;
  imu.accel_fs = IAM20680HT_ACCEL_FS_16G;
  imu.gyro_dlpf_en = IAM20680HT_GYRO_DLPF_DIS;
  imu.accel_dlpf_en = IAM20680HT_ACCEL_DLPF_DIS;

  iam20680ht_initialize(&imu);
}

void barometer_init() {
  baro.hspi = &hspi3;
  baro.cs_gpio_port = GPIOD;
  baro.cs_gpio_pin = GPIO_PIN_2;
  baro.resolution = MS5611_RESOLUTION_OSR_1024;
  ms5611_reset(&baro);
  HAL_Delay(10);
}

void gps_init() {
  ring_buffer_init(&uart7_rx_rb, uart7_rx_rb_data, sizeof(uart7_rx_rb_data));

  gps.transport_type = UBLOX_GNSS_TRANSPORT_UART;
  gps.transport_handle.uart = &huart7;

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);

  HAL_Delay(200);

  struct ublox_gnss_cfg_val cfg[7];

  cfg[0].key_id = 0x10740002;
  cfg[0].value = 0x00;

  cfg[1].key_id = 0x20110011;
  cfg[1].value = 0x02;

  cfg[2].key_id = 0x20910084;
  cfg[2].value = 0x01;

  cfg[3].key_id = 0x20110021;
  cfg[3].value = 0x08;

  cfg[4].key_id = 0x2091005c;
  cfg[4].value = 0x01;

  cfg[5].key_id = 0x20910007;
  cfg[5].value = 0x01;

  cfg[6].key_id = 0x2091002a;
  cfg[6].value = 0x01;

  ublox_gnss_cfg_val_set_list(&gps, cfg, 7, 0, 1);
}

// GPS parser process
uint8_t gps_process() {
  if ((bytes_to_read = ring_buffer_get_full(&uart7_rx_rb))) {
    uint8_t tmp[bytes_to_read];
    uint8_t *tmp2 = tmp;

    ring_buffer_read(&uart7_rx_rb, tmp, bytes_to_read);
    uint8_t msg[bytes_to_read];
    uint16_t len;
    uint8_t *rem;
    uint8_t class;
    uint8_t id;

    do {
      ublox_protocol_decode(tmp2, bytes_to_read, &class, &id, msg, sizeof(msg),
                            &len, &rem);

      if ((class == 0x01) && (id == 0x02)) {
        ublox_gnss_dec_ubx_nav_posllh(msg, len, &nav_posllh);
      } else if ((class == 0x01) && (id == 0x07)) {
        ublox_gnss_dec_ubx_nav_pvt(msg, len, &nav_pvt);
      } else if ((class == 0x01) && (id == 0x36)) {
        ublox_gnss_dec_ubx_nav_cov(msg, len, &nav_cov);
      } else if ((class == 0x01) && (id == 0x21)) {
        ublox_gnss_dec_ubx_nav_timeutc(msg, len, &nav_timeutc);
      }

      tmp2 = rem;
    } while (rem != (tmp + bytes_to_read));

    return 1;
  } else {
    return 0;
  }
}