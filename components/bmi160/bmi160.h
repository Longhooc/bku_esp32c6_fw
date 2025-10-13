#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// BMI160 default I2C address is 0x68/0x69, but we use SPI, so address is not used.

// Registers (subset) per BMI160 datasheet
#define BMI160_REG_CHIP_ID              0x00
#define BMI160_REG_ERR                  0x02
#define BMI160_REG_PMU_STATUS           0x03
#define BMI160_REG_DATA_ACC_X_L         0x12
#define BMI160_REG_DATA_GYR_X_L         0x0C
#define BMI160_REG_STATUS               0x1B
#define BMI160_REG_INT_STATUS_0         0x1C
#define BMI160_REG_INT_STATUS_1         0x1D
#define BMI160_REG_INT_STATUS_2         0x1E
#define BMI160_REG_INT_STATUS_3         0x1F
#define BMI160_REG_TEMP_L               0x20
#define BMI160_REG_ACC_CONF             0x40
#define BMI160_REG_ACC_RANGE            0x41
#define BMI160_REG_GYR_CONF             0x42
#define BMI160_REG_GYR_RANGE            0x43
#define BMI160_REG_INT_EN_0             0x50
#define BMI160_REG_INT_EN_1             0x51
#define BMI160_REG_INT_EN_2             0x52
#define BMI160_REG_INT_OUT_CTRL         0x53
#define BMI160_REG_INT_LATCH            0x54
#define BMI160_REG_INT_MAP_0            0x55
#define BMI160_REG_INT_MAP_1            0x56
#define BMI160_REG_INT_MAP_2            0x57
#define BMI160_REG_INT_MOTION_0         0x5F
#define BMI160_REG_INT_MOTION_1         0x60
#define BMI160_REG_INT_MOTION_2         0x61
#define BMI160_REG_INT_MOTION_3         0x62
#define BMI160_REG_CMD                  0x7E

// Commands
#define BMI160_CMD_SOFTRESET            0xB6
#define BMI160_CMD_ACC_PMU_SUSPEND      0x10
#define BMI160_CMD_ACC_PMU_NORMAL       0x11
#define BMI160_CMD_GYR_PMU_SUSPEND      0x14
#define BMI160_CMD_GYR_PMU_NORMAL       0x15

typedef enum {
    BMI160_ACC_RANGE_2G  = 0x03,
    BMI160_ACC_RANGE_4G  = 0x05,
    BMI160_ACC_RANGE_8G  = 0x08,
    BMI160_ACC_RANGE_16G = 0x0C,
} bmi160_acc_range_t;

typedef enum {
    BMI160_GYR_RANGE_2000_DPS = 0x00,
    BMI160_GYR_RANGE_1000_DPS = 0x01,
    BMI160_GYR_RANGE_500_DPS  = 0x02,
    BMI160_GYR_RANGE_250_DPS  = 0x03,
    BMI160_GYR_RANGE_125_DPS  = 0x04,
} bmi160_gyr_range_t;

typedef struct {
    int32_t accel_x;
    int32_t accel_y;
    int32_t accel_z;
    int32_t gyro_x;
    int32_t gyro_y;
    int32_t gyro_z;
    int32_t temperature_c_x100; // temperature in centi-degree Celsius
} bmi160_sample_t;

typedef struct {
    int gpio_cs;     // SPI CS GPIO
    int gpio_mosi;   // SPI MOSI GPIO
    int gpio_miso;   // SPI MISO GPIO
    int gpio_sck;    // SPI SCK GPIO
    int gpio_int1;   // INT1 GPIO (motion/wakeup)
    int gpio_int2;   // INT2 GPIO (wakeup/fifo/step)
    int spi_host;    // SPI2_HOST, SPI3_HOST, etc.
    int clock_speed_hz; // SPI clock
} bmi160_spi_bus_config_t;

typedef struct bmi160_handle_s* bmi160_handle_t;

esp_err_t bmi160_create(const bmi160_spi_bus_config_t *bus_cfg, bmi160_handle_t *out_handle);
esp_err_t bmi160_init_default(bmi160_handle_t handle);
esp_err_t bmi160_config_ranges(bmi160_handle_t handle, bmi160_acc_range_t acc, bmi160_gyr_range_t gyr);
esp_err_t bmi160_enable_anymotion_wakeup(bmi160_handle_t handle, uint8_t threshold, uint8_t duration);
// Configure any-motion threshold by physical unit. ms2_x100 is m/s^2 * 100 (e.g. 5.00 m/s^2 -> 500)
esp_err_t bmi160_enable_anymotion_wakeup_ms2(bmi160_handle_t handle, int ms2_x100, uint8_t duration);
esp_err_t bmi160_read_sample(bmi160_handle_t handle, bmi160_sample_t *out);
esp_err_t bmi160_read_int_status(bmi160_handle_t handle, uint32_t *out_status);
esp_err_t bmi160_read_status(bmi160_handle_t handle, uint8_t *out_status);
esp_err_t bmi160_read_pmu_status(bmi160_handle_t handle, uint8_t *out_pmu_status);
esp_err_t bmi160_wait_data_ready(bmi160_handle_t handle, int timeout_ms);
esp_err_t bmi160_force_acc_normal(bmi160_handle_t handle, int timeout_ms);
esp_err_t bmi160_dump_regs(bmi160_handle_t handle);
void      bmi160_destroy(bmi160_handle_t handle);


