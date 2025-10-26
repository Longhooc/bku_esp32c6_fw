#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// BMI160 default I2C address is 0x68/0x69, but we use SPI, so address is not used.

// Registers per BMI160 datasheet
#define BMI160_REG_CHIP_ID              0x00
#define BMI160_REG_ERR                  0x02
#define BMI160_REG_PMU_STATUS           0x03
#define BMI160_REG_DATA_GYR_X_L         0x0C
#define BMI160_REG_DATA_GYR_X_H         0x0D
#define BMI160_REG_DATA_GYR_Y_L         0x0E
#define BMI160_REG_DATA_GYR_Y_H         0x0F
#define BMI160_REG_DATA_GYR_Z_L         0x10
#define BMI160_REG_DATA_GYR_Z_H         0x11
#define BMI160_REG_DATA_ACC_X_L         0x12
#define BMI160_REG_DATA_ACC_X_H         0x13
#define BMI160_REG_DATA_ACC_Y_L         0x14
#define BMI160_REG_DATA_ACC_Y_H         0x15
#define BMI160_REG_DATA_ACC_Z_L         0x16
#define BMI160_REG_DATA_ACC_Z_H         0x17
#define BMI160_REG_STATUS               0x1B
#define BMI160_REG_INT_STATUS_0         0x1C
#define BMI160_REG_INT_STATUS_1         0x1D
#define BMI160_REG_INT_STATUS_2         0x1E
#define BMI160_REG_INT_STATUS_3         0x1F
#define BMI160_REG_TEMP_L               0x20
#define BMI160_REG_TEMP_H               0x21
#define BMI160_REG_FIFO_LENGTH_0        0x22
#define BMI160_REG_FIFO_LENGTH_1        0x23
#define BMI160_REG_FIFO_DATA            0x24
#define BMI160_REG_ACC_CONF             0x40
#define BMI160_REG_ACC_RANGE            0x41
#define BMI160_REG_GYR_CONF             0x42
#define BMI160_REG_GYR_RANGE           0x43
#define BMI160_REG_FIFO_CONFIG_0        0x46
#define BMI160_REG_FIFO_CONFIG_1        0x47
#define BMI160_REG_INT_EN_0             0x50
#define BMI160_REG_INT_EN_1             0x51
#define BMI160_REG_INT_EN_2             0x52
#define BMI160_REG_INT_OUT_CTRL         0x53
#define BMI160_REG_INT_LATCH            0x54
#define BMI160_REG_INT_MAP_0            0x55
#define BMI160_REG_INT_MAP_1            0x56
#define BMI160_REG_INT_MAP_2            0x57
#define BMI160_REG_INT_LOWHIGH_0        0x5A
#define BMI160_REG_INT_LOWHIGH_1        0x5B
#define BMI160_REG_INT_LOWHIGH_2        0x5C
#define BMI160_REG_INT_LOWHIGH_3        0x5D
#define BMI160_REG_INT_LOWHIGH_4        0x5E
#define BMI160_REG_INT_MOTION_0         0x5F
#define BMI160_REG_INT_MOTION_1         0x60
#define BMI160_REG_INT_MOTION_2         0x61
#define BMI160_REG_INT_MOTION_3         0x62
#define BMI160_REG_INT_TAP_0            0x63
#define BMI160_REG_INT_TAP_1            0x64
#define BMI160_REG_FOC_CONF             0x69
#define BMI160_REG_OFFSET_0             0x71
#define BMI160_REG_OFFSET_1             0x72
#define BMI160_REG_OFFSET_2             0x73
#define BMI160_REG_OFFSET_3             0x74
#define BMI160_REG_OFFSET_4             0x75
#define BMI160_REG_OFFSET_5             0x76
#define BMI160_REG_OFFSET_6             0x77
#define BMI160_REG_STEP_CNT_L           0x78
#define BMI160_REG_STEP_CNT_H           0x79
#define BMI160_REG_STEP_CONF_0          0x7A
#define BMI160_REG_STEP_CONF_1          0x7B
#define BMI160_REG_CMD                  0x7E

// Commands
#define BMI160_CMD_SOFTRESET            0xB6
#define BMI160_CMD_ACC_PMU_SUSPEND      0x10
#define BMI160_CMD_ACC_PMU_NORMAL       0x11
#define BMI160_CMD_GYR_PMU_SUSPEND      0x14
#define BMI160_CMD_GYR_PMU_NORMAL       0x15
#define BMI160_CMD_START_FOC            0x03
#define BMI160_CMD_FIFO_FLUSH           0xB0
#define BMI160_CMD_INT_RESET            0xB1
#define BMI160_CMD_STEP_CNT_CLR         0xB2

// Status bits
#define BMI160_STATUS_FOC_RDY           3
#define BMI160_STATUS_NVM_RDY           4
#define BMI160_STATUS_DRDY_GYR          6
#define BMI160_STATUS_DRDY_ACC          7

// Interrupt status bits
#define BMI160_STEP_INT_BIT             0
#define BMI160_ANYMOTION_INT_BIT        2
#define BMI160_D_TAP_INT_BIT            4
#define BMI160_S_TAP_INT_BIT            5
#define BMI160_NOMOTION_INT_BIT         7
#define BMI160_FFULL_INT_BIT            5
#define BMI160_DRDY_INT_BIT             4
#define BMI160_LOW_G_INT_BIT            3
#define BMI160_HIGH_G_INT_BIT           2

// Motion detection bits
#define BMI160_TAP_SIGN_BIT             7
#define BMI160_TAP_1ST_Z_BIT            6
#define BMI160_TAP_1ST_Y_BIT           5
#define BMI160_TAP_1ST_X_BIT           4
#define BMI160_ANYMOTION_SIGN_BIT       3
#define BMI160_ANYMOTION_1ST_Z_BIT      2
#define BMI160_ANYMOTION_1ST_Y_BIT      1
#define BMI160_ANYMOTION_1ST_X_BIT      0
#define BMI160_HIGH_G_SIGN_BIT          3
#define BMI160_HIGH_G_1ST_Z_BIT         2
#define BMI160_HIGH_G_1ST_Y_BIT         1
#define BMI160_HIGH_G_1ST_X_BIT         0

// Configuration bits
#define BMI160_ACCEL_RATE_SEL_BIT       0
#define BMI160_ACCEL_RATE_SEL_LEN       4
#define BMI160_GYRO_RATE_SEL_BIT        0
#define BMI160_GYRO_RATE_SEL_LEN        4
#define BMI160_GYRO_DLPF_SEL_BIT        4
#define BMI160_GYRO_DLPF_SEL_LEN        2
#define BMI160_ACCEL_DLPF_SEL_BIT       4
#define BMI160_ACCEL_DLPF_SEL_LEN      3
#define BMI160_ACCEL_RANGE_SEL_BIT     0
#define BMI160_ACCEL_RANGE_SEL_LEN      4
#define BMI160_GYRO_RANGE_SEL_BIT       0
#define BMI160_GYRO_RANGE_SEL_LEN       3

// FIFO bits
#define BMI160_FIFO_HEADER_EN_BIT       4
#define BMI160_FIFO_ACC_EN_BIT          6
#define BMI160_FIFO_GYR_EN_BIT          7

// Interrupt enable bits
#define BMI160_ANYMOTION_EN_BIT         0
#define BMI160_ANYMOTION_EN_LEN         3
#define BMI160_D_TAP_EN_BIT             4
#define BMI160_S_TAP_EN_BIT             5
#define BMI160_NOMOTION_EN_BIT          0
#define BMI160_NOMOTION_EN_LEN          3
#define BMI160_LOW_G_EN_BIT             3
#define BMI160_LOW_G_EN_LEN             1
#define BMI160_HIGH_G_EN_BIT            0
#define BMI160_HIGH_G_EN_LEN            3
#define BMI160_STEP_EN_BIT              3
#define BMI160_DRDY_EN_BIT              4
#define BMI160_FFULL_EN_BIT             5

// Interrupt output control bits
#define BMI160_INT1_EDGE_CTRL           0
#define BMI160_INT1_LVL                 1
#define BMI160_INT1_OD                  2
#define BMI160_INT1_OUTPUT_EN           3

// Latch mode bits
#define BMI160_LATCH_MODE_BIT           0
#define BMI160_LATCH_MODE_LEN           4

// Motion detection bits
#define BMI160_ANYMOTION_DUR_BIT        0
#define BMI160_ANYMOTION_DUR_LEN        2
#define BMI160_NOMOTION_DUR_BIT         2
#define BMI160_NOMOTION_DUR_LEN         6
#define BMI160_NOMOTION_SEL_BIT         0
#define BMI160_NOMOTION_SEL_LEN         1

// Tap detection bits
#define BMI160_TAP_DUR_BIT              0
#define BMI160_TAP_DUR_LEN              3
#define BMI160_TAP_SHOCK_BIT            6
#define BMI160_TAP_QUIET_BIT            7
#define BMI160_TAP_THRESH_BIT            0
#define BMI160_TAP_THRESH_LEN           5

// FOC (Fast Offset Compensation) bits
#define BMI160_FOC_ACC_Z_BIT            0
#define BMI160_FOC_ACC_Z_LEN            2
#define BMI160_FOC_ACC_Y_BIT            2
#define BMI160_FOC_ACC_Y_LEN            2
#define BMI160_FOC_ACC_X_BIT            4
#define BMI160_FOC_ACC_X_LEN            2
#define BMI160_FOC_GYR_EN               6

// Offset compensation bits
#define BMI160_GYR_OFFSET_X_MSB_BIT     0
#define BMI160_GYR_OFFSET_X_MSB_LEN     2
#define BMI160_GYR_OFFSET_Y_MSB_BIT     2
#define BMI160_GYR_OFFSET_Y_MSB_LEN     2
#define BMI160_GYR_OFFSET_Z_MSB_BIT     4
#define BMI160_GYR_OFFSET_Z_MSB_LEN     2
#define BMI160_ACC_OFFSET_EN            6
#define BMI160_GYR_OFFSET_EN            7

// Step detection bits
#define BMI160_STEP_BUF_MIN_BIT         0
#define BMI160_STEP_BUF_MIN_LEN         3
#define BMI160_STEP_CNT_EN_BIT          3
#define BMI160_STEP_TIME_MIN_BIT        0
#define BMI160_STEP_TIME_MIN_LEN        3
#define BMI160_STEP_THRESH_MIN_BIT      3
#define BMI160_STEP_THRESH_MIN_LEN      2
#define BMI160_STEP_ALPHA_BIT           5
#define BMI160_STEP_ALPHA_LEN           3

// PMU status bits
#define BMI160_ACC_PMU_STATUS_BIT       4
#define BMI160_ACC_PMU_STATUS_LEN       2
#define BMI160_GYR_PMU_STATUS_BIT       2
#define BMI160_GYR_PMU_STATUS_LEN       2

// FIFO data invalid marker
#define BMI160_FIFO_DATA_INVALID        0x80

// FIFO frame sizes for different modes
#define BMI160_ACCEL_FRAME_SIZE         6   // 6 bytes per accelerometer frame (headerless mode)
#define BMI160_GYRO_FRAME_SIZE          6   // 6 bytes per gyroscope frame (headerless mode)
#define BMI160_ACCEL_GYRO_FRAME_SIZE   12  // 12 bytes per accel+gyro frame (headerless mode)
#define BMI160_HEADER_FRAME_SIZE        13  // 13 bytes per frame with header (1 header + 6 accel + 6 gyro)

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

// Accelerometer Output Data Rate options
typedef enum {
    BMI160_ACCEL_RATE_25_2HZ = 5,  /**<   25/2  Hz */
    BMI160_ACCEL_RATE_25HZ,        /**<   25    Hz */
    BMI160_ACCEL_RATE_50HZ,        /**<   50    Hz */
    BMI160_ACCEL_RATE_100HZ,       /**<  100    Hz */
    BMI160_ACCEL_RATE_200HZ,       /**<  200    Hz */
    BMI160_ACCEL_RATE_400HZ,       /**<  400    Hz */
    BMI160_ACCEL_RATE_800HZ,       /**<  800    Hz */
    BMI160_ACCEL_RATE_1600HZ,      /**< 1600    Hz */
} bmi160_accel_rate_t;

// Gyroscope Output Data Rate options
typedef enum {
    BMI160_GYRO_RATE_25HZ = 6,     /**<   25    Hz */
    BMI160_GYRO_RATE_50HZ,         /**<   50    Hz */
    BMI160_GYRO_RATE_100HZ,        /**<  100    Hz */
    BMI160_GYRO_RATE_200HZ,        /**<  200    Hz */
    BMI160_GYRO_RATE_400HZ,        /**<  400    Hz */
    BMI160_GYRO_RATE_800HZ,        /**<  800    Hz */
    BMI160_GYRO_RATE_1600HZ,       /**< 1600    Hz */
    BMI160_GYRO_RATE_3200HZ,       /**< 3200    Hz */
} bmi160_gyro_rate_t;

// Digital Low-Pass Filter Mode options
typedef enum {
    BMI160_DLPF_MODE_NORM = 0x2,
    BMI160_DLPF_MODE_OSR2 = 0x1,
    BMI160_DLPF_MODE_OSR4 = 0x0,
} bmi160_dlpf_mode_t;

// Step Detection Mode options
typedef enum {
    BMI160_STEP_MODE_NORMAL = 0,
    BMI160_STEP_MODE_SENSITIVE,
    BMI160_STEP_MODE_ROBUST,
    BMI160_STEP_MODE_UNKNOWN,
} bmi160_step_mode_t;

// Tap Detection Shock Duration options
typedef enum {
    BMI160_TAP_SHOCK_DURATION_50MS = 0,
    BMI160_TAP_SHOCK_DURATION_75MS,
} bmi160_tap_shock_duration_t;

// Tap Detection Quiet Duration options
typedef enum {
    BMI160_TAP_QUIET_DURATION_30MS = 0,
    BMI160_TAP_QUIET_DURATION_20MS,
} bmi160_tap_quiet_duration_t;

// Double-Tap Detection Duration options
typedef enum {
    BMI160_DOUBLE_TAP_DURATION_50MS = 0,
    BMI160_DOUBLE_TAP_DURATION_100MS,
    BMI160_DOUBLE_TAP_DURATION_150MS,
    BMI160_DOUBLE_TAP_DURATION_200MS,
    BMI160_DOUBLE_TAP_DURATION_250MS,
    BMI160_DOUBLE_TAP_DURATION_375MS,
    BMI160_DOUBLE_TAP_DURATION_500MS,
    BMI160_DOUBLE_TAP_DURATION_700MS,
} bmi160_double_tap_duration_t;

// Interrupt Latch Mode options
typedef enum {
    BMI160_LATCH_MODE_NONE = 0, /**< Non-latched */
    BMI160_LATCH_MODE_312_5_US, /**< Temporary, 312.50 microseconds */
    BMI160_LATCH_MODE_625_US,   /**< Temporary, 625.00 microseconds */
    BMI160_LATCH_MODE_1_25_MS,  /**< Temporary,   1.25 milliseconds */
    BMI160_LATCH_MODE_2_5_MS,   /**< Temporary,   2.50 milliseconds */
    BMI160_LATCH_MODE_5_MS,     /**< Temporary,   5.00 milliseconds */
    BMI160_LATCH_MODE_10_MS,    /**< Temporary,  10.00 milliseconds */
    BMI160_LATCH_MODE_20_MS,    /**< Temporary,  20.00 milliseconds */
    BMI160_LATCH_MODE_40_MS,    /**< Temporary,  40.00 milliseconds */
    BMI160_LATCH_MODE_80_MS,    /**< Temporary,  80.00 milliseconds */
    BMI160_LATCH_MODE_160_MS,   /**< Temporary, 160.00 milliseconds */
    BMI160_LATCH_MODE_320_MS,   /**< Temporary, 320.00 milliseconds */
    BMI160_LATCH_MODE_640_MS,   /**< Temporary, 640.00 milliseconds */
    BMI160_LATCH_MODE_1_28_S,   /**< Temporary,   1.28 seconds      */
    BMI160_LATCH_MODE_2_56_S,   /**< Temporary,   2.56 seconds      */
    BMI160_LATCH_MODE_LATCH,    /**< Latched, @see resetInterrupt() */
} bmi160_interrupt_latch_mode_t;

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

// Basic functions
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

// Device ID and connection test
esp_err_t bmi160_get_device_id(bmi160_handle_t handle, uint8_t *device_id);
esp_err_t bmi160_test_connection(bmi160_handle_t handle, bool *is_connected);

// Data rate configuration
esp_err_t bmi160_get_gyro_rate(bmi160_handle_t handle, uint8_t *rate);
esp_err_t bmi160_set_gyro_rate(bmi160_handle_t handle, uint8_t rate);
esp_err_t bmi160_get_accel_rate(bmi160_handle_t handle, uint8_t *rate);
esp_err_t bmi160_set_accel_rate(bmi160_handle_t handle, uint8_t rate);

// Digital Low-Pass Filter configuration
esp_err_t bmi160_get_gyro_dlpf_mode(bmi160_handle_t handle, uint8_t *mode);
esp_err_t bmi160_set_gyro_dlpf_mode(bmi160_handle_t handle, uint8_t mode);
esp_err_t bmi160_get_accel_dlpf_mode(bmi160_handle_t handle, uint8_t *mode);
esp_err_t bmi160_set_accel_dlpf_mode(bmi160_handle_t handle, uint8_t mode);

// Range configuration
esp_err_t bmi160_get_full_scale_gyro_range(bmi160_handle_t handle, uint8_t *range);
esp_err_t bmi160_set_full_scale_gyro_range(bmi160_handle_t handle, uint8_t range);
esp_err_t bmi160_get_full_scale_accel_range(bmi160_handle_t handle, uint8_t *range);
esp_err_t bmi160_set_full_scale_accel_range(bmi160_handle_t handle, uint8_t range);

// Calibration functions
esp_err_t bmi160_auto_calibrate_gyro_offset(bmi160_handle_t handle);
esp_err_t bmi160_get_gyro_offset_enabled(bmi160_handle_t handle, bool *enabled);
esp_err_t bmi160_set_gyro_offset_enabled(bmi160_handle_t handle, bool enabled);
esp_err_t bmi160_get_x_gyro_offset(bmi160_handle_t handle, int16_t *offset);
esp_err_t bmi160_set_x_gyro_offset(bmi160_handle_t handle, int16_t offset);
esp_err_t bmi160_get_y_gyro_offset(bmi160_handle_t handle, int16_t *offset);
esp_err_t bmi160_set_y_gyro_offset(bmi160_handle_t handle, int16_t offset);
esp_err_t bmi160_get_z_gyro_offset(bmi160_handle_t handle, int16_t *offset);
esp_err_t bmi160_set_z_gyro_offset(bmi160_handle_t handle, int16_t offset);

esp_err_t bmi160_auto_calibrate_x_accel_offset(bmi160_handle_t handle, int target);
esp_err_t bmi160_auto_calibrate_y_accel_offset(bmi160_handle_t handle, int target);
esp_err_t bmi160_auto_calibrate_z_accel_offset(bmi160_handle_t handle, int target);
esp_err_t bmi160_get_accel_offset_enabled(bmi160_handle_t handle, bool *enabled);
esp_err_t bmi160_set_accel_offset_enabled(bmi160_handle_t handle, bool enabled);
esp_err_t bmi160_get_x_accel_offset(bmi160_handle_t handle, int8_t *offset);
esp_err_t bmi160_set_x_accel_offset(bmi160_handle_t handle, int8_t offset);
esp_err_t bmi160_get_y_accel_offset(bmi160_handle_t handle, int8_t *offset);
esp_err_t bmi160_set_y_accel_offset(bmi160_handle_t handle, int8_t offset);
esp_err_t bmi160_get_z_accel_offset(bmi160_handle_t handle, int8_t *offset);
esp_err_t bmi160_set_z_accel_offset(bmi160_handle_t handle, int8_t offset);

// Motion detection configuration
esp_err_t bmi160_get_freefall_detection_threshold(bmi160_handle_t handle, uint8_t *threshold);
esp_err_t bmi160_set_freefall_detection_threshold(bmi160_handle_t handle, uint8_t threshold);
esp_err_t bmi160_get_freefall_detection_duration(bmi160_handle_t handle, uint8_t *duration);
esp_err_t bmi160_set_freefall_detection_duration(bmi160_handle_t handle, uint8_t duration);

esp_err_t bmi160_get_shock_detection_threshold(bmi160_handle_t handle, uint8_t *threshold);
esp_err_t bmi160_set_shock_detection_threshold(bmi160_handle_t handle, uint8_t threshold);
esp_err_t bmi160_get_shock_detection_duration(bmi160_handle_t handle, uint8_t *duration);
esp_err_t bmi160_set_shock_detection_duration(bmi160_handle_t handle, uint8_t duration);

esp_err_t bmi160_get_motion_detection_threshold(bmi160_handle_t handle, uint8_t *threshold);
esp_err_t bmi160_set_motion_detection_threshold(bmi160_handle_t handle, uint8_t threshold);
esp_err_t bmi160_get_motion_detection_duration(bmi160_handle_t handle, uint8_t *duration);
esp_err_t bmi160_set_motion_detection_duration(bmi160_handle_t handle, uint8_t duration);

esp_err_t bmi160_get_zero_motion_detection_threshold(bmi160_handle_t handle, uint8_t *threshold);
esp_err_t bmi160_set_zero_motion_detection_threshold(bmi160_handle_t handle, uint8_t threshold);
esp_err_t bmi160_get_zero_motion_detection_duration(bmi160_handle_t handle, uint8_t *duration);
esp_err_t bmi160_set_zero_motion_detection_duration(bmi160_handle_t handle, uint8_t duration);

// Tap detection configuration
esp_err_t bmi160_get_tap_detection_threshold(bmi160_handle_t handle, uint8_t *threshold);
esp_err_t bmi160_set_tap_detection_threshold(bmi160_handle_t handle, uint8_t threshold);
esp_err_t bmi160_get_tap_shock_duration(bmi160_handle_t handle, bool *duration);
esp_err_t bmi160_set_tap_shock_duration(bmi160_handle_t handle, bool duration);
esp_err_t bmi160_get_tap_quiet_duration(bmi160_handle_t handle, bool *duration);
esp_err_t bmi160_set_tap_quiet_duration(bmi160_handle_t handle, bool duration);
esp_err_t bmi160_get_double_tap_detection_duration(bmi160_handle_t handle, uint8_t *duration);
esp_err_t bmi160_set_double_tap_detection_duration(bmi160_handle_t handle, uint8_t duration);

// Step detection functions
esp_err_t bmi160_get_step_detection_mode(bmi160_handle_t handle, uint8_t *mode);
esp_err_t bmi160_set_step_detection_mode(bmi160_handle_t handle, bmi160_step_mode_t mode);
esp_err_t bmi160_get_step_count_enabled(bmi160_handle_t handle, bool *enabled);
esp_err_t bmi160_set_step_count_enabled(bmi160_handle_t handle, bool enabled);
esp_err_t bmi160_get_step_count(bmi160_handle_t handle, uint16_t *count);
esp_err_t bmi160_reset_step_count(bmi160_handle_t handle);

// Interrupt enable/disable functions
esp_err_t bmi160_get_int_freefall_enabled(bmi160_handle_t handle, bool *enabled);
esp_err_t bmi160_set_int_freefall_enabled(bmi160_handle_t handle, bool enabled);
esp_err_t bmi160_get_int_shock_enabled(bmi160_handle_t handle, bool *enabled);
esp_err_t bmi160_set_int_shock_enabled(bmi160_handle_t handle, bool enabled);
esp_err_t bmi160_get_int_step_enabled(bmi160_handle_t handle, bool *enabled);
esp_err_t bmi160_set_int_step_enabled(bmi160_handle_t handle, bool enabled);
esp_err_t bmi160_get_int_motion_enabled(bmi160_handle_t handle, bool *enabled);
esp_err_t bmi160_set_int_motion_enabled(bmi160_handle_t handle, bool enabled);
esp_err_t bmi160_get_int_zero_motion_enabled(bmi160_handle_t handle, bool *enabled);
esp_err_t bmi160_set_int_zero_motion_enabled(bmi160_handle_t handle, bool enabled);
esp_err_t bmi160_get_int_tap_enabled(bmi160_handle_t handle, bool *enabled);
esp_err_t bmi160_set_int_tap_enabled(bmi160_handle_t handle, bool enabled);
esp_err_t bmi160_get_int_double_tap_enabled(bmi160_handle_t handle, bool *enabled);
esp_err_t bmi160_set_int_double_tap_enabled(bmi160_handle_t handle, bool enabled);

// FIFO functions
esp_err_t bmi160_get_gyro_fifo_enabled(bmi160_handle_t handle, bool *enabled);
esp_err_t bmi160_set_gyro_fifo_enabled(bmi160_handle_t handle, bool enabled);
esp_err_t bmi160_get_accel_fifo_enabled(bmi160_handle_t handle, bool *enabled);
esp_err_t bmi160_set_accel_fifo_enabled(bmi160_handle_t handle, bool enabled);
esp_err_t bmi160_get_int_fifo_buffer_full_enabled(bmi160_handle_t handle, bool *enabled);
esp_err_t bmi160_set_int_fifo_buffer_full_enabled(bmi160_handle_t handle, bool enabled);
esp_err_t bmi160_get_int_data_ready_enabled(bmi160_handle_t handle, bool *enabled);
esp_err_t bmi160_set_int_data_ready_enabled(bmi160_handle_t handle, bool enabled);

// FIFO data functions
esp_err_t bmi160_get_fifo_header_mode_enabled(bmi160_handle_t handle, bool *enabled);
esp_err_t bmi160_set_fifo_header_mode_enabled(bmi160_handle_t handle, bool enabled);
esp_err_t bmi160_reset_fifo(bmi160_handle_t handle);
esp_err_t bmi160_get_fifo_count(bmi160_handle_t handle, uint16_t *count);
esp_err_t bmi160_get_fifo_bytes(bmi160_handle_t handle, uint8_t *data, uint16_t length);

// FIFO headerless mode functions
esp_err_t bmi160_read_fifo_headerless_accel(bmi160_handle_t handle, int16_t *accel_data, uint16_t *sample_count);
esp_err_t bmi160_read_fifo_headerless_accel_gyro(bmi160_handle_t handle, int16_t *accel_data, int16_t *gyro_data, uint16_t *sample_count);
esp_err_t bmi160_init_headerless_mode(bmi160_handle_t handle, bool enable_accel, bool enable_gyro);

// Interrupt status functions
esp_err_t bmi160_get_int_status0(bmi160_handle_t handle, uint8_t *status);
esp_err_t bmi160_get_int_status1(bmi160_handle_t handle, uint8_t *status);
esp_err_t bmi160_get_int_status2(bmi160_handle_t handle, uint8_t *status);
esp_err_t bmi160_get_int_status3(bmi160_handle_t handle, uint8_t *status);
esp_err_t bmi160_get_int_freefall_status(bmi160_handle_t handle, bool *status);
esp_err_t bmi160_get_int_shock_status(bmi160_handle_t handle, bool *status);
esp_err_t bmi160_get_int_step_status(bmi160_handle_t handle, bool *status);
esp_err_t bmi160_get_int_motion_status(bmi160_handle_t handle, bool *status);
esp_err_t bmi160_get_int_zero_motion_status(bmi160_handle_t handle, bool *status);
esp_err_t bmi160_get_int_tap_status(bmi160_handle_t handle, bool *status);
esp_err_t bmi160_get_int_double_tap_status(bmi160_handle_t handle, bool *status);
esp_err_t bmi160_get_int_fifo_buffer_full_status(bmi160_handle_t handle, bool *status);
esp_err_t bmi160_get_int_data_ready_status(bmi160_handle_t handle, bool *status);

// Motion detection status functions
esp_err_t bmi160_get_x_neg_shock_detected(bmi160_handle_t handle, bool *detected);
esp_err_t bmi160_get_x_pos_shock_detected(bmi160_handle_t handle, bool *detected);
esp_err_t bmi160_get_y_neg_shock_detected(bmi160_handle_t handle, bool *detected);
esp_err_t bmi160_get_y_pos_shock_detected(bmi160_handle_t handle, bool *detected);
esp_err_t bmi160_get_z_neg_shock_detected(bmi160_handle_t handle, bool *detected);
esp_err_t bmi160_get_z_pos_shock_detected(bmi160_handle_t handle, bool *detected);

esp_err_t bmi160_get_x_neg_motion_detected(bmi160_handle_t handle, bool *detected);
esp_err_t bmi160_get_x_pos_motion_detected(bmi160_handle_t handle, bool *detected);
esp_err_t bmi160_get_y_neg_motion_detected(bmi160_handle_t handle, bool *detected);
esp_err_t bmi160_get_y_pos_motion_detected(bmi160_handle_t handle, bool *detected);
esp_err_t bmi160_get_z_neg_motion_detected(bmi160_handle_t handle, bool *detected);
esp_err_t bmi160_get_z_pos_motion_detected(bmi160_handle_t handle, bool *detected);

esp_err_t bmi160_get_x_neg_tap_detected(bmi160_handle_t handle, bool *detected);
esp_err_t bmi160_get_x_pos_tap_detected(bmi160_handle_t handle, bool *detected);
esp_err_t bmi160_get_y_neg_tap_detected(bmi160_handle_t handle, bool *detected);
esp_err_t bmi160_get_y_pos_tap_detected(bmi160_handle_t handle, bool *detected);
esp_err_t bmi160_get_z_neg_tap_detected(bmi160_handle_t handle, bool *detected);
esp_err_t bmi160_get_z_pos_tap_detected(bmi160_handle_t handle, bool *detected);

// Interrupt configuration functions
esp_err_t bmi160_get_int_enabled(bmi160_handle_t handle, bool *enabled);
esp_err_t bmi160_set_int_enabled(bmi160_handle_t handle, bool enabled);
esp_err_t bmi160_get_interrupt_mode(bmi160_handle_t handle, bool *mode);
esp_err_t bmi160_set_interrupt_mode(bmi160_handle_t handle, bool mode);
esp_err_t bmi160_get_interrupt_drive(bmi160_handle_t handle, bool *drive);
esp_err_t bmi160_set_interrupt_drive(bmi160_handle_t handle, bool drive);
esp_err_t bmi160_get_interrupt_latch(bmi160_handle_t handle, uint8_t *latch);
esp_err_t bmi160_set_interrupt_latch(bmi160_handle_t handle, uint8_t latch);
esp_err_t bmi160_reset_interrupt(bmi160_handle_t handle);

// Individual sensor reading functions
esp_err_t bmi160_get_acceleration_x(bmi160_handle_t handle, int16_t *x);
esp_err_t bmi160_get_acceleration_y(bmi160_handle_t handle, int16_t *y);
esp_err_t bmi160_get_acceleration_z(bmi160_handle_t handle, int16_t *z);
esp_err_t bmi160_get_rotation_x(bmi160_handle_t handle, int16_t *x);
esp_err_t bmi160_get_rotation_y(bmi160_handle_t handle, int16_t *y);
esp_err_t bmi160_get_rotation_z(bmi160_handle_t handle, int16_t *z);
esp_err_t bmi160_get_temperature(bmi160_handle_t handle, int16_t *temperature);

// Register access functions
esp_err_t bmi160_get_register(bmi160_handle_t handle, uint8_t reg, uint8_t *data);
esp_err_t bmi160_set_register(bmi160_handle_t handle, uint8_t reg, uint8_t data);


