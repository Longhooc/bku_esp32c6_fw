/* Sensor Module Header
 * 
 * This module handles BMI160 sensor initialization, configuration, and data polling.
 * It provides a clean interface for sensor operations and encapsulates all BMI160-related code.
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "bmi160.h"

#ifdef __cplusplus
extern "C" {
#endif

// Sensor task context structure
typedef struct {
    bmi160_handle_t handle;
    int acc_mg_per_lsb_x1000;      // mg per LSB * 1000 (e.g. 244 for 0.244 mg/LSB)
    int gyr_lsb_per_dps_x1000;     // LSB per dps * 1000 (e.g. 65536 for 65.536 LSB/(°/s))
    int poll_interval_ms;          // Polling interval in milliseconds
} sensor_task_ctx_t;

// Sensor configuration structure
typedef struct {
    int gpio_cs;           // SPI CS GPIO
    int gpio_mosi;         // SPI MOSI GPIO
    int gpio_miso;         // SPI MISO GPIO
    int gpio_sck;          // SPI SCK GPIO
    int gpio_int1;         // INT1 GPIO (motion/wakeup)
    int gpio_int2;         // INT2 GPIO (wakeup/fifo/step)
    int spi_host;          // SPI2_HOST, SPI3_HOST, etc.
    int clock_speed_hz;    // SPI clock
    int acc_range;         // Accelerometer range (BMI160_ACC_RANGE_*)
    int gyr_range;         // Gyroscope range (BMI160_GYR_RANGE_*)
    int anymotion_threshold_mg;        // Any-motion threshold in mg
    int anymotion_duration;            // Any-motion duration in samples
    int poll_interval_ms;  // Polling interval in milliseconds
} sensor_config_t;

// Default sensor configuration
#define SENSOR_DEFAULT_CONFIG() { \
    .gpio_cs = 11, \
    .gpio_mosi = 10, \
    .gpio_miso = 13, \
    .gpio_sck = 12, \
    .gpio_int1 = 4, \
    .gpio_int2 = 18, \
    .spi_host = SPI2_HOST, \
    .clock_speed_hz = 5 * 1000 * 1000, \
    .acc_range = BMI160_ACC_RANGE_16G, \
    .gyr_range = BMI160_GYR_RANGE_500_DPS, \
    .anymotion_threshold_mg = 7960, /* Max threshold for 16G range (255 LSB * 31.25mg = 7968mg ~ 8g) */ \
    .anymotion_duration = 3, /* Valid: 0-3 only (2 bits). Duration = anymotion_duration + 1 samples */ \
    .poll_interval_ms = 200 \
}

/**
 * @brief Initialize sensor module with default configuration
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sensor_init(void);

/**
 * @brief Initialize sensor module with custom configuration
 * 
 * @param config Custom sensor configuration
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sensor_init_with_config(const sensor_config_t *config);

/**
 * @brief Deinitialize sensor module and cleanup resources
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sensor_deinit(void);

/**
 * @brief Get current sensor handle
 * 
 * @return bmi160_handle_t Sensor handle or NULL if not initialized
 */
bmi160_handle_t sensor_get_handle(void);

/**
 * @brief Check if sensor is initialized
 * 
 * @return true if sensor is initialized
 * @return false if sensor is not initialized
 */
bool sensor_is_initialized(void);

/**
 * @brief Suspend sensor polling task
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sensor_suspend_poll_task(void);

/**
 * @brief Resume sensor polling task
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sensor_resume_poll_task(void);

#ifdef __cplusplus
}
#endif

/**
 * @brief Resume sensor from deep sleep without resetting it
 * 
 * This function re-initializes the SPI bus and handle but assumes
 * the sensor is already configured and running.
 */
esp_err_t sensor_resume(void);

#endif // SENSOR_H
