/* Battery Voltage Reading Module Header
 * 
 * This module handles reading LiFePO4 battery voltage from ADC.
 * It provides a clean interface for battery voltage monitoring.
 */

#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define OFFSET_VOLTAGE 90

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize battery voltage reading module
 * 
 * @param adc_channel ADC channel number (e.g., ADC1_CHANNEL_0)
 * @param voltage_divider_ratio Voltage divider ratio (e.g., 2.0 for 2:1 divider)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t battery_voltage_init(int adc_channel, float voltage_divider_ratio);

/**
 * @brief Read battery voltage in millivolts
 * 
 * @param voltage_mv Pointer to store voltage in millivolts
 * @return esp_err_t ESP_OK on success
 */
esp_err_t battery_voltage_read_mv(uint32_t *voltage_mv);

/**
 * @brief Read battery voltage in volts
 * 
 * @param voltage_v Pointer to store voltage in volts
 * @return esp_err_t ESP_OK on success
 */
esp_err_t battery_voltage_read_v(float *voltage_v);

/**
 * @brief Deinitialize battery voltage reading module
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t battery_voltage_deinit(void);

/**
 * @brief Check if battery module is initialized
 * 
 * @return true if initialized
 * @return false if not initialized
 */
bool battery_voltage_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif // BATTERY_H

