/* Battery Voltage Reading Module
 * 
 * This module handles reading LiFePO4 battery voltage from ADC.
 * Supports voltage divider for higher voltage measurements.
 */

#include "battery.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "battery_voltage";

// Battery voltage reading context
static struct {
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle;
    int adc_channel;
    float voltage_divider_ratio;
    bool initialized;
} battery_ctx = {
    .adc_handle = NULL,
    .cali_handle = NULL,
    .adc_channel = -1,
    .voltage_divider_ratio = 1.0f,
    .initialized = false
};

/**
 * @brief Calibration callback function for ADC
 */
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibration scheme: Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibration scheme: Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration successful");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "ADC calibration not supported, using raw ADC values");
    } else {
        ESP_LOGE(TAG, "ADC calibration failed");
    }

    return calibrated;
}

esp_err_t battery_voltage_init(int adc_channel, float voltage_divider_ratio)
{
    if (battery_ctx.initialized) {
        ESP_LOGW(TAG, "Battery voltage module already initialized");
        return ESP_OK;
    }

    if (voltage_divider_ratio <= 0.0f) {
        ESP_LOGE(TAG, "Invalid voltage divider ratio: %.2f", voltage_divider_ratio);
        return ESP_ERR_INVALID_ARG;
    }

    // Configure ADC unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &battery_ctx.adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure ADC channel
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12, // 0-3.9V range on ESP32-C6
    };
    ret = adc_oneshot_config_channel(battery_ctx.adc_handle, (adc_channel_t)adc_channel, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel %d: %s", adc_channel, esp_err_to_name(ret));
        adc_oneshot_del_unit(battery_ctx.adc_handle);
        battery_ctx.adc_handle = NULL;
        return ret;
    }

    // Initialize calibration
    adc_calibration_init(ADC_UNIT_1, (adc_channel_t)adc_channel, ADC_ATTEN_DB_12, &battery_ctx.cali_handle);

    battery_ctx.adc_channel = adc_channel;
    battery_ctx.voltage_divider_ratio = voltage_divider_ratio;
    battery_ctx.initialized = true;

    ESP_LOGI(TAG, "Battery voltage module initialized - ADC channel: %d, Divider ratio: %.2f", 
             adc_channel, voltage_divider_ratio);

    return ESP_OK;
}

esp_err_t battery_voltage_read_mv(uint32_t *voltage_mv)
{
    if (!battery_ctx.initialized) {
        ESP_LOGE(TAG, "Battery voltage module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (voltage_mv == NULL) {
        ESP_LOGE(TAG, "Invalid pointer for voltage_mv");
        return ESP_ERR_INVALID_ARG;
    }

    int adc_raw = 0;
    int voltage_raw_mv = 0;

    // Read ADC raw value
    esp_err_t ret = adc_oneshot_read(battery_ctx.adc_handle, (adc_channel_t)battery_ctx.adc_channel, &adc_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    // Convert to voltage (millivolts)
    // For ADC_ATTEN_DB_12 on ESP32-C6: 0-3.9V range (3900mV)
    if (battery_ctx.cali_handle) {
        ret = adc_cali_raw_to_voltage(battery_ctx.cali_handle, adc_raw, &voltage_raw_mv);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Calibration failed, using simple conversion");
            // Fallback: simple conversion for ADC_ATTEN_DB_12 (0-3.9V range)
            voltage_raw_mv = (adc_raw * 3900) / 4095;
        }
    } else {
        // No calibration available, use simple conversion
        // ADC_ATTEN_DB_12: 0-3.9V range (3900mV)
        voltage_raw_mv = (adc_raw * 3900) / 4095;
    }

    // Apply voltage divider ratio
    *voltage_mv = (uint32_t)(voltage_raw_mv * battery_ctx.voltage_divider_ratio + OFFSET_VOLTAGE);

    return ESP_OK;
}

esp_err_t battery_voltage_read_v(float *voltage_v)
{
    if (voltage_v == NULL) {
        ESP_LOGE(TAG, "Invalid pointer for voltage_v");
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t voltage_mv = 0;
    esp_err_t ret = battery_voltage_read_mv(&voltage_mv);
    if (ret != ESP_OK) {
        return ret;
    }

    *voltage_v = (float)voltage_mv / 1000.0f ;
    return ESP_OK;
}

esp_err_t battery_voltage_deinit(void)
{
    if (!battery_ctx.initialized) {
        return ESP_OK;
    }

    if (battery_ctx.cali_handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_delete_scheme_curve_fitting(battery_ctx.cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_delete_scheme_line_fitting(battery_ctx.cali_handle);
#endif
        battery_ctx.cali_handle = NULL;
    }

    if (battery_ctx.adc_handle) {
        adc_oneshot_del_unit(battery_ctx.adc_handle);
        battery_ctx.adc_handle = NULL;
    }

    battery_ctx.adc_channel = -1;
    battery_ctx.voltage_divider_ratio = 1.0f;
    battery_ctx.initialized = false;

    ESP_LOGI(TAG, "Battery voltage module deinitialized");
    return ESP_OK;
}

bool battery_voltage_is_initialized(void)
{
    return battery_ctx.initialized;
}

