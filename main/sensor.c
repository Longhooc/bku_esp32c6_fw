/* Sensor Module Implementation
 * 
 * This module handles BMI160 sensor initialization, configuration, and data polling.
 * It provides a clean interface for sensor operations and encapsulates all BMI160-related code.
 */

#include "sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include <stdlib.h>

static const char *TAG = "sensor";

// Static variables for sensor state management
static sensor_task_ctx_t *s_sensor_ctx = NULL;
static bmi160_handle_t s_sensor_handle = NULL;
static bool s_sensor_initialized = false;

/**
 * @brief BMI160 polling task
 * 
 * This task continuously reads sensor data and logs it with proper unit conversion.
 * 
 * @param pv Task parameter (sensor_task_ctx_t*)
 */
static void sensor_poll_task(void *pv)
{
    sensor_task_ctx_t *ctx = (sensor_task_ctx_t *)pv;
    bmi160_sample_t sample;
    uint32_t istat;
    
    ESP_LOGI(TAG, "Sensor polling task started");
    
    for (;;) {
        if (bmi160_read_sample(ctx->handle, &sample) == ESP_OK) {
            // Convert ACC raw -> mg (int64) with sign-aware rounding
            int64_t ax_mg = ((int64_t)sample.accel_x * (int64_t)ctx->acc_mg_per_lsb_x1000 + (sample.accel_x >= 0 ? 500 : -500)) / 1000;
            int64_t ay_mg = ((int64_t)sample.accel_y * (int64_t)ctx->acc_mg_per_lsb_x1000 + (sample.accel_y >= 0 ? 500 : -500)) / 1000;
            int64_t az_mg = ((int64_t)sample.accel_z * (int64_t)ctx->acc_mg_per_lsb_x1000 + (sample.accel_z >= 0 ? 500 : -500)) / 1000;
            
            // Convert mg -> m/s^2 * 100 (since 1 mg = 9.80665e-3 m/s^2)
            int64_t ax_ms2_x100 = (ax_mg * 980665 + (ax_mg >= 0 ? 500000 : -500000)) / 1000000;
            int64_t ay_ms2_x100 = (ay_mg * 980665 + (ay_mg >= 0 ? 500000 : -500000)) / 1000000;
            int64_t az_ms2_x100 = (az_mg * 980665 + (az_mg >= 0 ? 500000 : -500000)) / 1000000;

            // Convert GYR raw -> dps*100 using LSB per dps * 1000 with sign-aware rounding
            int64_t gx_dps_x100 = ((int64_t)sample.gyro_x * 100000 + (sample.gyro_x >= 0 ? (ctx->gyr_lsb_per_dps_x1000/2) : -(ctx->gyr_lsb_per_dps_x1000/2))) / (int64_t)ctx->gyr_lsb_per_dps_x1000;
            int64_t gy_dps_x100 = ((int64_t)sample.gyro_y * 100000 + (sample.gyro_y >= 0 ? (ctx->gyr_lsb_per_dps_x1000/2) : -(ctx->gyr_lsb_per_dps_x1000/2))) / (int64_t)ctx->gyr_lsb_per_dps_x1000;
            int64_t gz_dps_x100 = ((int64_t)sample.gyro_z * 100000 + (sample.gyro_z >= 0 ? (ctx->gyr_lsb_per_dps_x1000/2) : -(ctx->gyr_lsb_per_dps_x1000/2))) / (int64_t)ctx->gyr_lsb_per_dps_x1000;

            // Format acceleration values for display
            const char *sx = (ax_ms2_x100 < 0) ? "-" : "";
            const char *sy = (ay_ms2_x100 < 0) ? "-" : "";
            const char *sz = (az_ms2_x100 < 0) ? "-" : "";
            long long ax_whole = (long long)(ax_ms2_x100 < 0 ? -ax_ms2_x100 : ax_ms2_x100) / 100;
            long long ay_whole = (long long)(ay_ms2_x100 < 0 ? -ay_ms2_x100 : ay_ms2_x100) / 100;
            long long az_whole = (long long)(az_ms2_x100 < 0 ? -az_ms2_x100 : az_ms2_x100) / 100;
            int ax_frac = (int)((ax_ms2_x100 < 0 ? -ax_ms2_x100 : ax_ms2_x100) % 100);
            int ay_frac = (int)((ay_ms2_x100 < 0 ? -ay_ms2_x100 : ay_ms2_x100) % 100);
            int az_frac = (int)((az_ms2_x100 < 0 ? -az_ms2_x100 : az_ms2_x100) % 100);

            // Format gyroscope values for display
            const char *sgx = (gx_dps_x100 < 0) ? "-" : "";
            const char *sgy = (gy_dps_x100 < 0) ? "-" : "";
            const char *sgz = (gz_dps_x100 < 0) ? "-" : "";
            long long gx_whole = (long long)(gx_dps_x100 < 0 ? -gx_dps_x100 : gx_dps_x100) / 100;
            long long gy_whole = (long long)(gy_dps_x100 < 0 ? -gy_dps_x100 : gy_dps_x100) / 100;
            long long gz_whole = (long long)(gz_dps_x100 < 0 ? -gz_dps_x100 : gz_dps_x100) / 100;
            int gx_frac = (int)((gx_dps_x100 < 0 ? -gx_dps_x100 : gx_dps_x100) % 100);
            int gy_frac = (int)((gy_dps_x100 < 0 ? -gy_dps_x100 : gy_dps_x100) % 100);
            int gz_frac = (int)((gz_dps_x100 < 0 ? -gz_dps_x100 : gz_dps_x100) % 100);

            ESP_LOGI("bmi160", "acc(ms2)= %s%lld.%02d, %s%lld.%02d, %s%lld.%02d | gyr(dps)= %s%lld.%02d, %s%lld.%02d, %s%lld.%02d | t=%ld.%02ldC",
                sx, ax_whole, ax_frac,
                sy, ay_whole, ay_frac,
                sz, az_whole, az_frac,
                sgx, gx_whole, gx_frac,
                sgy, gy_whole, gy_frac,
                sgz, gz_whole, gz_frac,
                (long)(sample.temperature_c_x100/100), (long)(sample.temperature_c_x100%100));
        }
        
        // Check interrupt status
        if (bmi160_read_int_status(ctx->handle, &istat) == ESP_OK && istat) {
            ESP_LOGE("bmi160", "int_status=0x%08lx", (unsigned long)istat);
        }
        
        vTaskDelay(pdMS_TO_TICKS(ctx->poll_interval_ms));
    }
}

/**
 * @brief Initialize sensor with default configuration
 */
esp_err_t sensor_init(void)
{
    sensor_config_t config = SENSOR_DEFAULT_CONFIG();
    return sensor_init_with_config(&config);
}

/**
 * @brief Initialize sensor with custom configuration
 */
esp_err_t sensor_init_with_config(const sensor_config_t *config)
{
    if (s_sensor_initialized) {
        ESP_LOGW(TAG, "Sensor already initialized");
        return ESP_OK;
    }
    
    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create BMI160 SPI bus configuration
    bmi160_spi_bus_config_t bmi_bus = {
        .gpio_cs = config->gpio_cs,
        .gpio_mosi = config->gpio_mosi,
        .gpio_miso = config->gpio_miso,
        .gpio_sck = config->gpio_sck,
        .gpio_int1 = config->gpio_int1,
        .gpio_int2 = config->gpio_int2,
        .spi_host = config->spi_host,
        .clock_speed_hz = config->clock_speed_hz,
    };
    
    // Create BMI160 handle
    esp_err_t ret = bmi160_create(&bmi_bus, &s_sensor_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create BMI160 handle: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize BMI160 with default settings
    ret = bmi160_init_default(s_sensor_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMI160: %s", esp_err_to_name(ret));
        bmi160_destroy(s_sensor_handle);
        s_sensor_handle = NULL;
        return ret;
    }
    
    // Configure sensor ranges
    ret = bmi160_config_ranges(s_sensor_handle, config->acc_range, config->gyr_range);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure BMI160 ranges: %s", esp_err_to_name(ret));
        bmi160_destroy(s_sensor_handle);
        s_sensor_handle = NULL;
        return ret;
    }
    
    // Enable any-motion wakeup
    ret = bmi160_enable_anymotion_wakeup_ms2(s_sensor_handle, 
                                           config->anymotion_threshold_ms2_x100, 
                                           config->anymotion_duration);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable any-motion wakeup: %s", esp_err_to_name(ret));
        bmi160_destroy(s_sensor_handle);
        s_sensor_handle = NULL;
        return ret;
    }
    
    // Allocate task context
    s_sensor_ctx = (sensor_task_ctx_t *)malloc(sizeof(sensor_task_ctx_t));
    if (s_sensor_ctx == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for sensor task context");
        bmi160_destroy(s_sensor_handle);
        s_sensor_handle = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize task context
    s_sensor_ctx->handle = s_sensor_handle;
    s_sensor_ctx->poll_interval_ms = config->poll_interval_ms;
    
    // Set unit conversion scales based on configured ranges
    switch (config->acc_range) {
        case BMI160_ACC_RANGE_2G:
            s_sensor_ctx->acc_mg_per_lsb_x1000 = 61;   // 0.061 mg/LSB
            break;
        case BMI160_ACC_RANGE_4G:
            s_sensor_ctx->acc_mg_per_lsb_x1000 = 122;  // 0.122 mg/LSB
            break;
        case BMI160_ACC_RANGE_8G:
            s_sensor_ctx->acc_mg_per_lsb_x1000 = 244;  // 0.244 mg/LSB
            break;
        case BMI160_ACC_RANGE_16G:
            s_sensor_ctx->acc_mg_per_lsb_x1000 = 488;  // 0.488 mg/LSB
            break;
        default:
            s_sensor_ctx->acc_mg_per_lsb_x1000 = 244;  // Default to 8G
            break;
    }
    
    switch (config->gyr_range) {
        case BMI160_GYR_RANGE_2000_DPS:
            s_sensor_ctx->gyr_lsb_per_dps_x1000 = 16384;  // 16.384 LSB/(°/s)
            break;
        case BMI160_GYR_RANGE_1000_DPS:
            s_sensor_ctx->gyr_lsb_per_dps_x1000 = 32768;  // 32.768 LSB/(°/s)
            break;
        case BMI160_GYR_RANGE_500_DPS:
            s_sensor_ctx->gyr_lsb_per_dps_x1000 = 65536;  // 65.536 LSB/(°/s)
            break;
        case BMI160_GYR_RANGE_250_DPS:
            s_sensor_ctx->gyr_lsb_per_dps_x1000 = 131072; // 131.072 LSB/(°/s)
            break;
        case BMI160_GYR_RANGE_125_DPS:
            s_sensor_ctx->gyr_lsb_per_dps_x1000 = 262144; // 262.144 LSB/(°/s)
            break;
        default:
            s_sensor_ctx->gyr_lsb_per_dps_x1000 = 65536; // Default to 500 DPS
            break;
    }
    
    // Create sensor polling task
    BaseType_t task_ret = xTaskCreate(sensor_poll_task, 
                                    "sensor_poll", 
                                    2048, 
                                    s_sensor_ctx, 
                                    4, 
                                    NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor polling task");
        free(s_sensor_ctx);
        s_sensor_ctx = NULL;
        bmi160_destroy(s_sensor_handle);
        s_sensor_handle = NULL;
        return ESP_FAIL;
    }
    
    s_sensor_initialized = true;
    ESP_LOGI(TAG, "Sensor initialized successfully");
    
    return ESP_OK;
}

/**
 * @brief Deinitialize sensor module
 */
esp_err_t sensor_deinit(void)
{
    if (!s_sensor_initialized) {
        ESP_LOGW(TAG, "Sensor not initialized");
        return ESP_OK;
    }
    
    // Note: We can't easily delete the task from here as it's running
    // The task will continue running until the system shuts down
    // In a production system, you might want to add a task handle and delete it here
    
    if (s_sensor_handle != NULL) {
        bmi160_destroy(s_sensor_handle);
        s_sensor_handle = NULL;
    }
    
    if (s_sensor_ctx != NULL) {
        free(s_sensor_ctx);
        s_sensor_ctx = NULL;
    }
    
    s_sensor_initialized = false;
    ESP_LOGI(TAG, "Sensor deinitialized");
    
    return ESP_OK;
}

/**
 * @brief Get current sensor handle
 */
bmi160_handle_t sensor_get_handle(void)
{
    return s_sensor_handle;
}

/**
 * @brief Check if sensor is initialized
 */
bool sensor_is_initialized(void)
{
    return s_sensor_initialized;
}
