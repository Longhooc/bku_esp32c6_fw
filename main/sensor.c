/* Sensor Module Implementation
 * 
 * This module handles BMI160 sensor initialization, configuration, and data polling.
 * It provides a clean interface for sensor operations and encapsulates all BMI160-related code.
 */

#include "sensor.h"
#include "bmi160.h"
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
static TaskHandle_t s_sensor_poll_task_handle = NULL;

/**
 * @brief Arduino-style FIFO reading function - Wait for FIFO full
 * 
 * This function waits for FIFO buffer full interrupt before reading:
 * 1. Wait for FIFO buffer full interrupt
 * 2. Read FIFO data with bmi160_get_fifo_bytes() (equivalent to bmi.getFIFOBytes())
 * 3. Parse data in 6-byte frames (accelerometer only)
 */
static void sensor_demo_headerless_fifo(bmi160_handle_t handle)
{
    // Buffer for FIFO data (equivalent to Arduino fifoBuffer)
    uint8_t fifoBuffer[1024];
    uint16_t fifoCount = 0;
    uint32_t intStatus = 0;
    
    ESP_LOGI(TAG, "=== Arduino Style FIFO Reading (Wait for Full) ===");
    
    // 1. Wait for FIFO buffer full interrupt (with timeout)
    ESP_LOGI(TAG, "Waiting for FIFO buffer full interrupt...");
    // TickType_t startTime = xTaskGetTickCount();
    // TickType_t timeout = pdMS_TO_TICKS(5000); // 5 second timeout
    
    // bool fifoFullDetected = false;
    // while ((xTaskGetTickCount() - startTime) < timeout) {
    //     esp_err_t err = bmi160_read_int_status(handle, &intStatus);
    //     if (err == ESP_OK && (intStatus & 0x2000)) { // FIFO buffer full bit (bit 13)
    //         fifoFullDetected = true;
    //         ESP_LOGI(TAG, "FIFO buffer full interrupt detected! (0x%08lx)", (unsigned long)intStatus);
    //         break;
    //     }
    //     vTaskDelay(pdMS_TO_TICKS(10)); // Check every 10ms
    // }
    
    // if (!fifoFullDetected) {
    //     ESP_LOGW(TAG, "FIFO buffer full timeout, reading current FIFO count");
    // }
    
    // 2. Get FIFO count (equivalent to Arduino: fifoCount = bmi.getFIFOCount())
    esp_err_t err = bmi160_get_fifo_count(handle, &fifoCount);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get FIFO count: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "FIFO count: %d bytes", fifoCount);
    
    // 3. Check if we have enough data for at least one frame (6 bytes)
    if (fifoCount >= BMI160_ACCEL_FRAME_SIZE) {
        
        // Limit read size to buffer size
        if (fifoCount > sizeof(fifoBuffer)) {
            fifoCount = sizeof(fifoBuffer);
        }
        
        // 4. Read FIFO data (equivalent to Arduino: bmi.getFIFOBytes(fifoBuffer, fifoCount))
        err = bmi160_get_fifo_bytes(handle, fifoBuffer, fifoCount);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read FIFO bytes: %s", esp_err_to_name(err));
            return;
        }
        
        ESP_LOGI(TAG, "Read %d bytes from FIFO", fifoCount);
        
        // 5. Parse data in the buffer (equivalent to Arduino parsing loop)
        // Loop through buffer, jumping 6 bytes each time (1 frame)
        for (uint16_t i = 0; i <= fifoCount - BMI160_ACCEL_FRAME_SIZE; i += BMI160_ACCEL_FRAME_SIZE) {
            
            // Get 6 bytes for 1 data frame
            uint8_t ax_lsb = fifoBuffer[i + 0];
            uint8_t ax_msb = fifoBuffer[i + 1];
            uint8_t ay_lsb = fifoBuffer[i + 2];
            uint8_t ay_msb = fifoBuffer[i + 3];
            uint8_t az_lsb = fifoBuffer[i + 4];
            uint8_t az_msb = fifoBuffer[i + 5];
            
            // ===== CREATE VARIABLES AND READ DATA =====
            // Combine 2 8-bit bytes (LSB and MSB) into 1 16-bit variable
            int16_t ax = (int16_t)((ax_msb << 8) | ax_lsb);
            int16_t ay = (int16_t)((ay_msb << 8) | ay_lsb);
            int16_t az = (int16_t)((az_msb << 8) | az_lsb);
            // =========================================
            
            // 6. Print read data
            ESP_LOGI(TAG, "Frame[%d]: ax=%d, ay=%d, az=%d", 
                     i / BMI160_ACCEL_FRAME_SIZE, ax, ay, az);
        }
        
        ESP_LOGI(TAG, "Parsed %d accelerometer frames from FIFO", fifoCount / BMI160_ACCEL_FRAME_SIZE);
        
        // 7. Flush FIFO after reading to prevent overflow
        bmi160_reset_fifo(handle);
        ESP_LOGI(TAG, "FIFO flushed after reading");
        
    } else {
        ESP_LOGI(TAG, "Not enough data in FIFO (need at least %d bytes)", BMI160_ACCEL_FRAME_SIZE);
    }
}

/**
 * @brief BMI160 polling task - Run once per wake up
 * 
 * This task reads sensor data once and then exits.
 * It will be called again when device wakes up next time.
 * 
 * @param pv Task parameter (sensor_task_ctx_t*)
 */
static void sensor_poll_task(void *pv)
{
    sensor_task_ctx_t *ctx = (sensor_task_ctx_t *)pv;
    bmi160_sample_t sample;
    uint32_t istat;
    uint16_t fifo_count = 0;
    uint8_t fifo_data[1024]; // Buffer for FIFO data
    
    ESP_LOGI(TAG, "Sensor polling task started - single read mode");
    
    for (;;) {
        // Demo headerless FIFO reading (Arduino style)
        // sensor_demo_headerless_fifo(ctx->handle);
        
        // Longer delay to allow FIFO to fill up completely
        // vTaskDelay(pdMS_TO_TICKS(2000)); // 2 seconds delay
        
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
            
            // Check if FIFO buffer full interrupt is set (bit 13 = 0x2000)
            if (istat & 0x2000) {
                ESP_LOGW(TAG, "FIFO buffer full, flushing FIFO");
                bmi160_reset_fifo(ctx->handle);
            }
        }
        
        // Main task delay (in addition to the 100ms delay above)
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
    
    // Configure for headerless FIFO mode (chỉ bật accelerometer, tắt gyroscope như yêu cầu)
    ret = bmi160_init_headerless_mode(s_sensor_handle, true, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure headerless FIFO mode: %s", esp_err_to_name(ret));
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
                                    20048, 
                                    s_sensor_ctx, 
                                    4, 
                                    &s_sensor_poll_task_handle);
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

/**
 * @brief Suspend sensor polling task
 */
esp_err_t sensor_suspend_poll_task(void)
{
    if (!s_sensor_initialized) {
        ESP_LOGW(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_sensor_poll_task_handle != NULL) {
        vTaskSuspend(s_sensor_poll_task_handle);
        ESP_LOGI(TAG, "Sensor polling task suspended");
        return ESP_OK;
    }
    
    ESP_LOGW(TAG, "Sensor polling task handle not found");
    return ESP_ERR_NOT_FOUND;
}

/**
 * @brief Resume sensor polling task
 */
esp_err_t sensor_resume_poll_task(void)
{
    if (!s_sensor_initialized) {
        ESP_LOGW(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_sensor_poll_task_handle != NULL) {
        vTaskResume(s_sensor_poll_task_handle);
        ESP_LOGI(TAG, "Sensor polling task resumed");
        return ESP_OK;
    }
    
    ESP_LOGW(TAG, "Sensor polling task handle not found");
    return ESP_ERR_NOT_FOUND;
}
