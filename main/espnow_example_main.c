/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_example.h"
#include "sensor.h"
#include "battery.h"
#include "esp_adc/adc_oneshot.h"

// Extern sensor handle
extern bmi160_handle_t sensor_get_handle(void);
#include "esp_sleep.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#define ESPNOW_MAXDELAY 512

// Sleep configuration - Use deep sleep for lowest power consumption
#define SLEEP_DURATION_SECONDS 600
#define WAKE_DURATION_SECONDS 20

// Battery voltage reading configuration
// ADC_CHANNEL_0 = GPIO0, ADC_CHANNEL_1 = GPIO1, ADC_CHANNEL_2 = GPIO2, etc.
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_0  // GPIO0 on ESP32-C6
// Voltage divider ratio: Set to 1.0 if no divider, or divider ratio if using voltage divider
// Note: ADC_ATTEN_DB_12 can measure 0-3.9V, so LiFePO4 (max 3.6V) can be measured directly without divider
#define BATTERY_VOLTAGE_DIVIDER_RATIO 1.0f  // No divider needed for 0-3.9V range

// Sleep variables
static esp_timer_handle_t sleep_timer = NULL;
static bool is_sleep_mode = false;
RTC_DATA_ATTR static bool sensor_initialized = false;         // Track sensor initialization state (persists in deep sleep)
static SemaphoreHandle_t s_sensor_mutex = NULL; // Mutex to protect sensor access
static bool s_sensor_task_running = false;      // Track if sensor task is running
static esp_pm_config_t pm_config = {
    .max_freq_mhz = 80,
    .min_freq_mhz = 10,
    .light_sleep_enable = true};

static const char *TAG = "espnow_example";

static QueueHandle_t s_example_espnow_queue = NULL;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = {0, 0};

// Sensor data structure for ESPNOW
typedef struct
{
    int32_t ax_ms2_x100; // Acceleration X in m/s^2 * 100
    int32_t ay_ms2_x100; // Acceleration Y in m/s^2 * 100
    int32_t az_ms2_x100; // Acceleration Z in m/s^2 * 100
    uint32_t battery_voltage_mv; // Battery voltage in millivolts
} sensor_data_t;

static void example_espnow_deinit(example_espnow_send_param_t *send_param);

// Sleep function declarations
static void sleep_device(void);
static void sleep_timer_callback(void *arg);
static void enable_automatic_light_sleep(void);
static void disable_automatic_light_sleep(void);
static esp_err_t create_sleep_timer(void);
static void example_sensor_task_wrapper(void *arg);
static void setup_gpio8_low(void);

/**
 * @brief Setup GPIO8 as output and set it to LOW level
 */
static void setup_gpio8_low(void)
{
    ESP_LOGI(TAG, "Setting up GPIO8 as output and setting to LOW");
    
    // Configure GPIO8 as output
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,    // Disable interrupt
        .mode = GPIO_MODE_OUTPUT,          // Set as output mode
        .pin_bit_mask = (1ULL << 8),       // GPIO8 bit mask
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // Disable pull-down
        .pull_up_en = GPIO_PULLUP_DISABLE,      // Disable pull-up
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO8: %s", esp_err_to_name(ret));
        return;
    }
    
    // Set GPIO8 to LOW level
    ret = gpio_set_level(GPIO_NUM_8, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GPIO8 to LOW: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "GPIO8 configured as output and set to LOW level");
}

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    // Enable Long Range mode for ESP-NOW
    // Note: Both sender and receiver must have the same long range configuration
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
    ESP_LOGI(TAG, "Long Range mode enabled for ESP-NOW");
#endif
}

// Sleep timer callback function
static void sleep_timer_callback(void *arg)
{
    ESP_LOGI(TAG, "Sleep timer expired, entering deep sleep mode");
    sleep_device();
}

// Function to put device into deep sleep
static void sleep_device(void)
{
    ESP_LOGI(TAG, "Preparing to enter deep sleep");

    // Suspend sensor polling task to avoid SPI conflicts
    // sensor_suspend_poll_task();

    // Stop sleep timer
    // if (sleep_timer)
    // {
    //     esp_timer_stop(sleep_timer);
    //     esp_timer_delete(sleep_timer);
    //     sleep_timer = NULL;
    // }

    // Configure wake up sources for deep sleep
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_SECONDS * 1000000ULL);

    // Enable GPIO hold for IO4 to maintain its state (Input, Pull-down) during deep sleep
    // This prevents the pin from floating or changing state which could cause missed interrupts
    gpio_hold_en(GPIO_NUM_4);

    ESP_LOGI(TAG, "Deep sleep configured - wake up in %d seconds", SLEEP_DURATION_SECONDS);

    // Enter deep sleep (lowest power; execution restarts on wake)
    esp_deep_sleep_start();

    // No code after esp_deep_sleep_start() will run
    is_sleep_mode = false;

    // Do NOT resume sensor polling task - we use example_sensor_task_wrapper instead
    // This prevents SPI conflicts between poll task and wrapper task

    // Unreachable in deep sleep

    // Recreate timer for next sleep cycle
    // ESP_LOGI(TAG, "Recreating timer for next sleep cycle");
    // esp_err_t ret = create_sleep_timer();
    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Failed to recreate sleep timer: %s", esp_err_to_name(ret));
    // }
}

// Create sleep timer function
static esp_err_t create_sleep_timer(void)
{
    // Clean up existing timer if any
    if (sleep_timer)
    {
        esp_timer_stop(sleep_timer);
        esp_timer_delete(sleep_timer);
        sleep_timer = NULL;
    }

    const esp_timer_create_args_t sleep_timer_args = {
        .callback = &sleep_timer_callback,
        .name = "sleep_timer"};

    esp_err_t ret = esp_timer_create(&sleep_timer_args, &sleep_timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create sleep timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start timer for wake duration
    ret = esp_timer_start_once(sleep_timer, WAKE_DURATION_SECONDS * 1000000ULL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start sleep timer: %s", esp_err_to_name(ret));
        esp_timer_delete(sleep_timer);
        sleep_timer = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "Sleep timer created and started - will deep sleep in %d seconds", WAKE_DURATION_SECONDS);
    return ESP_OK;
}

// Wrapper function for sensor task that runs on each wake up
static void example_sensor_task_wrapper(void *arg)
{
    // Check if sensor task is already running
    if (s_sensor_task_running)
    {
        ESP_LOGW(TAG, "Sensor task already running, skipping...");
        vTaskDelete(NULL);
        return;
    }

    s_sensor_task_running = true;

    // Get sensor mutex before accessing sensor
    if (s_sensor_mutex != NULL)
    {
        if (xSemaphoreTake(s_sensor_mutex, portMAX_DELAY) != pdTRUE)
        {
            ESP_LOGE(TAG, "Failed to take sensor mutex");
            s_sensor_task_running = false;
            vTaskDelete(NULL);
            return;
        }
    }

    bmi160_handle_t handle = sensor_get_handle();
    if (handle == NULL)
    {
        ESP_LOGE(TAG, "Cannot get sensor handle");
        if (s_sensor_mutex != NULL)
        {
            xSemaphoreGive(s_sensor_mutex);
        }
        s_sensor_task_running = false;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Running sensor task for this wake cycle");
    // Read FIFO data once (non-blocking): whatever is available
    uint8_t fifoBuffer[1024];
    uint16_t fifoCount = 0;

    if (bmi160_get_fifo_count(handle, &fifoCount) == ESP_OK)
    {
        ESP_LOGI(TAG, "FIFO count: %d bytes", fifoCount);
        if (fifoCount >= 6)
        { // At least one frame
            uint16_t bytesToRead = (fifoCount > sizeof(fifoBuffer)) ? sizeof(fifoBuffer) : fifoCount;
            bytesToRead = (bytesToRead / 6) * 6; // Read complete frames only

            if (bmi160_get_fifo_bytes(handle, fifoBuffer, bytesToRead) == ESP_OK)
            {
                ESP_LOGI(TAG, "Read %d bytes from FIFO, parsing data...", bytesToRead);

                // Parse frames and find the one with maximum normal magnitude
                int16_t max_ax = 0, max_ay = 0, max_az = 0;
                uint32_t max_magnitude = 0;
                uint16_t max_frame_idx = 0;

                for (uint16_t i = 0; i <= bytesToRead - 6; i += 6)
                {
                    int16_t ax = (int16_t)((fifoBuffer[i + 1] << 8) | fifoBuffer[i + 0]);
                    int16_t ay = (int16_t)((fifoBuffer[i + 3] << 8) | fifoBuffer[i + 2]);
                    int16_t az = (int16_t)((fifoBuffer[i + 5] << 8) | fifoBuffer[i + 4]);

                    // Calculate magnitude (using squared magnitude to avoid sqrt)
                    uint32_t magnitude_squared = (uint32_t)(ax * ax) + (uint32_t)(ay * ay) + (uint32_t)(az * az);

                    if (magnitude_squared > max_magnitude)
                    {
                        max_magnitude = magnitude_squared;
                        max_ax = ax;
                        max_ay = ay;
                        max_az = az;
                        max_frame_idx = i / 6;
                    }
                }

                // Read actual accelerometer range from sensor to calculate correct conversion factor
                // Read register directly (same method as bmi160_enable_anymotion_wakeup_ms2)
                uint8_t acc_range_reg = 0;
                int acc_mg_per_lsb_x1000 = 244; // Default to 8G (0.244 mg/LSB)
                
                if (bmi160_get_register(handle, BMI160_REG_ACC_RANGE, &acc_range_reg) == ESP_OK) {
                    // Calculate conversion factor based on actual range
                    // Per BMI160 datasheet: 2G=0.061, 4G=0.122, 8G=0.244, 16G=0.488 mg/LSB
                    // Mask with 0x0F to get only range bits (same as bmi160_acc_lsb_mg_for_range)
                    switch (acc_range_reg & 0x0F) {
                        case BMI160_ACC_RANGE_2G:
                            acc_mg_per_lsb_x1000 = 61;   // 0.061 mg/LSB
                            break;
                        case BMI160_ACC_RANGE_4G:
                            acc_mg_per_lsb_x1000 = 122;  // 0.122 mg/LSB
                            break;
                        case BMI160_ACC_RANGE_8G:
                            acc_mg_per_lsb_x1000 = 244;  // 0.244 mg/LSB
                            break;
                        case BMI160_ACC_RANGE_16G:
                            acc_mg_per_lsb_x1000 = 488;  // 0.488 mg/LSB
                            break;
                        default:
                            ESP_LOGW(TAG, "Unknown acc range register 0x%02X, using default 8G", acc_range_reg);
                            acc_mg_per_lsb_x1000 = 244;
                            break;
                    }
                    ESP_LOGD(TAG, "Accelerometer range register: 0x%02X, conversion factor: %d (mg/LSB * 1000)", 
                             acc_range_reg, acc_mg_per_lsb_x1000);
                } else {
                    ESP_LOGW(TAG, "Failed to read accelerometer range register, using default 8G conversion");
                }

                // Convert max sample to m/s^2 (fixed-point, *100)
                // Convert raw LSB to mg * 100: (raw * mg_per_lsb_x1000) / 1000 = mg, then * 100 = mg * 100
                int32_t ax_mg_x100 = ((int32_t)max_ax * (int32_t)acc_mg_per_lsb_x1000 + (max_ax >= 0 ? 500 : -500)) / 1000;
                int32_t ay_mg_x100 = ((int32_t)max_ay * (int32_t)acc_mg_per_lsb_x1000 + (max_ay >= 0 ? 500 : -500)) / 1000;
                int32_t az_mg_x100 = ((int32_t)max_az * (int32_t)acc_mg_per_lsb_x1000 + (max_az >= 0 ? 500 : -500)) / 1000;

                // Convert mg to m/s^2: 1 mg = 9.80665e-3 m/s^2
                // mg * 100 -> m/s^2 * 100: (mg_x100 * 980665) / 1000000
                int32_t ax_ms2_x100 = (int32_t)((int64_t)ax_mg_x100 * 980665LL + (ax_mg_x100 >= 0 ? 500000 : -500000)) / 1000000LL;
                int32_t ay_ms2_x100 = (int32_t)((int64_t)ay_mg_x100 * 980665LL + (ay_mg_x100 >= 0 ? 500000 : -500000)) / 1000000LL;
                int32_t az_ms2_x100 = (int32_t)((int64_t)az_mg_x100 * 980665LL + (az_mg_x100 >= 0 ? 500000 : -500000)) / 1000000LL;

                // Read battery voltage
                uint32_t battery_voltage_mv = 0;
                if (battery_voltage_is_initialized())
                {
                    if (battery_voltage_read_mv(&battery_voltage_mv) == ESP_OK)
                    {
                        float battery_voltage_v = (float)battery_voltage_mv / 1000.0f;
                        ESP_LOGI(TAG, "Battery voltage: %lu mV (%.3f V)", battery_voltage_mv, battery_voltage_v);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Failed to read battery voltage, using 0");
                        battery_voltage_mv = 0;
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "Battery voltage module not initialized, using 0");
                    battery_voltage_mv = 0;
                }

                ESP_LOGI(TAG, "Max frame[%d] m/s^2: ax=%.3f, ay=%.3f, az=%.3f, Battery: %lu mV (%.3f V)",
                         max_frame_idx,
                         (float)ax_ms2_x100 / 100.0f,
                         (float)ay_ms2_x100 / 100.0f,
                         (float)az_ms2_x100 / 100.0f,
                         battery_voltage_mv,
                         (float)battery_voltage_mv / 1000.0f);

                // Send sensor data via ESPNOW
                sensor_data_t sensor_data = {
                    .ax_ms2_x100 = ax_ms2_x100,
                    .ay_ms2_x100 = ay_ms2_x100,
                    .az_ms2_x100 = az_ms2_x100,
                    .battery_voltage_mv = battery_voltage_mv};

                uint8_t send_buffer[sizeof(example_espnow_data_t) + sizeof(sensor_data_t)];
                example_espnow_data_t *espnow_data = (example_espnow_data_t *)send_buffer;

                espnow_data->type = EXAMPLE_ESPNOW_DATA_BROADCAST;
                espnow_data->state = 0;
                espnow_data->seq_num = s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_BROADCAST]++;
                espnow_data->magic = 0x12345678;
                espnow_data->crc = 0;
                memcpy(espnow_data->payload, &sensor_data, sizeof(sensor_data_t));
                espnow_data->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)espnow_data,
                                                sizeof(example_espnow_data_t) + sizeof(sensor_data_t));

                esp_err_t ret = esp_now_send(s_example_broadcast_mac, send_buffer, sizeof(send_buffer));
                if (ret != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed to send sensor data via ESPNOW: %s", esp_err_to_name(ret));
                }
                else
                {
                    ESP_LOGI(TAG, "Sent ESPNOW: accel(%.3f, %.3f, %.3f) m/s^2, battery: %lu mV (%.3f V)",
                             (float)ax_ms2_x100 / 100.0f,
                             (float)ay_ms2_x100 / 100.0f,
                             (float)az_ms2_x100 / 100.0f,
                             battery_voltage_mv,
                             (float)battery_voltage_mv / 1000.0f);
                }
            }

            // Flush FIFO for next accumulation window
            bmi160_reset_fifo(handle);
        }
        else
        {
            ESP_LOGI(TAG, "FIFO not ready (count < 6), skipping this cycle");
        }
    }

    ESP_LOGI(TAG, "Sensor task completed for this wake cycle");

    // Release mutex before deleting task
    if (s_sensor_mutex != NULL)
    {
        xSemaphoreGive(s_sensor_mutex);
    }

    s_sensor_task_running = false;

    // Enter deep sleep immediately after finishing this cycle
    sleep_device();
}

// Enable automatic light sleep for power optimization
static void enable_automatic_light_sleep(void)
{
    ESP_LOGI(TAG, "Enabling automatic light sleep for power optimization");
    
    // Configure power management
    esp_err_t ret = esp_pm_configure(&pm_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure power management: %s", esp_err_to_name(ret));
        return;
    }
    
    // Enable automatic light sleep
    esp_pm_config_t pm_config_light = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 10,
        .light_sleep_enable = true
    };
    
    ret = esp_pm_configure(&pm_config_light);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable light sleep: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Automatic light sleep enabled successfully");
    }
}

// Disable automatic light sleep
static void disable_automatic_light_sleep(void)
{
    ESP_LOGI(TAG, "Disabling automatic light sleep");

    esp_pm_config_t pm_config_no_sleep = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 10,
        .light_sleep_enable = false};

    esp_err_t ret = esp_pm_configure(&pm_config_no_sleep);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to disable light sleep: %s", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Automatic light sleep disabled");
    }
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (tx_info == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t *mac_addr = recv_info->src_addr;
    uint8_t *des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (IS_BROADCAST_ADDR(des_addr))
    {
        /* If added a peer with encryption before, the receive packets may be
         * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
         * Users can check the destination address to distinguish it.
         */
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
    }
    else
    {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL)
    {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint32_t *magic)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(example_espnow_data_t))
    {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc)
    {
        return buf->type;
    }

    return -1;
}

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    /* Fill all remaining bytes after the data with random values */
    esp_fill_random(buf->payload, send_param->len - sizeof(example_espnow_data_t));
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static void example_espnow_task(void *pvParameter)
{
    example_espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint32_t recv_magic = 0;
    bool is_broadcast = false;
    int ret;

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
    {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        switch (evt.id)
        {
        case EXAMPLE_ESPNOW_SEND_CB:
        {
            example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
            is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

            ESP_LOGD(TAG, "Send data to " MACSTR ", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

            if (is_broadcast && (send_param->broadcast == false))
            {
                break;
            }

            if (!is_broadcast)
            {
                send_param->count--;
                if (send_param->count == 0)
                {
                    ESP_LOGI(TAG, "Send done");
                    example_espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
            }

            /* Delay a while before sending the next data. */
            if (send_param->delay > 0)
            {
                vTaskDelay(send_param->delay / portTICK_PERIOD_MS);
            }

            ESP_LOGI(TAG, "send data to " MACSTR "", MAC2STR(send_cb->mac_addr));

            memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
            example_espnow_data_prepare(send_param);

            /* Send the next data after the previous data is sent. */
            if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
            {
                ESP_LOGE(TAG, "Send error");
                example_espnow_deinit(send_param);
                vTaskDelete(NULL);
            }
            break;
        }
        case EXAMPLE_ESPNOW_RECV_CB:
        {
            example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

            ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
            free(recv_cb->data);
            if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST)
            {
                ESP_LOGI(TAG, "Receive %dth broadcast data from: " MACSTR ", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                /* If MAC address does not exist in peer list, add it to peer list. */
                if (esp_now_is_peer_exist(recv_cb->mac_addr) == false)
                {
                    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                    if (peer == NULL)
                    {
                        ESP_LOGE(TAG, "Malloc peer information fail");
                        example_espnow_deinit(send_param);
                        vTaskDelete(NULL);
                    }
                    memset(peer, 0, sizeof(esp_now_peer_info_t));
                    peer->channel = CONFIG_ESPNOW_CHANNEL;
                    peer->ifidx = ESPNOW_WIFI_IF;
                    peer->encrypt = true;
                    memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                    memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                    ESP_ERROR_CHECK(esp_now_add_peer(peer));
                    free(peer);
                }

                /* Indicates that the device has received broadcast ESPNOW data. */
                if (send_param->state == 0)
                {
                    send_param->state = 1;
                }

                /* If receive broadcast ESPNOW data which indicates that the other device has received
                 * broadcast ESPNOW data and the local magic number is bigger than that in the received
                 * broadcast ESPNOW data, stop sending broadcast ESPNOW data and start sending unicast
                 * ESPNOW data.
                 */
                if (recv_state == 1)
                {
                    /* The device which has the bigger magic number sends ESPNOW data, the other one
                     * receives ESPNOW data.
                     */
                    if (send_param->unicast == false && send_param->magic >= recv_magic)
                    {
                        ESP_LOGI(TAG, "Start sending unicast data");
                        ESP_LOGI(TAG, "send data to " MACSTR "", MAC2STR(recv_cb->mac_addr));

                        /* Start sending unicast ESPNOW data. */
                        memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        example_espnow_data_prepare(send_param);
                        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Send error");
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        else
                        {
                            send_param->broadcast = false;
                            send_param->unicast = true;
                        }
                    }
                }
            }
            else if (ret == EXAMPLE_ESPNOW_DATA_UNICAST)
            {
                ESP_LOGI(TAG, "Receive %dth unicast data from: " MACSTR ", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */
                send_param->broadcast = false;
            }
            else
            {
                ESP_LOGI(TAG, "Receive error data from: " MACSTR "", MAC2STR(recv_cb->mac_addr));
            }
            break;
        }
        default:
            ESP_LOGE(TAG, "Callback type error: %d", evt.id);
            break;
        }
    }
}

static esp_err_t example_espnow_init(void)
{
    example_espnow_send_param_t *send_param;

    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL)
    {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(example_espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(example_espnow_recv_cb));
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK(esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW));
    ESP_ERROR_CHECK(esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL));
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL)
    {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL)
    {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param);

    // xTaskCreate(example_espnow_task, "example_espnow_task", 2048, send_param, 4, NULL);

    return ESP_OK;
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vQueueDelete(s_example_espnow_queue);
    s_example_espnow_queue = NULL;
    esp_now_deinit();
}
static void configure_io4_wakeup(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << GPIO_NUM_4,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    esp_err_t err = esp_sleep_enable_ext1_wakeup(1ULL << GPIO_NUM_4, ESP_EXT1_WAKEUP_ANY_HIGH);
    
    if (err != ESP_OK) {
        printf("Lỗi cấu hình wakeup: %s\n", esp_err_to_name(err));
    }
}

void app_main(void)
{
    // Disable GPIO hold for IO4 if it was enabled during deep sleep
    // This allows reconfiguring the pin if needed
    gpio_hold_dis(GPIO_NUM_4);

    // Create sensor mutex
    s_sensor_mutex = xSemaphoreCreateMutex();
    if (s_sensor_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create sensor mutex");
        return;
    }
    //io4 interup
    configure_io4_wakeup();
    // Cách thay thế gọn hơn trên C6/S3
// esp_deep_sleep_enable_gpio_wakeup(1ULL << GPIO_NUM_4, ESP_GPIO_WAKEUP_GPIO_HIGH);

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
        ESP_LOGI(TAG, "Wake by IO4 falling edge");
    }
    // Check if we woke up from light sleep
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER || wakeup_reason == ESP_SLEEP_WAKEUP_EXT1)
    {
        ESP_LOGI(TAG, "Woke up from deep sleep (Reason: %s)", 
                 wakeup_reason == ESP_SLEEP_WAKEUP_TIMER ? "Timer" : "IO4/EXT1");
        is_sleep_mode = false;
    }
    else
    {
        ESP_LOGI(TAG, "Starting fresh boot - need to initialize sensor");
        is_sleep_mode = false;
        sensor_initialized = false; // Mark sensor as not initialized
    }

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Setup GPIO8 as output and set to LOW
    setup_gpio8_low();

    // Initialize battery voltage reading module
    // Using GPIO0 (ADC_CHANNEL_0) for battery voltage reading
    // ADC_ATTEN_DB_12 can measure 0-3.9V, so LiFePO4 (max 3.6V) can be measured directly
    ESP_LOGI(TAG, "Initializing battery voltage reading module - GPIO0 (ADC_CHANNEL_0)");
    ESP_LOGI(TAG, "ADC range: 0-3.9V (ADC_ATTEN_DB_12), Voltage divider ratio: %.3f", 
             BATTERY_VOLTAGE_DIVIDER_RATIO);
    esp_err_t battery_ret = battery_voltage_init(BATTERY_ADC_CHANNEL, BATTERY_VOLTAGE_DIVIDER_RATIO);
    if (battery_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize battery voltage module: %s", esp_err_to_name(battery_ret));
    }
    else
    {
        ESP_LOGI(TAG, "Battery voltage module initialized successfully");
        
        // Read and log battery voltage
        uint32_t voltage_mv = 0;
        float voltage_v = 0.0f;
        if (battery_voltage_read_mv(&voltage_mv) == ESP_OK)
        {
            battery_voltage_read_v(&voltage_v);
            ESP_LOGI(TAG, "Battery voltage: %lu mV (%.3f V)", voltage_mv, voltage_v);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read battery voltage");
        }
    }

    example_wifi_init();
    example_espnow_init();

    enable_automatic_light_sleep();
    // Initialize sensor
    if (!sensor_initialized)
    {
        ESP_LOGI(TAG, "Initializing sensor for the first time");
        esp_err_t sensor_ret = sensor_init();
        if (sensor_ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize sensor: %s", esp_err_to_name(sensor_ret));
        }
        else
        {
            ESP_LOGI(TAG, "Sensor initialized successfully");
            sensor_initialized = true;

            // Suspend sensor polling task immediately - we use example_sensor_task_wrapper instead
            // This prevents SPI conflicts
            sensor_suspend_poll_task();
            ESP_LOGI(TAG, "Sensor polling task suspended to prevent SPI conflicts");

            // Create initial sensor reading task
            ESP_LOGI(TAG, "Creating initial sensor task");
            // xTaskCreate(example_sensor_task_wrapper, "sensor_init", 20048, NULL, 4, NULL);
            example_sensor_task_wrapper(NULL);
        }
    }
    else
    {
        ESP_LOGI(TAG, "Woke from deep sleep - sensor already initialized");
        
        // Resume sensor (re-init SPI bus without resetting sensor)
        esp_err_t sensor_ret = sensor_resume();
        if (sensor_ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to resume sensor: %s", esp_err_to_name(sensor_ret));
            // If resume fails, try full init
            sensor_initialized = false;
            sensor_init();
        }
        else
        {
            ESP_LOGI(TAG, "Sensor resumed successfully");
            
            // Suspend polling task as usual
            sensor_suspend_poll_task();
            
            // Run the wrapper task to read FIFO
            example_sensor_task_wrapper(NULL);
        }
    }

    // Enable automatic light sleep for power optimization
   

    // Create timer for sleep cycle - wake for WAKE_DURATION_SECONDS then sleep for SLEEP_DURATION_SECONDS
    ESP_LOGI(TAG, "Starting wake window - will deep sleep in %d seconds", WAKE_DURATION_SECONDS);

    // ret = create_sleep_timer();
    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Failed to create initial sleep timer: %s", esp_err_to_name(ret));
    // }
    // else
    // {
    //     ESP_LOGI(TAG, "Sleep timer started - device will deep sleep in %d seconds", WAKE_DURATION_SECONDS);
    //     ESP_LOGI(TAG, "Deep sleep mode is active for power optimization");
    // }

    // Main loop - keep the system running
    ESP_LOGI(TAG, "Entering main loop - system will continue sleep/wake cycles");
    while (1)
    {
        // Read and log battery voltage periodically (every 10 seconds)
        static uint32_t battery_read_counter = 0;
        if (battery_voltage_is_initialized() && (battery_read_counter % 10 == 0))
        {
            uint32_t voltage_mv = 0;
            float voltage_v = 0.0f;
            if (battery_voltage_read_mv(&voltage_mv) == ESP_OK)
            {
                battery_voltage_read_v(&voltage_v);
                ESP_LOGI(TAG, "Battery voltage: %lu mV (%.3f V)", voltage_mv, voltage_v);
            }
        }
        battery_read_counter++;
        
        // Let the system handle sleep/wake cycles automatically
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
}
