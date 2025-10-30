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

// Extern sensor handle
extern bmi160_handle_t sensor_get_handle(void);
#include "esp_sleep.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#define ESPNOW_MAXDELAY 512

// Sleep configuration - Use light sleep instead of deep sleep for better power efficiency
#define SLEEP_DURATION_SECONDS 20
#define WAKE_DURATION_SECONDS 20

// Sleep variables
static esp_timer_handle_t sleep_timer = NULL;
static bool is_sleep_mode = false;
static bool sensor_initialized = false;         // Track sensor initialization state
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
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
}

// Sleep timer callback function
static void sleep_timer_callback(void *arg)
{
    ESP_LOGI(TAG, "Sleep timer expired, entering light sleep mode");
    // sleep_device();
}

// Function to put device into light sleep (more power efficient than deep sleep)
static void sleep_device(void)
{
    ESP_LOGI(TAG, "Preparing to enter light sleep");

    // Suspend sensor polling task to avoid SPI conflicts
    // sensor_suspend_poll_task();

    // Stop sleep timer
    // if (sleep_timer)
    // {
    //     esp_timer_stop(sleep_timer);
    //     esp_timer_delete(sleep_timer);
    //     sleep_timer = NULL;
    // }

    // Configure wake up sources for light sleep
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_SECONDS * 1000000ULL);

    ESP_LOGI(TAG, "Light sleep configured - wake up in %d seconds", SLEEP_DURATION_SECONDS);

    // Enter light sleep (keeps RAM and CPU state, only ~0.8mA vs ~10μA deep sleep)
    esp_light_sleep_start();

    // After wake up, continue execution here
    ESP_LOGI(TAG, "Woke up from light sleep");
    esp_restart();
    is_sleep_mode = false;

    // Do NOT resume sensor polling task - we use example_sensor_task_wrapper instead
    // This prevents SPI conflicts between poll task and wrapper task

    // Run sensor task once after wake up
    if (sensor_initialized)
    {
        ESP_LOGI(TAG, "Creating sensor task after wake up");
        // xTaskCreate(example_sensor_task_wrapper, "sensor_wake", 20048, NULL, 4, NULL);
    }

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

    ESP_LOGI(TAG, "Sleep timer created and started - will light sleep in %d seconds", WAKE_DURATION_SECONDS);
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
    vTaskDelay(pdMS_TO_TICKS(2000));
    // Read FIFO data using Arduino-style function
    uint8_t fifoBuffer[1024];
    uint16_t fifoCount = 0;
    while (1)
    {
        if (bmi160_get_fifo_count(handle, &fifoCount) == ESP_OK)
        {
            ESP_LOGI(TAG, "FIFO count: %d bytes", fifoCount);
            vTaskDelay(pdMS_TO_TICKS(2000));
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

                        ESP_LOGI(TAG, "Frame[%d]: ax=%d, ay=%d, az=%d, mag^2=%lu", i / 6, ax, ay, az, (unsigned long)magnitude_squared);

                        // Update max if this frame has larger magnitude
                        if (magnitude_squared > max_magnitude)
                        {
                            max_magnitude = magnitude_squared;
                            max_ax = ax;
                            max_ay = ay;
                            max_az = az;
                            max_frame_idx = i / 6;
                        }
                    }

                    // Convert max sample to m/s^2
                    // For BMI160_ACC_RANGE_8G: acc_mg_per_lsb = 0.244 mg/LSB
                    // Convert LSB -> mg -> m/s^2
                    // mg = lsb * 0.244
                    // m/s^2 = mg * 9.80665 / 1000
                    // Combined: m/s^2 = lsb * 0.244 * 9.80665 / 1000 = lsb * 0.0023928226
                    // Using fixed-point arithmetic with *100 to preserve precision
                    int32_t ax_mg_x100 = (int32_t)max_ax * 244 / 1000; // mg * 100
                    int32_t ay_mg_x100 = (int32_t)max_ay * 244 / 1000; // mg * 100
                    int32_t az_mg_x100 = (int32_t)max_az * 244 / 1000; // mg * 100

                    // Convert mg to m/s^2 (using high precision)
                    // m/s^2 = mg * 9.80665 / 1000 = mg / 101.972
                    // With mg_x100: m/s^2_x100 = mg_x100 * 9.80665 / 1000 = mg_x100 / 102
                    int32_t ax_ms2_x100 = (int32_t)(ax_mg_x100 * 980665LL / 1000000LL);
                    int32_t ay_ms2_x100 = (int32_t)(ay_mg_x100 * 980665LL / 1000000LL);
                    int32_t az_ms2_x100 = (int32_t)(az_mg_x100 * 980665LL / 1000000LL);

                    // Display selected frame
                    ESP_LOGI(TAG, "Selected max frame[%d]: ax=%d, ay=%d, az=%d (LSB)", max_frame_idx, max_ax, max_ay, max_az);
                    ESP_LOGI(TAG, "In m/s^2: ax=%.3f, ay=%.3f, az=%.3f",
                             (float)ax_ms2_x100 / 100.0f,
                             (float)ay_ms2_x100 / 100.0f,
                             (float)az_ms2_x100 / 100.0f);

                    // Send sensor data via ESPNOW
                    sensor_data_t sensor_data = {
                        .ax_ms2_x100 = ax_ms2_x100,
                        .ay_ms2_x100 = ay_ms2_x100,
                        .az_ms2_x100 = az_ms2_x100};

                    // Create ESPNOW data packet with sensor data
                    uint8_t send_buffer[sizeof(example_espnow_data_t) + sizeof(sensor_data_t)];
                    example_espnow_data_t *espnow_data = (example_espnow_data_t *)send_buffer;

                    // Fill ESPNOW header
                    espnow_data->type = EXAMPLE_ESPNOW_DATA_BROADCAST;
                    espnow_data->state = 0;
                    espnow_data->seq_num = s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_BROADCAST]++;
                    espnow_data->magic = 0x12345678; // Magic number for sensor data
                    espnow_data->crc = 0;            // Will be calculated later

                    // Copy sensor data to payload
                    memcpy(espnow_data->payload, &sensor_data, sizeof(sensor_data_t));

                    // Calculate CRC
                    espnow_data->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)espnow_data,
                                                    sizeof(example_espnow_data_t) + sizeof(sensor_data_t));

                    // Send via ESPNOW to broadcast address
                    esp_err_t ret = esp_now_send(s_example_broadcast_mac, send_buffer, sizeof(send_buffer));
                    if (ret == ESP_OK)
                    {
                        ESP_LOGI(TAG, "Sensor data sent via ESPNOW: ax=%.3f, ay=%.3f, az=%.3f m/s^2",
                                 (float)ax_ms2_x100 / 100.0f,
                                 (float)ay_ms2_x100 / 100.0f,
                                 (float)az_ms2_x100 / 100.0f);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to send sensor data via ESPNOW: %s", esp_err_to_name(ret));
                    }
                }

                // Flush FIFO
                bmi160_reset_fifo(handle);
            }
        }

        ESP_LOGE(TAG, "Sensor task completed for this wake cycle");

        // Release mutex before deleting task
        if (s_sensor_mutex != NULL)
        {
            xSemaphoreGive(s_sensor_mutex);
        }

        s_sensor_task_running = false;

        // Task must not return in FreeRTOS - delete itself after completing work
        ESP_LOGI(TAG, "Deleting sensor task...");

        // Enter light sleep immediately after finishing this cycle
        sleep_device();
    }
    // vTaskDelete(NULL); // Delete current task
}

// Enable automatic light sleep for power optimization
static void enable_automatic_light_sleep(void)
{
    ESP_LOGI(TAG, "Enabling automatic light sleep for power optimization");

    esp_err_t ret = esp_pm_configure(&pm_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure power management: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Automatic light sleep enabled successfully");
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

void app_main(void)
{
    // Create sensor mutex
    s_sensor_mutex = xSemaphoreCreateMutex();
    if (s_sensor_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create sensor mutex");
        return;
    }

    // Check if we woke up from light sleep
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER)
    {
        ESP_LOGI(TAG, "Woke up from light sleep by timer - sensor already initialized");
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

    example_wifi_init();
    example_espnow_init();

    // Initialize sensor on first boot only
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
            xTaskCreate(example_sensor_task_wrapper, "sensor_init", 20048, NULL, 4, NULL);
        }
    }
    else
    {
        ESP_LOGI(TAG, "Woke from light sleep - sensor already initialized");
        // Do not create task here - sleep_device() will create it after wake up
        // This prevents duplicate task creation
    }

    // Enable automatic light sleep for power optimization
    enable_automatic_light_sleep();

    // Create timer for sleep cycle - wake for WAKE_DURATION_SECONDS then sleep for SLEEP_DURATION_SECONDS
    ESP_LOGI(TAG, "Starting sleep/wake cycle - will light sleep in %d seconds", WAKE_DURATION_SECONDS);

    // ret = create_sleep_timer();
    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Failed to create initial sleep timer: %s", esp_err_to_name(ret));
    // }
    // else
    // {
    //     ESP_LOGI(TAG, "Sleep timer started - device will light sleep in %d seconds", WAKE_DURATION_SECONDS);
    //     ESP_LOGI(TAG, "Automatic light sleep is active for power optimization");
    // }

    // Main loop - keep the system running
    ESP_LOGI(TAG, "Entering main loop - system will continue sleep/wake cycles");
    while (1)
    {
        // Let the system handle sleep/wake cycles automatically
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
}
