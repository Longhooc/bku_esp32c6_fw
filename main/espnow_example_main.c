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
#include "bmi160.h"
#include "driver/spi_master.h"

#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_example";

static QueueHandle_t s_example_espnow_queue = NULL;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

static void example_espnow_deinit(example_espnow_send_param_t *send_param);

typedef struct {
    bmi160_handle_t handle;
    int acc_mg_per_lsb_x1000;      // mg per LSB * 1000 (e.g. 244 for 0.244 mg/LSB)
    int gyr_lsb_per_dps_x1000;     // LSB per dps * 1000 (e.g. 65536 for 65.536 LSB/(°/s))
} bmi_task_ctx_t;

static void bmi160_poll_task(void *pv)
{
    bmi_task_ctx_t *ctx = (bmi_task_ctx_t *)pv;
    bmi160_sample_t sample;
    uint32_t istat;
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

            const char *sx = (ax_ms2_x100 < 0) ? "-" : "";
            const char *sy = (ay_ms2_x100 < 0) ? "-" : "";
            const char *sz = (az_ms2_x100 < 0) ? "-" : "";
            long long ax_whole = (long long)(ax_ms2_x100 < 0 ? -ax_ms2_x100 : ax_ms2_x100) / 100;
            long long ay_whole = (long long)(ay_ms2_x100 < 0 ? -ay_ms2_x100 : ay_ms2_x100) / 100;
            long long az_whole = (long long)(az_ms2_x100 < 0 ? -az_ms2_x100 : az_ms2_x100) / 100;
            int ax_frac = (int)((ax_ms2_x100 < 0 ? -ax_ms2_x100 : ax_ms2_x100) % 100);
            int ay_frac = (int)((ay_ms2_x100 < 0 ? -ay_ms2_x100 : ay_ms2_x100) % 100);
            int az_frac = (int)((az_ms2_x100 < 0 ? -az_ms2_x100 : az_ms2_x100) % 100);

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
        if (bmi160_read_int_status(ctx->handle, &istat) == ESP_OK && istat) {
            ESP_LOGE("bmi160", "int_status=0x%08lx", (unsigned long)istat);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (tx_info == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;
    uint8_t * des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (IS_BROADCAST_ADDR(des_addr)) {
        /* If added a peer with encryption before, the receive packets may be
         * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
         * Users can check the destination address to distinguish it.
         */
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
    } else {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint32_t *magic)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(example_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
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
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case EXAMPLE_ESPNOW_SEND_CB:
            {
                example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

                ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                if (is_broadcast && (send_param->broadcast == false)) {
                    break;
                }

                if (!is_broadcast) {
                    send_param->count--;
                    if (send_param->count == 0) {
                        ESP_LOGI(TAG, "Send done");
                        example_espnow_deinit(send_param);
                        vTaskDelete(NULL);
                    }
                }

                /* Delay a while before sending the next data. */
                if (send_param->delay > 0) {
                    vTaskDelay(send_param->delay/portTICK_PERIOD_MS);
                }

                ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));

                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                example_espnow_data_prepare(send_param);

                /* Send the next data after the previous data is sent. */
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
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
                if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
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
                        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                        free(peer);
                    }

                    /* Indicates that the device has received broadcast ESPNOW data. */
                    if (send_param->state == 0) {
                        send_param->state = 1;
                    }

                    /* If receive broadcast ESPNOW data which indicates that the other device has received
                     * broadcast ESPNOW data and the local magic number is bigger than that in the received
                     * broadcast ESPNOW data, stop sending broadcast ESPNOW data and start sending unicast
                     * ESPNOW data.
                     */
                    if (recv_state == 1) {
                        /* The device which has the bigger magic number sends ESPNOW data, the other one
                         * receives ESPNOW data.
                         */
                        if (send_param->unicast == false && send_param->magic >= recv_magic) {
                    	    ESP_LOGI(TAG, "Start sending unicast data");
                    	    ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(recv_cb->mac_addr));

                    	    /* Start sending unicast ESPNOW data. */
                            memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                            example_espnow_data_prepare(send_param);
                            if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                                ESP_LOGE(TAG, "Send error");
                                example_espnow_deinit(send_param);
                                vTaskDelete(NULL);
                            }
                            else {
                                send_param->broadcast = false;
                                send_param->unicast = true;
                            }
                        }
                    }
                }
                else if (ret == EXAMPLE_ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */
                    send_param->broadcast = false;
                }
                else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
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
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
    ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
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
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
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
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param);

    xTaskCreate(example_espnow_task, "example_espnow_task", 2048, send_param, 4, NULL);

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
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    example_wifi_init();
    example_espnow_init();

    // BMI160 init on ESP32C6 SPI pins per user wiring
    bmi160_spi_bus_config_t bmi_bus = {
        .gpio_cs = 11,
        .gpio_mosi = 10,
        .gpio_miso = 13,
        .gpio_sck = 12,
        .gpio_int1 = 4,
        .gpio_int2 = 18,
        .spi_host = SPI2_HOST,
        .clock_speed_hz = 5 * 1000 * 1000,
    };
    bmi160_handle_t bmi = NULL;
    if (bmi160_create(&bmi_bus, &bmi) == ESP_OK) {
        esp_err_t bmi_init_ret = bmi160_init_default(bmi);
        if (bmi_init_ret == ESP_OK) {
            // Configure accelerometer to 8G range for large acceleration detection
            ESP_ERROR_CHECK(bmi160_config_ranges(bmi, BMI160_ACC_RANGE_8G, BMI160_GYR_RANGE_500_DPS));
            // Set any-motion threshold to 6.00 m/s^2 (ms2_x100 = 600) with duration = 8 samples (~80ms at 100Hz)
            ESP_ERROR_CHECK(bmi160_enable_anymotion_wakeup_ms2(bmi, 600 /* 6.00 m/s^2 */, 8));
            bmi_task_ctx_t *ctx = (bmi_task_ctx_t *)malloc(sizeof(bmi_task_ctx_t));
            if (ctx) {
                ctx->handle = bmi;
                // Set unit conversion scales based on configured ranges: ACC=8G (0.244 mg/LSB), GYR=500 dps (65.536 LSB/(°/s))
                ctx->acc_mg_per_lsb_x1000 = 244;
                ctx->gyr_lsb_per_dps_x1000 = 65536;
                xTaskCreate(bmi160_poll_task, "bmi160_poll", 2048, ctx, 4, NULL);
            } else {
                ESP_LOGE("bmi160", "no mem for task ctx");
            }
        } else {
            ESP_LOGW("bmi160", "init failed (%d). Check wiring: CS=%d MOSI=%d MISO=%d SCK=%d VCC/GND, sensor present?", (int)bmi_init_ret, bmi_bus.gpio_cs, bmi_bus.gpio_mosi, bmi_bus.gpio_miso, bmi_bus.gpio_sck);
        }
    } else {
        ESP_LOGE("bmi160", "create failed");
    }
}
