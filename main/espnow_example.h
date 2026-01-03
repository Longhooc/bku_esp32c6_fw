/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef ESPNOW_EXAMPLE_H
#define ESPNOW_EXAMPLE_H

/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE           6

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

/* ============================================================================
 * Command and FFT Structures for Bi-Directional Communication
 * ============================================================================ */

/* Command types from Gateway → Node */
#define CMD_NORMAL_MODE      0x00
#define CMD_START_FFT        0x01
#define CMD_STOP_FFT         0x02
#define CMD_HEARTBEAT        0x03

/* Command magic number for validation */
#define CMD_MAGIC            0xABCD1234

/* FFT configuration */
#define FFT_SAMPLES_PER_PACKET  32    // 32 float values per ESP-NOW packet
#define FFT_MAX_SAMPLES         2048  // Maximum FFT size (increased for better resolution)
#define FFT_DEFAULT_SAMPLE_RATE 1000  // Default sample rate in Hz

/* Command packet structure (Gateway → Node) */
typedef struct {
    uint8_t command_type;      // CMD_START_FFT, CMD_STOP_FFT, etc.
    uint16_t sample_count;     // Number of samples to collect (e.g., 512)
    uint16_t sample_rate_hz;   // Sampling rate in Hz (e.g., 1000)
    uint32_t magic;            // CMD_MAGIC for validation
    uint8_t reserved[3];       // Reserved for future use, padding to 12 bytes
} __attribute__((packed)) command_packet_t;

/* FFT data packet structure (Node → Gateway) */
typedef struct {
    uint8_t packet_number;     // Current packet number (0, 1, 2, ...)
    uint8_t total_packets;     // Total number of packets in this FFT transmission
    uint16_t samples_in_packet; // Number of valid samples in this packet (up to 32)
    uint32_t timestamp_ms;     // Timestamp when FFT was collected
    float fft_data[FFT_SAMPLES_PER_PACKET]; // FFT magnitude data
} __attribute__((packed)) fft_data_packet_t;

/* FFT ACK packet structure (Gateway → Node) */
typedef struct {
    uint8_t packet_number;     // Which packet we're ACKing
    uint32_t timestamp_ms;     // Timestamp for matching
    uint8_t status;            // 0=OK, 1=Duplicate, 2=Error
    uint8_t received_count;    // How many packets received so far
} __attribute__((packed)) fft_ack_packet_t;

/* ACK status codes */
#define FFT_ACK_OK        0
#define FFT_ACK_DUPLICATE 1
#define FFT_ACK_ERROR     2

/* Retry configuration */
#define FFT_ACK_TIMEOUT_MS 3000
#define MAX_FFT_RETRIES 6
#define MAX_FFT_PACKETS 64    // Increased to support 2048 timestamps (1024 bins / 32 = 32 packets)

typedef enum {
    EXAMPLE_ESPNOW_SEND_CB,
    EXAMPLE_ESPNOW_RECV_CB,
} example_espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} example_espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} example_espnow_event_recv_cb_t;

typedef union {
    example_espnow_event_send_cb_t send_cb;
    example_espnow_event_recv_cb_t recv_cb;
} example_espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    example_espnow_event_id_t id;
    example_espnow_event_info_t info;
} example_espnow_event_t;

enum {
    EXAMPLE_ESPNOW_DATA_BROADCAST,
    EXAMPLE_ESPNOW_DATA_UNICAST,
    EXAMPLE_ESPNOW_DATA_MAX,
};

/* User defined field of ESPNOW data in this example. */
typedef struct {
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     //Sequence number of ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint8_t payload[0];                   //Real payload of ESPNOW data.
} __attribute__((packed)) example_espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    bool unicast;                         //Send unicast ESPNOW data.
    bool broadcast;                       //Send broadcast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                       //Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                       //Delay between sending two ESPNOW data, unit: ms.
    int len;                              //Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      //Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
} example_espnow_send_param_t;

#endif
