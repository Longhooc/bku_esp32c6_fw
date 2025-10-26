#include "bmi160.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

typedef struct bmi160_handle_s {
    spi_device_handle_t spi;
    int gpio_cs;
    int gpio_int1;
    int gpio_int2;
} bmi160_handle_int_t;

static const char *TAG_BMI160 = "bmi160";

static inline uint8_t bmi160_spi_make_addr(uint8_t reg, bool is_read)
{
    // BMI160 SPI: bit7 = 1 for read, 0 for write
    return (is_read ? (reg | 0x80) : (reg & 0x7F));
}

static bool bmi160_is_acc_normal(uint8_t pmu)
{
    // PMU_STATUS: ACC pmu bits [5:4], 0x00=suspend, 0x01=low power, 0x03=normal
    return ((pmu >> 4) & 0x03) == 0x03;
}

static bool bmi160_is_gyr_normal(uint8_t pmu)
{
    // PMU_STATUS: GYR pmu bits [1:0]
    return (pmu & 0x03) == 0x03;
}

static esp_err_t bmi160_spi_write(bmi160_handle_t h, uint8_t reg, const uint8_t *data, size_t len)
{
    bmi160_handle_int_t *handle = (bmi160_handle_int_t*)h;
    uint8_t header = bmi160_spi_make_addr(reg, false);
    spi_transaction_t t = {
        .flags = 0,
        .length = (len + 1) * 8,
    };
    uint8_t buf[1 + 16];
    if (len > 16) return ESP_ERR_INVALID_ARG;
    buf[0] = header;
    if (len && data) memcpy(&buf[1], data, len);
    t.tx_buffer = buf;
    return spi_device_transmit(handle->spi, &t);
}

static esp_err_t bmi160_spi_read(bmi160_handle_t h, uint8_t reg, uint8_t *data, size_t len)
{
    bmi160_handle_int_t *handle = (bmi160_handle_int_t*)h;
    uint8_t header = bmi160_spi_make_addr(reg, true);
    
    // For large reads (like FIFO), use dynamic allocation
    if (len > 16) {
        uint8_t *buf_tx = malloc(len + 1);
        uint8_t *buf_rx = malloc(len + 1);
        if (!buf_tx || !buf_rx) {
            free(buf_tx);
            free(buf_rx);
            return ESP_ERR_NO_MEM;
        }
        
        memset(buf_tx, 0, len + 1);
        memset(buf_rx, 0, len + 1);
        buf_tx[0] = header;
        
        spi_transaction_t t = {
            .flags = 0,
            .length = (len + 1) * 8,
            .tx_buffer = buf_tx,
            .rx_buffer = buf_rx,
        };
        
        // Add timeout for large transactions
        esp_err_t err = spi_device_transmit(handle->spi, &t);
        if (err == ESP_OK && len) {
            memcpy(data, &buf_rx[1], len);
        }
        
        free(buf_tx);
        free(buf_rx);
        return err;
    }
    
    // For small reads, use stack buffers
    uint8_t buf_tx[1 + 16] = {0};
    uint8_t buf_rx[1 + 16] = {0};
    buf_tx[0] = header;
    
    spi_transaction_t t = {
        .flags = 0,
        .length = (len + 1) * 8,
        .tx_buffer = buf_tx,
        .rx_buffer = buf_rx,
    };
    
    esp_err_t err = spi_device_transmit(handle->spi, &t);
    if (err != ESP_OK) return err;
    if (len) memcpy(data, &buf_rx[1], len);
    return ESP_OK;
}

esp_err_t bmi160_create(const bmi160_spi_bus_config_t *bus_cfg, bmi160_handle_t *out_handle)
{
    if (!bus_cfg || !out_handle) return ESP_ERR_INVALID_ARG;

    spi_bus_config_t buscfg = {
        .mosi_io_num = bus_cfg->gpio_mosi,
        .miso_io_num = bus_cfg->gpio_miso,
        .sclk_io_num = bus_cfg->gpio_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    // Init SPI bus if not already
    esp_err_t err = spi_bus_initialize(bus_cfg->spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = bus_cfg->clock_speed_hz > 0 ? bus_cfg->clock_speed_hz : 5 * 1000 * 1000,
        .spics_io_num = bus_cfg->gpio_cs,
        .queue_size = 3,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
    };

    bmi160_handle_int_t *handle = calloc(1, sizeof(bmi160_handle_int_t));
    if (!handle) return ESP_ERR_NO_MEM;
    handle->gpio_cs = bus_cfg->gpio_cs;
    handle->gpio_int1 = bus_cfg->gpio_int1;
    handle->gpio_int2 = bus_cfg->gpio_int2;

    err = spi_bus_add_device(bus_cfg->spi_host, &devcfg, &handle->spi);
    if (err != ESP_OK) { free(handle); return err; }

    // Configure INT pins
    if (handle->gpio_int1 >= 0) {
        gpio_config_t io = { .pin_bit_mask = 1ULL << handle->gpio_int1, .mode = GPIO_MODE_INPUT, .pull_up_en = 1, .pull_down_en = 0, .intr_type = GPIO_INTR_ANYEDGE };
        gpio_config(&io);
    }
    if (handle->gpio_int2 >= 0) {
        gpio_config_t io = { .pin_bit_mask = 1ULL << handle->gpio_int2, .mode = GPIO_MODE_INPUT, .pull_up_en = 1, .pull_down_en = 0, .intr_type = GPIO_INTR_ANYEDGE };
        gpio_config(&io);
    }

    *out_handle = (bmi160_handle_t)handle;
    ESP_LOGI(TAG_BMI160, "created");
    return ESP_OK;
}

esp_err_t bmi160_init_default(bmi160_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
	uint8_t chip_id = 0;
    // Soft reset
    uint8_t cmd = BMI160_CMD_SOFTRESET;
    ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_CMD, &cmd, 1), TAG_BMI160, "cmd write failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    // Read chip id
	ESP_RETURN_ON_ERROR(bmi160_spi_read(handle, BMI160_REG_CHIP_ID, &chip_id, 1), TAG_BMI160, "id read failed");
	// Validate WHO_AM_I (0xD1). Retry a few times after reset in case bus settles late.
	if (chip_id != 0xD1) {
		for (int i = 0; i < 5 && chip_id != 0xD1; ++i) {
			vTaskDelay(pdMS_TO_TICKS(5));
			(void)bmi160_spi_read(handle, BMI160_REG_CHIP_ID, &chip_id, 1);
		}
	}
	if (chip_id != 0xD1) {
		ESP_LOGE(TAG_BMI160, "unexpected CHIP_ID=0x%02X (expect 0xD1)", (unsigned)chip_id);
		return ESP_ERR_INVALID_RESPONSE;
	}
	ESP_LOGI(TAG_BMI160, "chip id OK: 0x%02X", (unsigned)chip_id);

    // Enable accelerometer and gyro normal mode
    cmd = BMI160_CMD_ACC_PMU_NORMAL; ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_CMD, &cmd, 1), TAG_BMI160, "acc pmu");
    vTaskDelay(pdMS_TO_TICKS(5));
    cmd = BMI160_CMD_GYR_PMU_NORMAL; ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_CMD, &cmd, 1), TAG_BMI160, "gyr pmu");
    vTaskDelay(pdMS_TO_TICKS(80));

    // Basic ODR configs: ACC_CONF (ODR=100Hz, bandwidth avg)
    uint8_t acc_conf = 0x28; // ODR=100Hz (0x8), OSR4 avg (0x2 <<4 -> 0x20) combined -> 0x28
    ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_ACC_CONF, &acc_conf, 1), TAG_BMI160, "acc conf");
    uint8_t gyr_conf = 0x28; // ODR=100Hz, normal filter
    ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_GYR_CONF, &gyr_conf, 1), TAG_BMI160, "gyr conf");

    // Default ranges: 4G and 500 dps
    uint8_t acc_range = BMI160_ACC_RANGE_4G;
    ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_ACC_RANGE, &acc_range, 1), TAG_BMI160, "acc range");
    uint8_t gyr_range = BMI160_GYR_RANGE_500_DPS;
    ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_GYR_RANGE, &gyr_range, 1), TAG_BMI160, "gyr range");

    // INT output push-pull, active high on INT1 and INT2
    uint8_t int_out = 0;
    int_out |= (1 << 0); // INT1 push-pull
    int_out |= (1 << 1); // INT1 active high
    int_out |= (1 << 2); // INT2 push-pull
    int_out |= (1 << 3); // INT2 active high
    ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_INT_OUT_CTRL, &int_out, 1), TAG_BMI160, "int out");

    // Latch duration 80ms
    uint8_t int_latch = 0x0B; // latch 80ms
    ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_INT_LATCH, &int_latch, 1), TAG_BMI160, "int latch");

    // Configure FIFO: Enable accelerometer and gyroscope FIFO with header mode
    ESP_RETURN_ON_ERROR(bmi160_set_accel_fifo_enabled(handle, true), TAG_BMI160, "accel fifo enable");
    ESP_RETURN_ON_ERROR(bmi160_set_gyro_fifo_enabled(handle, false), TAG_BMI160, "gyro fifo enable");
    ESP_RETURN_ON_ERROR(bmi160_set_fifo_header_mode_enabled(handle, false), TAG_BMI160, "fifo header enable");
    
    // Enable FIFO buffer full interrupt (optional)
    ESP_RETURN_ON_ERROR(bmi160_set_int_fifo_buffer_full_enabled(handle, true), TAG_BMI160, "fifo full int enable");

    return ESP_OK;
}

esp_err_t bmi160_config_ranges(bmi160_handle_t handle, bmi160_acc_range_t acc, bmi160_gyr_range_t gyr)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    uint8_t v = (uint8_t)acc;
    ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_ACC_RANGE, &v, 1), TAG_BMI160, "acc range");
    v = (uint8_t)gyr;
    ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_GYR_RANGE, &v, 1), TAG_BMI160, "gyr range");
    return ESP_OK;
}

esp_err_t bmi160_enable_anymotion_wakeup(bmi160_handle_t handle, uint8_t threshold, uint8_t duration)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    // Configure any-motion on all axes
    // INT_MOTION_0: any-motion (slope) threshold LSB = 7.81mg at 2G range. We just pass raw.
    ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_INT_MOTION_0, &threshold, 1), TAG_BMI160, "mot0");
    // INT_MOTION_1: duration
    ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_INT_MOTION_1, &duration, 1), TAG_BMI160, "mot1");
    // INT_MOTION_2: enable axes XYZ
    uint8_t mot2 = 0x07; // x,y,z enable
    ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_INT_MOTION_2, &mot2, 1), TAG_BMI160, "mot2");
    // INT enable: INT_EN_0 bit 0..2 for any-motion xyz
    uint8_t en0 = 0x07;
    ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_INT_EN_0, &en0, 1), TAG_BMI160, "en0");
    // Map any-motion to INT1
    uint8_t map0 = 0x07; // map any-motion xyz to INT1
    ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_INT_MAP_0, &map0, 1), TAG_BMI160, "map0");
    return ESP_OK;
}

static inline int bmi160_acc_lsb_mg_for_range(uint8_t acc_range_reg)
{
    // Per BMI160 datasheet, typical LSB/ACC at different ranges (approx mg/LSB):
    // 2G: 0.061 mg/LSB; 4G: 0.122; 8G: 0.244; 16G: 0.488
    // We'll use integer micro-g per LSB to avoid float.
    switch (acc_range_reg & 0x0F) {
        case BMI160_ACC_RANGE_2G:   return 61;   // 0.061 mg
        case BMI160_ACC_RANGE_4G:   return 122;  // 0.122 mg
        case BMI160_ACC_RANGE_8G:   return 244;  // 0.244 mg
        case BMI160_ACC_RANGE_16G:  return 488;  // 0.488 mg
        default: return 122; // fallback 4G
    }
}

esp_err_t bmi160_enable_anymotion_wakeup_ms2(bmi160_handle_t handle, int ms2_x100, uint8_t duration)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    // Read current ACC_RANGE to compute LSB
    uint8_t acc_range = 0;
    ESP_RETURN_ON_ERROR(bmi160_spi_read(handle, BMI160_REG_ACC_RANGE, &acc_range, 1), TAG_BMI160, "rd acc_range");
    int lsb_mg = bmi160_acc_lsb_mg_for_range(acc_range); // mg per LSB in thousandths
    // Convert m/s^2*100 to mg: 1g = 9.80665 m/s^2 = 980.665 cm/s^2 = 9806.65 mg
    // ms2_x100 (m/s^2 * 100) -> mg = ms2_x100 * 1000 / 9.80665 ≈ (ms2_x100 * 1000) / 9.807
    // Use integer math: mg ≈ (ms2_x100 * 1000 + 5) / 10 (approx 9.81) -> actually that's 100x off. Better:
    // mg ≈ (ms2_x100 * 1000) / 981 -> since ms2_x100 already *100, true formula: mg = (ms2_x100 * 1000) / 981
    int mg = (ms2_x100 * 1000 + 490) / 981;
    // Now convert mg to LSB based on range
    // lsb_mg is mg per LSB in thousandths (e.g., 122 -> 0.122 mg/LSB). Our mg is in mg.
    // LSB = mg / (lsb_mg/1000) = mg * 1000 / lsb_mg
    int lsb = (mg * 1000 + (lsb_mg/2)) / lsb_mg;
    if (lsb < 1) lsb = 1;
    if (lsb > 255) lsb = 255;
    uint8_t thr = (uint8_t)lsb;
    ESP_LOGI(TAG_BMI160, "any-motion threshold ~%d mg (LSB=%u) at range reg 0x%02X", mg, thr, acc_range);
    return bmi160_enable_anymotion_wakeup(handle, thr, duration);
}

esp_err_t bmi160_read_sample(bmi160_handle_t handle, bmi160_sample_t *out)
{
    if (!handle || !out) return ESP_ERR_INVALID_ARG;
    // Ensure data ready to avoid stale zeros
    (void)bmi160_force_acc_normal(handle, 50);
    (void)bmi160_wait_data_ready(handle, 20);
    // Optionally verify PMU status shows acc/gyr normal (bits per datasheet: 0x03 ACC normal, 0x03 GYR normal in nibbles)
    uint8_t pmu = 0;
    if (bmi160_read_pmu_status(handle, &pmu) == ESP_OK) {
        // If either sensor appears in suspend (value 0x00), warn via log once in a while
        bool acc_susp = ((pmu & 0x30) >> 4) == 0x00;
        bool gyr_susp = (pmu & 0x03) == 0x00;
        if (acc_susp || gyr_susp) {
            ESP_LOGW(TAG_BMI160, "PMU suggests acc_susp=%d gyr_susp=%d", (int)acc_susp, (int)gyr_susp);
        }
    }
	uint8_t buf[14] = {0};
    // Read gyro (0x0C..0x11) and accel (0x12..0x17) and temp (0x20..0x21)
    ESP_RETURN_ON_ERROR(bmi160_spi_read(handle, BMI160_REG_DATA_GYR_X_L, buf, 12), TAG_BMI160, "gyr+acc");
	// Detect floating or shorted bus patterns (all 0x00 or all 0xFF)
	bool all_zero = true, all_ff = true;
	for (int i = 0; i < 12; ++i) { if (buf[i] != 0x00) all_zero = false; if (buf[i] != 0xFF) all_ff = false; }
	uint8_t tbuf[2];
	ESP_RETURN_ON_ERROR(bmi160_spi_read(handle, BMI160_REG_TEMP_L, tbuf, 2), TAG_BMI160, "temp");
	if ((all_zero || all_ff) && (tbuf[0] == tbuf[1]) && (tbuf[0] == 0x00 || tbuf[0] == 0xFF)) {
		return ESP_ERR_INVALID_RESPONSE;
	}
	int16_t gx = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t gy = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t gz = (int16_t)((buf[5] << 8) | buf[4]);
    int16_t ax = (int16_t)((buf[7] << 8) | buf[6]);
    int16_t ay = (int16_t)((buf[9] << 8) | buf[8]);
    int16_t az = (int16_t)((buf[11] << 8) | buf[10]);
    // Fallback: if accel looks all-zero while gyro has data, read accel registers separately once
    if ((ax == 0 && ay == 0 && az == 0) && (gx != 0 || gy != 0 || gz != 0)) {
        uint8_t abuf[6] = {0};
        if (bmi160_spi_read(handle, BMI160_REG_DATA_ACC_X_L, abuf, 6) == ESP_OK) {
            ax = (int16_t)((abuf[1] << 8) | abuf[0]);
            ay = (int16_t)((abuf[3] << 8) | abuf[2]);
            az = (int16_t)((abuf[5] << 8) | abuf[4]);
        }
    }
    int16_t traw = (int16_t)((tbuf[1] << 8) | tbuf[0]);
	// Extra guard: if every parsed value is zero, likely not connected yet
	if (gx == 0 && gy == 0 && gz == 0 && ax == 0 && ay == 0 && az == 0 && traw == 0) {
		return ESP_ERR_INVALID_RESPONSE;
	}

    out->gyro_x = gx; out->gyro_y = gy; out->gyro_z = gz;
    out->accel_x = ax; out->accel_y = ay; out->accel_z = az;
    // Temperature conversion per datasheet: T[°C] = 23 + raw/512
    out->temperature_c_x100 = 2300 + ((int32_t)traw * 100) / 512;
    return ESP_OK;
}

esp_err_t bmi160_read_int_status(bmi160_handle_t handle, uint32_t *out_status)
{
    if (!handle || !out_status) return ESP_ERR_INVALID_ARG;
    uint8_t s0,s1,s2,s3;
    ESP_RETURN_ON_ERROR(bmi160_spi_read(handle, BMI160_REG_INT_STATUS_0, &s0, 1), TAG_BMI160, "s0");
    ESP_RETURN_ON_ERROR(bmi160_spi_read(handle, BMI160_REG_INT_STATUS_1, &s1, 1), TAG_BMI160, "s1");
    ESP_RETURN_ON_ERROR(bmi160_spi_read(handle, BMI160_REG_INT_STATUS_2, &s2, 1), TAG_BMI160, "s2");
    ESP_RETURN_ON_ERROR(bmi160_spi_read(handle, BMI160_REG_INT_STATUS_3, &s3, 1), TAG_BMI160, "s3");
    *out_status = (uint32_t)s0 | ((uint32_t)s1 << 8) | ((uint32_t)s2 << 16) | ((uint32_t)s3 << 24);
    return ESP_OK;
}

esp_err_t bmi160_read_status(bmi160_handle_t handle, uint8_t *out_status)
{
    if (!handle || !out_status) return ESP_ERR_INVALID_ARG;
    return bmi160_spi_read(handle, BMI160_REG_STATUS, out_status, 1);
}

esp_err_t bmi160_read_pmu_status(bmi160_handle_t handle, uint8_t *out_pmu_status)
{
    if (!handle || !out_pmu_status) return ESP_ERR_INVALID_ARG;
    return bmi160_spi_read(handle, BMI160_REG_PMU_STATUS, out_pmu_status, 1);
}

esp_err_t bmi160_wait_data_ready(bmi160_handle_t handle, int timeout_ms)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms >= 0 ? timeout_ms : 0);
    uint8_t status = 0;
    do {
        esp_err_t err = bmi160_read_status(handle, &status);
        if (err != ESP_OK) return err;
        bool acc_drdy = (status & (1u << 7)) != 0; // STATUS[7] accel data ready
        bool gyr_drdy = (status & (1u << 6)) != 0; // STATUS[6] gyro data ready
        if (acc_drdy && gyr_drdy) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(1));
    } while ((xTaskGetTickCount() - start) < timeout);
    return ESP_ERR_TIMEOUT;
}

esp_err_t bmi160_force_acc_normal(bmi160_handle_t handle, int timeout_ms)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    uint8_t pmu = 0;
    if (bmi160_read_pmu_status(handle, &pmu) != ESP_OK) return ESP_FAIL;
    if (!bmi160_is_acc_normal(pmu)) {
        uint8_t cmd = BMI160_CMD_ACC_PMU_NORMAL;
        ESP_RETURN_ON_ERROR(bmi160_spi_write(handle, BMI160_REG_CMD, &cmd, 1), TAG_BMI160, "acc pmu cmd");
    }
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms >= 0 ? timeout_ms : 0);
    do {
        if (bmi160_read_pmu_status(handle, &pmu) == ESP_OK && bmi160_is_acc_normal(pmu)) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(2));
    } while ((xTaskGetTickCount() - start) < timeout);
    return ESP_ERR_TIMEOUT;
}

esp_err_t bmi160_dump_regs(bmi160_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    uint8_t id=0, err=0, pmu=0, st=0, acc_range=0, acc_conf=0, gyr_conf=0, gyr_range=0;
    (void)bmi160_spi_read(handle, BMI160_REG_CHIP_ID, &id, 1);
    (void)bmi160_spi_read(handle, BMI160_REG_ERR, &err, 1);
    (void)bmi160_spi_read(handle, BMI160_REG_PMU_STATUS, &pmu, 1);
    (void)bmi160_spi_read(handle, BMI160_REG_STATUS, &st, 1);
    (void)bmi160_spi_read(handle, BMI160_REG_ACC_CONF, &acc_conf, 1);
    (void)bmi160_spi_read(handle, BMI160_REG_ACC_RANGE, &acc_range, 1);
    (void)bmi160_spi_read(handle, BMI160_REG_GYR_CONF, &gyr_conf, 1);
    (void)bmi160_spi_read(handle, BMI160_REG_GYR_RANGE, &gyr_range, 1);
    ESP_LOGI(TAG_BMI160, "DUMP id=0x%02X err=0x%02X pmu=0x%02X st=0x%02X acc_conf=0x%02X acc_range=0x%02X gyr_conf=0x%02X gyr_range=0x%02X",
        id, err, pmu, st, acc_conf, acc_range, gyr_conf, gyr_range);
    return ESP_OK;
}

void bmi160_destroy(bmi160_handle_t handle)
{
    if (!handle) return;
    bmi160_handle_int_t *h = (bmi160_handle_int_t*)handle;
    spi_bus_remove_device(h->spi);
    free(h);
}

// Helper functions for register bit operations
static esp_err_t bmi160_reg_read_bits(bmi160_handle_t handle, uint8_t reg, uint8_t *data, uint8_t pos, uint8_t len)
{
    uint8_t reg_data;
    esp_err_t err = bmi160_spi_read(handle, reg, &reg_data, 1);
    if (err != ESP_OK) return err;
    
    uint8_t mask = ((1 << len) - 1) << pos;
    *data = (reg_data & mask) >> pos;
    return ESP_OK;
}

static esp_err_t bmi160_reg_write_bits(bmi160_handle_t handle, uint8_t reg, uint8_t data, uint8_t pos, uint8_t len)
{
    uint8_t reg_data;
    esp_err_t err = bmi160_spi_read(handle, reg, &reg_data, 1);
    if (err != ESP_OK) return err;
    
    uint8_t mask = ((1 << len) - 1) << pos;
    data <<= pos;
    data &= mask;
    reg_data &= ~mask;
    reg_data |= data;
    
    return bmi160_spi_write(handle, reg, &reg_data, 1);
}

// Device ID and connection test
esp_err_t bmi160_get_device_id(bmi160_handle_t handle, uint8_t *device_id)
{
    if (!handle || !device_id) return ESP_ERR_INVALID_ARG;
    return bmi160_spi_read(handle, BMI160_REG_CHIP_ID, device_id, 1);
}

esp_err_t bmi160_test_connection(bmi160_handle_t handle, bool *is_connected)
{
    if (!handle || !is_connected) return ESP_ERR_INVALID_ARG;
    uint8_t device_id;
    esp_err_t err = bmi160_get_device_id(handle, &device_id);
    if (err != ESP_OK) return err;
    *is_connected = (device_id == 0xD1);
    return ESP_OK;
}

// Data rate configuration
esp_err_t bmi160_get_gyro_rate(bmi160_handle_t handle, uint8_t *rate)
{
    if (!handle || !rate) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_read_bits(handle, BMI160_REG_GYR_CONF, rate, BMI160_GYRO_RATE_SEL_BIT, BMI160_GYRO_RATE_SEL_LEN);
}

esp_err_t bmi160_set_gyro_rate(bmi160_handle_t handle, uint8_t rate)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_write_bits(handle, BMI160_REG_GYR_CONF, rate, BMI160_GYRO_RATE_SEL_BIT, BMI160_GYRO_RATE_SEL_LEN);
}

esp_err_t bmi160_get_accel_rate(bmi160_handle_t handle, uint8_t *rate)
{
    if (!handle || !rate) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_read_bits(handle, BMI160_REG_ACC_CONF, rate, BMI160_ACCEL_RATE_SEL_BIT, BMI160_ACCEL_RATE_SEL_LEN);
}

esp_err_t bmi160_set_accel_rate(bmi160_handle_t handle, uint8_t rate)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_write_bits(handle, BMI160_REG_ACC_CONF, rate, BMI160_ACCEL_RATE_SEL_BIT, BMI160_ACCEL_RATE_SEL_LEN);
}

// Digital Low-Pass Filter configuration
esp_err_t bmi160_get_gyro_dlpf_mode(bmi160_handle_t handle, uint8_t *mode)
{
    if (!handle || !mode) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_read_bits(handle, BMI160_REG_GYR_CONF, mode, BMI160_GYRO_DLPF_SEL_BIT, BMI160_GYRO_DLPF_SEL_LEN);
}

esp_err_t bmi160_set_gyro_dlpf_mode(bmi160_handle_t handle, uint8_t mode)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_write_bits(handle, BMI160_REG_GYR_CONF, mode, BMI160_GYRO_DLPF_SEL_BIT, BMI160_GYRO_DLPF_SEL_LEN);
}

esp_err_t bmi160_get_accel_dlpf_mode(bmi160_handle_t handle, uint8_t *mode)
{
    if (!handle || !mode) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_read_bits(handle, BMI160_REG_ACC_CONF, mode, BMI160_ACCEL_DLPF_SEL_BIT, BMI160_ACCEL_DLPF_SEL_LEN);
}

esp_err_t bmi160_set_accel_dlpf_mode(bmi160_handle_t handle, uint8_t mode)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_write_bits(handle, BMI160_REG_ACC_CONF, mode, BMI160_ACCEL_DLPF_SEL_BIT, BMI160_ACCEL_DLPF_SEL_LEN);
}

// Range configuration
esp_err_t bmi160_get_full_scale_gyro_range(bmi160_handle_t handle, uint8_t *range)
{
    if (!handle || !range) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_read_bits(handle, BMI160_REG_GYR_RANGE, range, BMI160_GYRO_RANGE_SEL_BIT, BMI160_GYRO_RANGE_SEL_LEN);
}

esp_err_t bmi160_set_full_scale_gyro_range(bmi160_handle_t handle, uint8_t range)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_write_bits(handle, BMI160_REG_GYR_RANGE, range, BMI160_GYRO_RANGE_SEL_BIT, BMI160_GYRO_RANGE_SEL_LEN);
}

esp_err_t bmi160_get_full_scale_accel_range(bmi160_handle_t handle, uint8_t *range)
{
    if (!handle || !range) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_read_bits(handle, BMI160_REG_ACC_RANGE, range, BMI160_ACCEL_RANGE_SEL_BIT, BMI160_ACCEL_RANGE_SEL_LEN);
}

esp_err_t bmi160_set_full_scale_accel_range(bmi160_handle_t handle, uint8_t range)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_write_bits(handle, BMI160_REG_ACC_RANGE, range, BMI160_ACCEL_RANGE_SEL_BIT, BMI160_ACCEL_RANGE_SEL_LEN);
}

// Individual sensor reading functions
esp_err_t bmi160_get_acceleration_x(bmi160_handle_t handle, int16_t *x)
{
    if (!handle || !x) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2];
    esp_err_t err = bmi160_spi_read(handle, BMI160_REG_DATA_ACC_X_L, buf, 2);
    if (err != ESP_OK) return err;
    *x = (int16_t)((buf[1] << 8) | buf[0]);
    return ESP_OK;
}

esp_err_t bmi160_get_acceleration_y(bmi160_handle_t handle, int16_t *y)
{
    if (!handle || !y) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2];
    esp_err_t err = bmi160_spi_read(handle, BMI160_REG_DATA_ACC_Y_L, buf, 2);
    if (err != ESP_OK) return err;
    *y = (int16_t)((buf[1] << 8) | buf[0]);
    return ESP_OK;
}

esp_err_t bmi160_get_acceleration_z(bmi160_handle_t handle, int16_t *z)
{
    if (!handle || !z) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2];
    esp_err_t err = bmi160_spi_read(handle, BMI160_REG_DATA_ACC_Z_L, buf, 2);
    if (err != ESP_OK) return err;
    *z = (int16_t)((buf[1] << 8) | buf[0]);
    return ESP_OK;
}

esp_err_t bmi160_get_rotation_x(bmi160_handle_t handle, int16_t *x)
{
    if (!handle || !x) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2];
    esp_err_t err = bmi160_spi_read(handle, BMI160_REG_DATA_GYR_X_L, buf, 2);
    if (err != ESP_OK) return err;
    *x = (int16_t)((buf[1] << 8) | buf[0]);
    return ESP_OK;
}

esp_err_t bmi160_get_rotation_y(bmi160_handle_t handle, int16_t *y)
{
    if (!handle || !y) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2];
    esp_err_t err = bmi160_spi_read(handle, BMI160_REG_DATA_GYR_Y_L, buf, 2);
    if (err != ESP_OK) return err;
    *y = (int16_t)((buf[1] << 8) | buf[0]);
    return ESP_OK;
}

esp_err_t bmi160_get_rotation_z(bmi160_handle_t handle, int16_t *z)
{
    if (!handle || !z) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2];
    esp_err_t err = bmi160_spi_read(handle, BMI160_REG_DATA_GYR_Z_L, buf, 2);
    if (err != ESP_OK) return err;
    *z = (int16_t)((buf[1] << 8) | buf[0]);
    return ESP_OK;
}

esp_err_t bmi160_get_temperature(bmi160_handle_t handle, int16_t *temperature)
{
    if (!handle || !temperature) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2];
    esp_err_t err = bmi160_spi_read(handle, BMI160_REG_TEMP_L, buf, 2);
    if (err != ESP_OK) return err;
    *temperature = (int16_t)((buf[1] << 8) | buf[0]);
    return ESP_OK;
}

// Register access functions
esp_err_t bmi160_get_register(bmi160_handle_t handle, uint8_t reg, uint8_t *data)
{
    if (!handle || !data) return ESP_ERR_INVALID_ARG;
    return bmi160_spi_read(handle, reg, data, 1);
}

esp_err_t bmi160_set_register(bmi160_handle_t handle, uint8_t reg, uint8_t data)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    return bmi160_spi_write(handle, reg, &data, 1);
}

// Calibration functions
esp_err_t bmi160_auto_calibrate_gyro_offset(bmi160_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    uint8_t foc_conf = (1 << BMI160_FOC_GYR_EN);
    esp_err_t err = bmi160_spi_write(handle, BMI160_REG_FOC_CONF, &foc_conf, 1);
    if (err != ESP_OK) return err;
    
    uint8_t cmd = BMI160_CMD_START_FOC;
    err = bmi160_spi_write(handle, BMI160_REG_CMD, &cmd, 1);
    if (err != ESP_OK) return err;
    
    // Wait for FOC to complete
    uint8_t status;
    for (int i = 0; i < 250; i++) {
        err = bmi160_spi_read(handle, BMI160_REG_STATUS, &status, 1);
        if (err != ESP_OK) return err;
        if (status & (1 << BMI160_STATUS_FOC_RDY)) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t bmi160_auto_calibrate_x_accel_offset(bmi160_handle_t handle, int target)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    uint8_t foc_conf;
    if (target == 1)
        foc_conf = (0x1 << BMI160_FOC_ACC_X_BIT);
    else if (target == -1)
        foc_conf = (0x2 << BMI160_FOC_ACC_X_BIT);
    else if (target == 0)
        foc_conf = (0x3 << BMI160_FOC_ACC_X_BIT);
    else
        return ESP_ERR_INVALID_ARG;
    
    esp_err_t err = bmi160_spi_write(handle, BMI160_REG_FOC_CONF, &foc_conf, 1);
    if (err != ESP_OK) return err;
    
    uint8_t cmd = BMI160_CMD_START_FOC;
    err = bmi160_spi_write(handle, BMI160_REG_CMD, &cmd, 1);
    if (err != ESP_OK) return err;
    
    // Wait for FOC to complete
    uint8_t status;
    for (int i = 0; i < 250; i++) {
        err = bmi160_spi_read(handle, BMI160_REG_STATUS, &status, 1);
        if (err != ESP_OK) return err;
        if (status & (1 << BMI160_STATUS_FOC_RDY)) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t bmi160_auto_calibrate_y_accel_offset(bmi160_handle_t handle, int target)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    uint8_t foc_conf;
    if (target == 1)
        foc_conf = (0x1 << BMI160_FOC_ACC_Y_BIT);
    else if (target == -1)
        foc_conf = (0x2 << BMI160_FOC_ACC_Y_BIT);
    else if (target == 0)
        foc_conf = (0x3 << BMI160_FOC_ACC_Y_BIT);
    else
        return ESP_ERR_INVALID_ARG;
    
    esp_err_t err = bmi160_spi_write(handle, BMI160_REG_FOC_CONF, &foc_conf, 1);
    if (err != ESP_OK) return err;
    
    uint8_t cmd = BMI160_CMD_START_FOC;
    err = bmi160_spi_write(handle, BMI160_REG_CMD, &cmd, 1);
    if (err != ESP_OK) return err;
    
    // Wait for FOC to complete
    uint8_t status;
    for (int i = 0; i < 250; i++) {
        err = bmi160_spi_read(handle, BMI160_REG_STATUS, &status, 1);
        if (err != ESP_OK) return err;
        if (status & (1 << BMI160_STATUS_FOC_RDY)) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t bmi160_auto_calibrate_z_accel_offset(bmi160_handle_t handle, int target)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    uint8_t foc_conf;
    if (target == 1)
        foc_conf = (0x1 << BMI160_FOC_ACC_Z_BIT);
    else if (target == -1)
        foc_conf = (0x2 << BMI160_FOC_ACC_Z_BIT);
    else if (target == 0)
        foc_conf = (0x3 << BMI160_FOC_ACC_Z_BIT);
    else
        return ESP_ERR_INVALID_ARG;
    
    esp_err_t err = bmi160_spi_write(handle, BMI160_REG_FOC_CONF, &foc_conf, 1);
    if (err != ESP_OK) return err;
    
    uint8_t cmd = BMI160_CMD_START_FOC;
    err = bmi160_spi_write(handle, BMI160_REG_CMD, &cmd, 1);
    if (err != ESP_OK) return err;
    
    // Wait for FOC to complete
    uint8_t status;
    for (int i = 0; i < 250; i++) {
        err = bmi160_spi_read(handle, BMI160_REG_STATUS, &status, 1);
        if (err != ESP_OK) return err;
        if (status & (1 << BMI160_STATUS_FOC_RDY)) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_ERR_TIMEOUT;
}

// Interrupt status functions
esp_err_t bmi160_get_int_status0(bmi160_handle_t handle, uint8_t *status)
{
    if (!handle || !status) return ESP_ERR_INVALID_ARG;
    return bmi160_spi_read(handle, BMI160_REG_INT_STATUS_0, status, 1);
}

esp_err_t bmi160_get_int_status1(bmi160_handle_t handle, uint8_t *status)
{
    if (!handle || !status) return ESP_ERR_INVALID_ARG;
    return bmi160_spi_read(handle, BMI160_REG_INT_STATUS_1, status, 1);
}

esp_err_t bmi160_get_int_status2(bmi160_handle_t handle, uint8_t *status)
{
    if (!handle || !status) return ESP_ERR_INVALID_ARG;
    return bmi160_spi_read(handle, BMI160_REG_INT_STATUS_2, status, 1);
}

esp_err_t bmi160_get_int_status3(bmi160_handle_t handle, uint8_t *status)
{
    if (!handle || !status) return ESP_ERR_INVALID_ARG;
    return bmi160_spi_read(handle, BMI160_REG_INT_STATUS_3, status, 1);
}

esp_err_t bmi160_get_int_freefall_status(bmi160_handle_t handle, bool *status)
{
    if (!handle || !status) return ESP_ERR_INVALID_ARG;
    uint8_t int_status;
    esp_err_t err = bmi160_spi_read(handle, BMI160_REG_INT_STATUS_1, &int_status, 1);
    if (err != ESP_OK) return err;
    *status = (int_status & (1 << BMI160_LOW_G_INT_BIT)) != 0;
    return ESP_OK;
}

esp_err_t bmi160_get_int_shock_status(bmi160_handle_t handle, bool *status)
{
    if (!handle || !status) return ESP_ERR_INVALID_ARG;
    uint8_t int_status;
    esp_err_t err = bmi160_spi_read(handle, BMI160_REG_INT_STATUS_1, &int_status, 1);
    if (err != ESP_OK) return err;
    *status = (int_status & (1 << BMI160_HIGH_G_INT_BIT)) != 0;
    return ESP_OK;
}

esp_err_t bmi160_get_int_step_status(bmi160_handle_t handle, bool *status)
{
    if (!handle || !status) return ESP_ERR_INVALID_ARG;
    uint8_t int_status;
    esp_err_t err = bmi160_spi_read(handle, BMI160_REG_INT_STATUS_0, &int_status, 1);
    if (err != ESP_OK) return err;
    *status = (int_status & (1 << BMI160_STEP_INT_BIT)) != 0;
    return ESP_OK;
}

esp_err_t bmi160_get_int_motion_status(bmi160_handle_t handle, bool *status)
{
    if (!handle || !status) return ESP_ERR_INVALID_ARG;
    uint8_t int_status;
    esp_err_t err = bmi160_spi_read(handle, BMI160_REG_INT_STATUS_0, &int_status, 1);
    if (err != ESP_OK) return err;
    *status = (int_status & (1 << BMI160_ANYMOTION_INT_BIT)) != 0;
    return ESP_OK;
}

esp_err_t bmi160_get_int_tap_status(bmi160_handle_t handle, bool *status)
{
    if (!handle || !status) return ESP_ERR_INVALID_ARG;
    uint8_t int_status;
    esp_err_t err = bmi160_spi_read(handle, BMI160_REG_INT_STATUS_0, &int_status, 1);
    if (err != ESP_OK) return err;
    *status = (int_status & (1 << BMI160_S_TAP_INT_BIT)) != 0;
    return ESP_OK;
}

esp_err_t bmi160_get_int_double_tap_status(bmi160_handle_t handle, bool *status)
{
    if (!handle || !status) return ESP_ERR_INVALID_ARG;
    uint8_t int_status;
    esp_err_t err = bmi160_spi_read(handle, BMI160_REG_INT_STATUS_0, &int_status, 1);
    if (err != ESP_OK) return err;
    *status = (int_status & (1 << BMI160_D_TAP_INT_BIT)) != 0;
    return ESP_OK;
}

// FIFO functions
esp_err_t bmi160_reset_fifo(bmi160_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    uint8_t cmd = BMI160_CMD_FIFO_FLUSH;
    return bmi160_spi_write(handle, BMI160_REG_CMD, &cmd, 1);
}

esp_err_t bmi160_get_fifo_count(bmi160_handle_t handle, uint16_t *count)
{
    if (!handle || !count) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2];
    esp_err_t err = bmi160_spi_read(handle, BMI160_REG_FIFO_LENGTH_0, buf, 2);
    if (err != ESP_OK) return err;
    *count = (uint16_t)((buf[1] << 8) | buf[0]);
    return ESP_OK;
}

esp_err_t bmi160_get_fifo_bytes(bmi160_handle_t handle, uint8_t *data, uint16_t length)
{
    if (!handle || !data || length == 0) return ESP_ERR_INVALID_ARG;
    return bmi160_spi_read(handle, BMI160_REG_FIFO_DATA, data, length);
}

// Step detection functions
esp_err_t bmi160_get_step_count(bmi160_handle_t handle, uint16_t *count)
{
    if (!handle || !count) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2];
    esp_err_t err = bmi160_spi_read(handle, BMI160_REG_STEP_CNT_L, buf, 2);
    if (err != ESP_OK) return err;
    *count = (uint16_t)((buf[1] << 8) | buf[0]);
    return ESP_OK;
}

esp_err_t bmi160_reset_step_count(bmi160_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    uint8_t cmd = BMI160_CMD_STEP_CNT_CLR;
    return bmi160_spi_write(handle, BMI160_REG_CMD, &cmd, 1);
}

esp_err_t bmi160_reset_interrupt(bmi160_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    uint8_t cmd = BMI160_CMD_INT_RESET;
    return bmi160_spi_write(handle, BMI160_REG_CMD, &cmd, 1);
}

// FIFO configuration functions
esp_err_t bmi160_set_gyro_fifo_enabled(bmi160_handle_t handle, bool enabled)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_write_bits(handle, BMI160_REG_FIFO_CONFIG_1, enabled ? 1 : 0, BMI160_FIFO_GYR_EN_BIT, 1);
}

esp_err_t bmi160_get_gyro_fifo_enabled(bmi160_handle_t handle, bool *enabled)
{
    if (!handle || !enabled) return ESP_ERR_INVALID_ARG;
    uint8_t reg_val;
    esp_err_t err = bmi160_reg_read_bits(handle, BMI160_REG_FIFO_CONFIG_1, &reg_val, BMI160_FIFO_GYR_EN_BIT, 1);
    if (err != ESP_OK) return err;
    *enabled = (reg_val != 0);
    return ESP_OK;
}

esp_err_t bmi160_set_accel_fifo_enabled(bmi160_handle_t handle, bool enabled)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_write_bits(handle, BMI160_REG_FIFO_CONFIG_1, enabled ? 1 : 0, BMI160_FIFO_ACC_EN_BIT, 1);
}

esp_err_t bmi160_get_accel_fifo_enabled(bmi160_handle_t handle, bool *enabled)
{
    if (!handle || !enabled) return ESP_ERR_INVALID_ARG;
    uint8_t reg_val;
    esp_err_t err = bmi160_reg_read_bits(handle, BMI160_REG_FIFO_CONFIG_1, &reg_val, BMI160_FIFO_ACC_EN_BIT, 1);
    if (err != ESP_OK) return err;
    *enabled = (reg_val != 0);
    return ESP_OK;
}

esp_err_t bmi160_set_fifo_header_mode_enabled(bmi160_handle_t handle, bool enabled)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_write_bits(handle, BMI160_REG_FIFO_CONFIG_1, enabled ? 1 : 0, BMI160_FIFO_HEADER_EN_BIT, 1);
}

esp_err_t bmi160_get_fifo_header_mode_enabled(bmi160_handle_t handle, bool *enabled)
{
    if (!handle || !enabled) return ESP_ERR_INVALID_ARG;
    uint8_t reg_val;
    esp_err_t err = bmi160_reg_read_bits(handle, BMI160_REG_FIFO_CONFIG_1, &reg_val, BMI160_FIFO_HEADER_EN_BIT, 1);
    if (err != ESP_OK) return err;
    *enabled = (reg_val != 0);
    return ESP_OK;
}

esp_err_t bmi160_set_int_fifo_buffer_full_enabled(bmi160_handle_t handle, bool enabled)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_write_bits(handle, BMI160_REG_INT_EN_1, enabled ? 1 : 0, BMI160_FFULL_EN_BIT, 1);
}

esp_err_t bmi160_get_int_fifo_buffer_full_enabled(bmi160_handle_t handle, bool *enabled)
{
    if (!handle || !enabled) return ESP_ERR_INVALID_ARG;
    uint8_t reg_val;
    esp_err_t err = bmi160_reg_read_bits(handle, BMI160_REG_INT_EN_1, &reg_val, BMI160_FFULL_EN_BIT, 1);
    if (err != ESP_OK) return err;
    *enabled = (reg_val != 0);
    return ESP_OK;
}

esp_err_t bmi160_set_int_data_ready_enabled(bmi160_handle_t handle, bool enabled)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    return bmi160_reg_write_bits(handle, BMI160_REG_INT_EN_1, enabled ? 1 : 0, BMI160_DRDY_EN_BIT, 1);
}

esp_err_t bmi160_get_int_data_ready_enabled(bmi160_handle_t handle, bool *enabled)
{
    if (!handle || !enabled) return ESP_ERR_INVALID_ARG;
    uint8_t reg_val;
    esp_err_t err = bmi160_reg_read_bits(handle, BMI160_REG_INT_EN_1, &reg_val, BMI160_DRDY_EN_BIT, 1);
    if (err != ESP_OK) return err;
    *enabled = (reg_val != 0);
    return ESP_OK;
}

// FIFO headerless mode functions
esp_err_t bmi160_read_fifo_headerless_accel(bmi160_handle_t handle, int16_t *accel_data, uint16_t *sample_count)
{
    if (!handle || !accel_data || !sample_count) return ESP_ERR_INVALID_ARG;
    
    // Initialize output parameters
    *sample_count = 0;
    
    uint16_t fifo_count = 0;
    esp_err_t err = bmi160_get_fifo_count(handle, &fifo_count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BMI160, "Failed to get FIFO count: %s", esp_err_to_name(err));
        return err;
    }
    
    // Check if we have enough data for at least one accelerometer frame
    if (fifo_count < BMI160_ACCEL_FRAME_SIZE) {
        ESP_LOGD(TAG_BMI160, "Not enough FIFO data: %d bytes (need %d)", fifo_count, BMI160_ACCEL_FRAME_SIZE);
        return ESP_OK;
    }
    
    // Limit read size to prevent buffer overflow
    uint16_t max_samples = 100; // Maximum samples to read at once
    uint16_t max_bytes = max_samples * BMI160_ACCEL_FRAME_SIZE;
    if (fifo_count > max_bytes) {
        ESP_LOGW(TAG_BMI160, "FIFO count (%d) exceeds max read size (%d), limiting read", fifo_count, max_bytes);
        fifo_count = max_bytes;
    }
    
    // Ensure we read complete frames only
    uint16_t bytes_to_read = (fifo_count / BMI160_ACCEL_FRAME_SIZE) * BMI160_ACCEL_FRAME_SIZE;
    if (bytes_to_read == 0) {
        ESP_LOGW(TAG_BMI160, "No complete frames available in FIFO");
        return ESP_OK;
    }
    
    uint16_t expected_samples = bytes_to_read / BMI160_ACCEL_FRAME_SIZE;
    ESP_LOGD(TAG_BMI160, "Reading %d bytes (%d samples) from FIFO", bytes_to_read, expected_samples);
    
    // Allocate buffer for FIFO data
    uint8_t *fifo_buffer = malloc(bytes_to_read);
    if (!fifo_buffer) return ESP_ERR_NO_MEM;
    
    // Read FIFO data
    err = bmi160_get_fifo_bytes(handle, fifo_buffer, bytes_to_read);
    if (err != ESP_OK) {
        free(fifo_buffer);
        return err;
    }
    
    // Parse accelerometer data
    uint16_t samples_parsed = 0;
    for (uint16_t i = 0; i <= bytes_to_read - BMI160_ACCEL_FRAME_SIZE; i += BMI160_ACCEL_FRAME_SIZE) {
        // Extract accelerometer data (little-endian format)
        int16_t ax = (int16_t)((fifo_buffer[i + 1] << 8) | fifo_buffer[i + 0]);
        int16_t ay = (int16_t)((fifo_buffer[i + 3] << 8) | fifo_buffer[i + 2]);
        int16_t az = (int16_t)((fifo_buffer[i + 5] << 8) | fifo_buffer[i + 4]);
        
        // Basic validation: check for reasonable values (not all zeros or all 0xFF)
        bool valid_sample = !((ax == 0 && ay == 0 && az == 0) || 
                             (ax == -1 && ay == -1 && az == -1));
        
        if (valid_sample) {
            // Store in output array (3 values per sample: x, y, z)
            accel_data[samples_parsed * 3 + 0] = ax;
            accel_data[samples_parsed * 3 + 1] = ay;
            accel_data[samples_parsed * 3 + 2] = az;
            samples_parsed++;
        } else {
            ESP_LOGW(TAG_BMI160, "Invalid accelerometer sample detected at offset %d: ax=%d, ay=%d, az=%d", 
                     i, ax, ay, az);
        }
    }
    
    *sample_count = samples_parsed;
    free(fifo_buffer);
    
    ESP_LOGI(TAG_BMI160, "Read %d accelerometer samples from FIFO (%d bytes)", samples_parsed, bytes_to_read);
    return ESP_OK;
}

esp_err_t bmi160_read_fifo_headerless_accel_gyro(bmi160_handle_t handle, int16_t *accel_data, int16_t *gyro_data, uint16_t *sample_count)
{
    if (!handle || !accel_data || !gyro_data || !sample_count) return ESP_ERR_INVALID_ARG;
    
    uint16_t fifo_count = 0;
    esp_err_t err = bmi160_get_fifo_count(handle, &fifo_count);
    if (err != ESP_OK) return err;
    
    // Check if we have enough data for at least one accel+gyro frame
    if (fifo_count < BMI160_ACCEL_GYRO_FRAME_SIZE) {
        *sample_count = 0;
        return ESP_OK;
    }
    
    // Limit read size to prevent buffer overflow
    uint16_t max_samples = 50; // Maximum samples to read at once
    uint16_t max_bytes = max_samples * BMI160_ACCEL_GYRO_FRAME_SIZE;
    if (fifo_count > max_bytes) {
        fifo_count = max_bytes;
    }
    
    // Ensure we read complete frames only
    uint16_t bytes_to_read = (fifo_count / BMI160_ACCEL_GYRO_FRAME_SIZE) * BMI160_ACCEL_GYRO_FRAME_SIZE;
    if (bytes_to_read == 0) {
        *sample_count = 0;
        return ESP_OK;
    }
    
    // Allocate buffer for FIFO data
    uint8_t *fifo_buffer = malloc(bytes_to_read);
    if (!fifo_buffer) return ESP_ERR_NO_MEM;
    
    // Read FIFO data
    err = bmi160_get_fifo_bytes(handle, fifo_buffer, bytes_to_read);
    if (err != ESP_OK) {
        free(fifo_buffer);
        return err;
    }
    
    // Parse accelerometer and gyroscope data
    uint16_t samples_parsed = 0;
    for (uint16_t i = 0; i <= bytes_to_read - BMI160_ACCEL_GYRO_FRAME_SIZE; i += BMI160_ACCEL_GYRO_FRAME_SIZE) {
        // Extract accelerometer data (first 6 bytes)
        int16_t ax = (int16_t)((fifo_buffer[i + 1] << 8) | fifo_buffer[i + 0]);
        int16_t ay = (int16_t)((fifo_buffer[i + 3] << 8) | fifo_buffer[i + 2]);
        int16_t az = (int16_t)((fifo_buffer[i + 5] << 8) | fifo_buffer[i + 4]);
        
        // Extract gyroscope data (next 6 bytes)
        int16_t gx = (int16_t)((fifo_buffer[i + 7] << 8) | fifo_buffer[i + 6]);
        int16_t gy = (int16_t)((fifo_buffer[i + 9] << 8) | fifo_buffer[i + 8]);
        int16_t gz = (int16_t)((fifo_buffer[i + 11] << 8) | fifo_buffer[i + 10]);
        
        // Store in output arrays (3 values per sample: x, y, z)
        accel_data[samples_parsed * 3 + 0] = ax;
        accel_data[samples_parsed * 3 + 1] = ay;
        accel_data[samples_parsed * 3 + 2] = az;
        
        gyro_data[samples_parsed * 3 + 0] = gx;
        gyro_data[samples_parsed * 3 + 1] = gy;
        gyro_data[samples_parsed * 3 + 2] = gz;
        
        samples_parsed++;
    }
    
    *sample_count = samples_parsed;
    free(fifo_buffer);
    
    ESP_LOGI(TAG_BMI160, "Read %d accel+gyro samples from FIFO (%d bytes)", samples_parsed, bytes_to_read);
    return ESP_OK;
}

esp_err_t bmi160_init_headerless_mode(bmi160_handle_t handle, bool enable_accel, bool enable_gyro)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    ESP_LOGI(TAG_BMI160, "Initializing BMI160 for headerless FIFO mode (accel=%d, gyro=%d)", enable_accel, enable_gyro);
    
    // Disable header mode (RẤT QUAN TRỌNG - như bạn đã nhấn mạnh)
    ESP_RETURN_ON_ERROR(bmi160_set_fifo_header_mode_enabled(handle, false), TAG_BMI160, "disable fifo header");
    
    // Configure FIFO for accelerometer
    if (enable_accel) {
        ESP_RETURN_ON_ERROR(bmi160_set_accel_fifo_enabled(handle, true), TAG_BMI160, "enable accel fifo");
    } else {
        ESP_RETURN_ON_ERROR(bmi160_set_accel_fifo_enabled(handle, false), TAG_BMI160, "disable accel fifo");
    }
    
    // Configure FIFO for gyroscope
    if (enable_gyro) {
        ESP_RETURN_ON_ERROR(bmi160_set_gyro_fifo_enabled(handle, true), TAG_BMI160, "enable gyro fifo");
    } else {
        ESP_RETURN_ON_ERROR(bmi160_set_gyro_fifo_enabled(handle, false), TAG_BMI160, "disable gyro fifo");
    }
    
    // Enable FIFO buffer full interrupt (optional)
    ESP_RETURN_ON_ERROR(bmi160_set_int_fifo_buffer_full_enabled(handle, true), TAG_BMI160, "enable fifo full interrupt");
    
    // Flush FIFO to start fresh
    ESP_RETURN_ON_ERROR(bmi160_reset_fifo(handle), TAG_BMI160, "flush fifo");
    
    ESP_LOGI(TAG_BMI160, "BMI160 headerless FIFO mode configured successfully");
    return ESP_OK;
}


