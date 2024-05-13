#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#define I2C_PRIMARY_SCL_IO 33
#define I2C_PRIMARY_SDA_IO 32
#define I2C_PRIMARY_FREQUENCY_HZ 400000
#define I2C_PRIMARY_TX_BUFFER_DISABLE 0
#define I2C_PRIMARY_RX_BUFFER_DISABLE 0

#define BH1750_I2C_ADDRESS 0x23

#define BH1750_CONTINUOUS_HIGH_RES_MODE 0x10

#define ACK_CHECK_ENABLE 0x1
#define ACK_CHECK_DISABLE 0x0
#define ACK_VALUE 0x0
#define NACK_VALUE 0x1

#define TAG "MAIN"

static uint8_t mode = BH1750_CONTINUOUS_HIGH_RES_MODE;

esp_err_t i2c_primary_init();
static esp_err_t bh1750_primary_write_secondary(i2c_port_t, uint8_t*, size_t);
static esp_err_t bh1750_primary_read_secondary(i2c_port_t, uint8_t*, size_t);

int app_main() {
    ESP_ERROR_CHECK(i2c_primary_init());

    while (true) {
        uint16_t raw_lux;
        float lux;
        uint8_t sleep_ms = 120;
        esp_err_t ret;

        uint8_t write_buffer[1] = {mode};
        ret = i2c_master_write_to_device(I2C_NUM_0, BH1750_I2C_ADDRESS, write_buffer, sizeof(write_buffer), 1000 / portTICK_PERIOD_MS);
        if (ret != ESP_OK) {
            return -1;
        }
        vTaskDelay(sleep_ms / portTICK_PERIOD_MS);

        uint8_t data_rd[2] = {0};
        ret = bh1750_primary_read_secondary(I2C_NUM_0, data_rd, 2);
        if (ret != ESP_OK) {
            return -1;
        }
        raw_lux = ((uint16_t)data_rd[0] << 8 | data_rd[1]);
        lux = raw_lux / 1.2;
        ESP_LOGI(TAG, "Illuminance: %.1f lx", lux);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    return 0;
}

esp_err_t i2c_primary_init() {
    esp_err_t error;

    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_PRIMARY_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_PRIMARY_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_PRIMARY_FREQUENCY_HZ
    };

    error = i2c_param_config(I2C_NUM_0, &config);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver configuration failed with error = %d", error);
    }

    error = i2c_driver_install(I2C_NUM_0, config.mode, I2C_PRIMARY_RX_BUFFER_DISABLE, I2C_PRIMARY_TX_BUFFER_DISABLE, 0);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed with error = %d", error);
    }

    uint8_t reg = 0x00;
    error = bh1750_primary_write_secondary(I2C_NUM_0, &reg, 1);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "BH1750 sensor not found at 0x%02x", BH1750_I2C_ADDRESS);
    } else {
        ESP_LOGI(TAG, "BH1750 sensor found at 0x%02x", BH1750_I2C_ADDRESS);
    }

    return ESP_OK;
}

static esp_err_t bh1750_primary_write_secondary(i2c_port_t i2c_num, uint8_t* data_wr, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_ENABLE);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_ENABLE);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t bh1750_primary_read_secondary(i2c_port_t i2c_num, uint8_t* data_rd, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( BH1750_I2C_ADDRESS << 1 ) | I2C_MASTER_READ, ACK_CHECK_ENABLE);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VALUE);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VALUE);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
