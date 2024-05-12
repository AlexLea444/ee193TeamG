/* Code heavily borrowed from ESP-IDF Example, 
 * modified for use of SI7054-A20-IM:
 * https://github.com/espressif/esp-idf/blob/v4.3/examples/peripherals/i2c/
 *      i2c_self_test/main/i2c_example_main.c#L36
 */

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <stdbool.h>

#include "i2c.h"

#include "common.h"

static const char *TAG = "i2c";

// SemaphoreHandle_t print_mux = NULL;

esp_err_t ic_read_register(i2c_port_t i2c_num, uint8_t *user_reg) {
        int ret;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, IC_ADDRESS << 1 | IC_WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, IC_CMD_R_UR1, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) {
                return ret;
        }
        vTaskDelay(30 / portTICK_PERIOD_MS);
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, IC_ADDRESS << 1 | IC_READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, user_reg, NACK_VAL);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        return ret;
}

/* 
 * Resolution options:
 * 00 - 14 bit Measurement Resolution
 * 01 - 12 bit Measurement Resolution
 * 10 - 13 bit Measurement Resolution
 * 11 - 11 bit Measurement Resolution
 *
 * NOTE: User Register is first read so reserved bits are not altered.
 *       This is not required by the device, but recommended "for future 
 *       compatibility".
 *
 */
esp_err_t ic_update_res(i2c_port_t i2c_num, uint8_t resolution) {
        int ret;
        uint8_t user_reg;
        uint8_t res_write = ((resolution & 0x2)<<6) + (resolution & 0x1);
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();

        ret = ic_read_register(i2c_num, &user_reg);
        if (ret != ESP_OK) {
                return ret;
        }
        
        vTaskDelay(30 / portTICK_PERIOD_MS);
        res_write = res_write ^ (user_reg & 0x7E);

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, IC_ADDRESS << 1 | IC_WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, IC_CMD_W_UR1, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, res_write, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        vTaskDelay(30 / portTICK_PERIOD_MS);

        //validate successful write
        ret = ic_read_register(i2c_num, &user_reg);
        if ((user_reg>>7 != res_write>>7) || (user_reg<<7 != res_write<<7))
                return ESP_FAIL;
        
        return ret;
}

esp_err_t ic_probe_temp(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l) {
        int ret;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, IC_ADDRESS << 1 | IC_WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, IC_CMD_MEASURE_H, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) {
                return ret;
        }
        vTaskDelay(30 / portTICK_PERIOD_MS);
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, IC_ADDRESS << 1 | IC_READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, data_h, ACK_VAL);
        i2c_master_read_byte(cmd, data_l, NACK_VAL);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        return ret;
}

esp_err_t i2c_master_init(void) {
        int i2c_master_port = I2C_MASTER_NUM;
        i2c_config_t conf = {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = I2C_MASTER_SDA_IO,
                .sda_pullup_en = GPIO_PULLUP_DISABLE,
                .scl_io_num = I2C_MASTER_SCL_IO,
                .scl_pullup_en = GPIO_PULLUP_DISABLE,
                .master.clk_speed = I2C_MASTER_FREQ_HZ,
        };

        esp_err_t err = i2c_param_config(i2c_master_port, &conf);
        if (err != ESP_OK) {
                return err;
        }

        return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void scan_temperature(char *temp_result) {
        int ret;
        uint8_t sensor_data_h, sensor_data_l, user_register;
        float temp;
        
        // Check that VDD of IC is not set too low
        ret = ic_read_register(I2C_MASTER_NUM, &user_register);
        if (ret == ESP_ERR_TIMEOUT) {
                ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
                printf("*******************\n");
                printf("HOST CHECK VDD( SI7054 )\n");
                printf("*******************\n");
                if (user_register & 0x40)
                        ESP_LOGE(TAG, "VDD Low");
                else
                        ESP_LOGI(TAG, "VDD OK");
        }
        vTaskDelay(DELAY_TIME_BETWEEN_ITEMS_MS / portTICK_PERIOD_MS);

        // Update the resolution of the IC output to desired
        ret = ic_update_res(I2C_MASTER_NUM, 0x3);
        if (ret == ESP_ERR_TIMEOUT) {
                ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
                printf("*******************\n");
                printf("HOST UPDATE RES( SI7054 )\n");
                printf("*******************\n");
        }
        vTaskDelay(DELAY_TIME_BETWEEN_ITEMS_MS / portTICK_PERIOD_MS);
        
        ret = ic_probe_temp(I2C_MASTER_NUM, &sensor_data_h, &sensor_data_l);
        if (ret == ESP_ERR_TIMEOUT) {
                ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
                printf("*******************\n");
                printf("HOST READ SENSOR( SI7054 )\n");
                printf("*******************\n");
                // printf("data_h: %02x\n", sensor_data_h);
                // printf("data_l: %02x\n", sensor_data_l);
                temp = (sensor_data_h << 8 | sensor_data_l) * 175.72 / 65536 - 46.85;
                printf("Temperature (°C): %.02f [Lux]\n", temp);
                sprintf(temp_result, "%.04f", temp);
        } else {
                ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }
        vTaskDelay(DELAY_TIME_BETWEEN_ITEMS_MS / portTICK_PERIOD_MS);
        //---------------------------------------------------
}

float get_temp()
{
        int ret;
        uint8_t sensor_data_h, sensor_data_l, user_register;
        
        ret = ic_probe_temp(I2C_MASTER_NUM, &sensor_data_h, &sensor_data_l);
        if (ret == ESP_ERR_TIMEOUT) {
                ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
                return (sensor_data_h << 8 | sensor_data_l) * 175.72 / 65536 - 46.85;
        } else {
                // Error detected
                return 1000.0;
        }
}
