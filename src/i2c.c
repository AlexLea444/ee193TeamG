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


/*
 * Important values for SI7054-A20-IM
 * https://www.silabs.com/documents/public/data-sheets/Si7050-1-3-4-5-A20-1.pdf
 */
#define IC_ADDRESS 0x40
#define IC_WRITE_BIT 0x0        // Inverted from Data Sheet
#define IC_READ_BIT 0x1         // Inverted from Data Sheet
#define IC_CMD_MEASURE_H 0xE3   // Measure Temperature, Hold Master Mode
#define IC_CMD_MEASURE_NH 0xF3  // Measure Temperature, No Hold Master Mode
#define IC_CMD_RESET 0xFE
#define IC_CMD_W_UR1 0xE6       // Write User Register 1
#define IC_CMD_R_UR1 0xE7       // Read User Register 1
#define IC_UR1_VDD 0x40         // Mask to read VDD Status of User Register 1
#define IC_UR1_MR_14 0x81       // Mask to write Resolution to User Register 1
                                // 0bX000_000X
                                // 00 - 14 bit Measurement Resolution
                                // 01 - 12 bit Measurement Resolution
                                // 10 - 13 bit Measurement Resolution
                                // 11 - 11 bit Measurement Resolution

#define I2C_MASTER_SDA_IO 2
#define I2C_MASTER_SCL_IO 4
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_NUM 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TX_BUF_DISABLE 0

#define ACK_CHECK_EN 0x1    // Enable Host to check ACK from Client
#define ACK_VAL 0x0         // TODO: Check
#define NACK_VAL 0x1        // TODO: Check

#define DELAY_TIME_BETWEEN_ITEMS_MS 1000

// static const char* TAG = "i2c";

// SemaphoreHandle_t print_mux = NULL;

static esp_err_t ic_read_register(i2c_port_t i2c_num, uint8_t *user_reg) {
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
static esp_err_t ic_update_res(i2c_port_t i2c_num, uint8_t resolution) {
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

static esp_err_t ic_probe_temp(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l) {
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

static esp_err_t i2c_master_init(void) {
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

static void i2c_test_task(void *arg) {
        int ret;
        uint32_t task_idx = (uint32_t)arg;
        uint8_t sensor_data_h, sensor_data_l, user_register;
        int cnt = 0;
        float temp;
        
        // Check that VDD of IC is not set too low
        ESP_LOGI("i2c", "TASK[%d] test cnt: %d", (int)task_idx, cnt++);
        ret = ic_read_register(I2C_MASTER_NUM, &user_register);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        if (ret == ESP_ERR_TIMEOUT) {
                ESP_LOGE("i2c", "I2C Timeout");
        } else if (ret == ESP_OK) {
                printf("*******************\n");
                printf("TASK[%d]  HOST CHECK VDD( SI7054 )\n", (int)task_idx);
                printf("*******************\n");
                if (user_register & 0x40)
                         printf("WARNING: VDD LOW\n");
                else
                        printf("VDD OK\n");
        }
        xSemaphoreGive(print_mux);
        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_PERIOD_MS);

        // Update the resolution of the IC output to desired
        ret = ic_update_res(I2C_MASTER_NUM, 0x3);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        if (ret == ESP_ERR_TIMEOUT) {
                ESP_LOGE("i2c", "I2C Timeout");
        } else if (ret == ESP_OK) {
                printf("*******************\n");
                printf("TASK[%d]  HOST UPDATE RES( SI7054 )\n", (int)task_idx);
                printf("*******************\n");
        }
        xSemaphoreGive(print_mux);
        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_PERIOD_MS);
        
        // Continuously read out temperature readings
        while (1) {
                ESP_LOGI("i2c", "TASK[%d] test cnt: %d", (int)task_idx, cnt++);
                ret = ic_probe_temp(I2C_MASTER_NUM, &sensor_data_h, &sensor_data_l);
                xSemaphoreTake(print_mux, portMAX_DELAY);
                if (ret == ESP_ERR_TIMEOUT) {
                        ESP_LOGE("i2c", "I2C Timeout");
                } else if (ret == ESP_OK) {
                        printf("*******************\n");
                        printf("TASK[%d]  HOST READ SENSOR( SI7054 )\n", (int)task_idx);
                        printf("*******************\n");
                        // printf("data_h: %02x\n", sensor_data_h);
                        // printf("data_l: %02x\n", sensor_data_l);
                        temp = (sensor_data_h << 8 | sensor_data_l) * 175.72 / 65536 - 46.85;
                        printf("Temperature (°C): %.02f [Lux]\n", temp);
                } else {
                        ESP_LOGW("i2c", "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
                }
                xSemaphoreGive(print_mux);
                vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_PERIOD_MS);
                //---------------------------------------------------
        }
        vSemaphoreDelete(print_mux);
        vTaskDelete(NULL);
}

