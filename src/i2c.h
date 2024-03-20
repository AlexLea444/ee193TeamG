#ifndef __I2C_H__
#define __I2C_H__

#include "driver/gpio.h"
#include "driver/i2c.h"

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

static esp_err_t ic_read_register(i2c_port_t i2c_num, uint8_t *user_reg);
static esp_err_t ic_update_res(i2c_port_t i2c_num, uint8_t resolution);
static esp_err_t ic_probe_temp(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l); 
static esp_err_t i2c_master_init(void);
static void i2c_test_task(void *arg);

#endif
