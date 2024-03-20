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

#include "common.h"
#include "i2c.h"
#include "light_sleep.h"

// SemaphoreHandle_t print_mux = NULL;



/* TODO: Check Course Website for Sample ADC Reading Code
static void therm_adc_read() {
        REG_WRITE(APB_SARADC_ONETIME_SAMPLE_REG, )        
}*/

static const char *TAG = "main";
                                
void app_main() {
        // Setup WiFi/MQTT
        /*esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
                ESP_ERROR_CHECK(nvs_flash_erase());
                ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        ESP_LOGI(TAG, "Connecting to WiFi...");
        wifi_connect(WIFI_SSID, WIFI_PASS);

        esp_mqtt_client_config_t mqtt_cfg = {
                .broker.address.uri = BROKER_URI,
        };
        esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
        esp_mqtt_client_start(client);

        ESP_LOGI(TAG, "Sending MQTT message in 5 seconds...");
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        esp_mqtt_client_publish(client, "alea01/")

        vTaskDelay(1000 / portTICK_PERIOD_MS);*/
        ESP_ERROR_CHECK(i2c_master_init());
        
        for (;;) {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                ESP_ERROR_CHECK(set_timer_wakeup());
                light_sleep("light_sleep_task");
                i2c_test_task("i2c_test_task");
        }
}
