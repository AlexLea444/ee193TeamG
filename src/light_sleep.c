#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "driver/uart.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#include "light_sleep.h"
 
#define TIMER_WAKEUP_TIME_US (10 * 1000 * 1000)

// static const char *TAG = "timer_wakeup";

esp_err_t set_timer_wakeup(void) {
    ESP_RETURN_ON_ERROR(esp_sleep_enable_timer_wakeup(TIMER_WAKEUP_TIME_US), "light sleep", "Configure timer as wakeup source failed");
    ESP_LOGI("light sleep", "timer wakeup source is ready");
    return ESP_OK;
}

static void light_sleep(void *args) {
        while (true) {
                ESP_LOGI("light sleep", "Entering light sleep");
                /* To make sure the complete line is printed before entering 
                 * sleep mode, need to wait until UART TX FIFO is empty:
                 */
                uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);

                /* Enter sleep mode */
                esp_light_sleep_start();

                 /* Get timestamp after waking up from sleep */
                int64_t t_after_us = esp_timer_get_time();

                /* Determine wake up reason */
                const char* wakeup_reason;
                switch (esp_sleep_get_wakeup_cause()) {
                        case ESP_SLEEP_WAKEUP_TIMER:
                                wakeup_reason = "timer";
                                break;
                        case ESP_SLEEP_WAKEUP_UART:
                                wakeup_reason = "uart";
                                /* Hang-up for a while to switch and execuse the uart task
                                 * Otherwise the chip may fall sleep again before running uart task */
                                vTaskDelay(1);
                                break;
                        default:
                                wakeup_reason = "other";
                                break;
                }
        }
        vTaskDelete(NULL);
}
