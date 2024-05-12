#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // Used for timer delay
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "minimal_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_sleep.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "Reading.h"


static const char *TAG = "MAIN";

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void log_error_if_nonzero(const char *message, int error_code);
int get_time();
int32_t get_temp();
esp_mqtt_client_handle_t setup_mqtt_and_wifi();
void send_all_readings(nvs_handle_t nvs_handle);
void take_reading(nvs_handle_t nvs_handle);

// static const char TAG[] = "main";
#define WIFI_SSID      "tufts_eecs"
#define WIFI_PASS      "foundedin1883"

#define BROKER_URI "mqtt://en1-pi.eecs.tufts.edu"
int current_time = -1;

void app_main() {
  ESP_LOGI(TAG,"I have arisen");
  //   // Enable Flash (aka non-volatile storage, NVS)
  esp_err_t err = nvs_flash_init();
  ESP_ERROR_CHECK(err);
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGI(TAG, "Error attempting to setup flash");
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  nvs_handle_t my_handle;
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  int32_t reading_counter = -1000;
  ESP_ERROR_CHECK(err);
  switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(TAG,"Woke up from timer");
            err = nvs_get_i32(my_handle, "reading_counter", &reading_counter);
            if(reading_counter >= 9)
            {
              ESP_LOGI(TAG, "at max sending all readings");
              send_all_readings(my_handle);
            }
            else ESP_LOGI(TAG, "not at the max taking normal readings");
            take_reading(my_handle);
            nvs_close(my_handle);
            vTaskDelay(3000/portTICK_PERIOD_MS);
            esp_deep_sleep(10000000);
            break;
        default:
            ESP_LOGI(TAG,"Work up because this is the first time ever.");
            vTaskDelay(500/portTICK_PERIOD_MS);
            reading_counter = 0;
            err = nvs_set_i32(my_handle, "reading_counter", reading_counter);
            err = nvs_commit(my_handle);
            take_reading(my_handle);
            ESP_LOGI(TAG,"closing NVS.");
            nvs_close(my_handle);
            esp_sleep_enable_timer_wakeup(10000000);
            ESP_LOGI(TAG,"sleeping");
            esp_deep_sleep_start();
            break;
    }
    printf("Done!\n");
}

// get time waits for the time to be updated by the event handler loop. 
// which runs in parallel. 
int get_time()
{
  ESP_LOGI(TAG, "getting the time");
  while(current_time == -1)
  {
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
  return current_time;
}

// event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        char *expected_topic = "time";
        int index = 0;
        while(index < event->topic_len && index < 4 && expected_topic[index] == event->topic[index]) index++;
        if(index != 4) break;
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        current_time = atoi(event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

// logs and error if the error is not zero
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}
// calls take reading and saves it to memory
void take_reading(nvs_handle_t nvs_handle)
{
  ESP_LOGI(TAG, "taking a reading");
  // take a reading 
  int32_t reading_counter = -1;
  int32_t reading = get_temp();
  // get the reading number
  ESP_LOGI(TAG, "getting reading counter");
  ESP_ERROR_CHECK(nvs_get_i32(nvs_handle, "reading_counter", &reading_counter));
  ESP_LOGI(TAG, "committing");
  ESP_ERROR_CHECK(nvs_commit(nvs_handle));
  ESP_LOGI(TAG, "committed");
  char reading_key[10];
  sprintf(reading_key, "reading%li", reading_counter);
  ESP_LOGI(TAG, "Reading key is: %s", reading_key);
  ESP_LOGI(TAG, "Creating reading key");
  // save it to memory
  ESP_LOGI(TAG, "saving the data to memory. ");
  nvs_set_i32(nvs_handle, reading_key, reading);
  reading_counter++;
  // udpate reading counter
  nvs_set_i32(nvs_handle, "reading_counter", reading_counter);
  nvs_commit(nvs_handle);
  ESP_LOGI(TAG, "data has been saved to memory");
}

// takes the reading from the sensor
int32_t get_temp()
{
  // update later with actual code.
  ESP_LOGI(TAG, "attempting to get temperature");
  vTaskDelay(1000/portTICK_PERIOD_MS);
  ESP_LOGI(TAG, "returning temperature");
  return 1000;
}
void send_all_readings(nvs_handle_t nvs_handle)
{
  ESP_LOGI(TAG, "send all readings has been called "); 
  esp_mqtt_client_handle_t client = setup_mqtt_and_wifi();
  int curr_time = get_time();
  Reading readings[10];
  for(int i = 0; i < 10; i++)
  {
    ESP_LOGI(TAG, "getting reading %i", i); 
    char reading_key[10];
    sprintf(reading_key, "reading%i", i);

    nvs_set_i32(nvs_handle, reading_key, &readings[i].temp);
    ESP_LOGI(TAG, "reading was: %li", readings[i].temp);
    // the readings are ordered in reverse order of recent
    // to get the estiamted time take the current time -9 and multiply it by 30 minutes. 
    readings[i].time = curr_time - (9-i)*1800;
    readings[i].battery_level = -1;
  }
  // we will always send readings in groups of 10
  for(int i = 0; i < 10; i++)
  {
    ESP_LOGI(TAG, "sending reading %i to client", i); 
    Reading curr_reading = readings[i];
    char *data_to_send = sprintf("%i,%i,%i", curr_reading.time, curr_reading.temp, curr_reading.battery_level);
    esp_mqtt_client_publish(client, "teamG/node1/tempupdate", data_to_send, 0, 0, 1);
  }
  esp_mqtt_client_disconnect(client);
  ESP_LOGI(TAG, "sending reading is done"); 
}


esp_mqtt_client_handle_t setup_mqtt_and_wifi()
{
  wifi_connect(WIFI_SSID, WIFI_PASS);

    // Initialize the MQTT client
    // Read the documentation for more information on what you can configure:
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/protocols/mqtt.html
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = BROKER_URI,
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    esp_mqtt_client_start(client);
    return client;
}
