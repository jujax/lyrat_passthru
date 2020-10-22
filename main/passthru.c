/*

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "audio_pipeline.h"
#include "i2s_stream.h"
#include "board.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_a2dp_api.h"
#include "esp_peripherals.h"
#include "nvs_flash.h"
#include "audio_element.h"
#include "audio_event_iface.h"
#include "input_key_service.h"
#include "periph_touch.h"
#include "a2dp_stream.h"
#include "wav_encoder.h"
#include "freertos/timers.h"
#include "sdkconfig.h"
#include "audio_mem.h"
#include "audio_common.h"
#include "periph_button.h"
#include "periph_adc_button.h"
#include "periph_touch.h"
#include "bluetooth_service.h"
#include <stdio.h>
#include "esp_system.h"
#include "nvs.h"
#include "equalizer.h"
#include "driver/ledc.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#define CONFIG_BT_REMOTE_NAME "WH-1000XM4"

#define BT_CONNECT_TIMEOUT 20000

typedef uint8_t esp_peer_bdname_t[ESP_BT_GAP_MAX_BDNAME_LEN + 1];

static esp_peer_bdname_t remote_bt_device_name;
static esp_bd_addr_t remote_bd_addr = {0};

static const char *TAG = "PASSTHRU";

static audio_pipeline_handle_t pipeline;
static audio_element_handle_t bt_stream_writer, i2s_stream_writer, bt_stream_reader, i2s_stream_reader;

static bool output_is_bt = false;
static nvs_handle_t my_handle;
static esp_err_t err;

static ledc_channel_config_t ledc_channel = {
    .channel = LEDC_CHANNEL_0,
    .duty = 0,
    .gpio_num = 22,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .hpoint = 0,
    .timer_sel = LEDC_TIMER_1
};
static ledc_timer_config_t ledc_timer = {
    .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
    .freq_hz = 5000,                      // frequency of PWM signal
    .speed_mode = LEDC_LOW_SPEED_MODE,    // timer mode
    .timer_num = LEDC_TIMER_1,            // timer index
    .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
};

#include "bluetooth_output.h"
#include "line_output.h"

void app_main(void)
{

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        printf("Done\n");

        // Read
        int32_t mode = 0; // value will default to 0, if not set yet in NVS
        err = nvs_get_i32(my_handle, "mode", &mode);
        switch (err)
        {
        case ESP_OK:
            printf("Done\n");
            printf("Changing output to %s", mode ? "Bluetooth" : "Line");
            output_is_bt = mode;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
    }

    ledc_timer_config(&ledc_timer);

    ledc_channel_config(&ledc_channel);

    if (output_is_bt)
        bluetooth_output();
    else
        line_out();

    nvs_close(my_handle);
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}