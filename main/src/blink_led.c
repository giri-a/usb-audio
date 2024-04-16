/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "blink";

uint32_t micros()
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    return tv_now.tv_sec * 1000000L + tv_now.tv_usec;
}
uint32_t millis()
{
    return micros()/1000;
}
/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

extern uint32_t blink_interval_ms;
static uint8_t s_led_state = 0;

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        /* DONT KNOW WHY: on my Waveshare ESP32-S3-Pico, the R and G seems to be interchanged!!!*/
        /* So the colors are like below!*/
        switch(s_led_state){                        //G   R  B
            case 1: led_strip_set_pixel(led_strip,0, 16, 16, 16); 
                    //ESP_LOGI(TAG, "Turning the LED WHITE!");
                    break;
            case 2: led_strip_set_pixel(led_strip,0,  0, 16,  0); 
                    //ESP_LOGI(TAG, "Turning the LED RED!");
                    break;
            case 3: led_strip_set_pixel(led_strip,0,  0, 16, 16);
                    //ESP_LOGI(TAG, "Turning the LED PURPLE!");
                    break;
            case 4: led_strip_set_pixel(led_strip,0, 16,  0,  0);
                    //ESP_LOGI(TAG, "Turning the LED GREEN!");
                    break;
            case 5: led_strip_set_pixel(led_strip,0, 16,  0, 16);
                    //ESP_LOGI(TAG, "Turning the LED CYAN!");
                    break;
            case 6: led_strip_set_pixel(led_strip,0, 16, 16,  0);
                    //ESP_LOGI(TAG, "Turning the LED YELLOW!");
                    break;
            case 7: led_strip_set_pixel(led_strip,0, 0, 0, 16);
                    //ESP_LOGI(TAG, "Turning the LED BLUE!");
                    break;
        }
        /*led_strip_set_pixel(led_strip, 0, 16, 16, 16);*/
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
        //ESP_LOGI(TAG, "Turning the LED OFF!");
    }
}

void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

/*
void app_main(void)
{

    configure_led();

    while (1) {
        //ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == false ? "OFF" : "ON");
        blink_led();
        // Toggle the LED state 
        //s_led_state = !s_led_state;
        s_led_state = s_led_state + 1;
        if(s_led_state == 8)
            s_led_state = 0;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
*/

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;

  // Blink every interval ms
  if ( millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  blink_led();
  s_led_state = 1 - s_led_state; // toggle
}