/*
 * Adapted from skainet code : 
 * https://github.com/espressif/esp-skainet/tree/master/examples/usb_mic_recorder
 * discussed in https://github.com/espressif/esp-idf/issues/12774 
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tusb.h"
#include "tusb_config.h"
#include "esp_private/usb_phy.h"
#include "esp_err.h"
#include "i2s_functions.h"
#include "driver/gpio.h"
#include "blink.h"

/* entities defined in uad_callbacks.c */
extern uint32_t sampFreq;       // current sampling frequency (reqd in I2S programming)
extern uint8_t clkValid;
extern uint32_t sampleRatesList[]; // sampFreq is one of the vales in this array

extern uint32_t blink_state;

esp_err_t usb_headset_init(void);

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_NUM_7) |\
                              (1ULL<<GPIO_NUM_8) |\
                              (1ULL<<GPIO_NUM_9) |\
                              (1ULL<<GPIO_NUM_10) )

void init_gpio()
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void app_main()
{
    init_gpio();

    sampFreq = sampleRatesList[0];
    clkValid = 1;

    ESP_ERROR_CHECK(bsp_i2s_init(I2S_NUM_1, sampFreq));

    // Create a task for tinyusb device stack

    //(void) xTaskCreateStatic( usb_device_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, usb_device_stack, &usb_device_taskdef);
    ESP_ERROR_CHECK(usb_headset_init());

    configure_led();

    blink_state = BLINK_NOT_MOUNTED;

    while(1)
    {
        // led_blinking_task();
        drive_led();
         vTaskDelay(pdMS_TO_TICKS(50));
    } 
}
