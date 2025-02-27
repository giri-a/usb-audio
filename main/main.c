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
#include "esp_log.h"
#include "i2s_functions.h"
#include "data_buffers.h"
#include "uad_callbacks.h"
#include "driver/gpio.h"
#include "blink.h"

static const char *TAG = "main";

/* entities defined in uad_callbacks.c */
extern uint32_t sampFreq;       // current sampling frequency (reqd in I2S programming)
extern uint8_t clkValid;
extern uint32_t sampleRatesList[]; // sampFreq is one of the vales in this array

extern uint32_t blink_state;

#define I2S_DATA_IN_BUFSIZ (CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX/ 2)
#define I2S_DATA_OUT_BUFSIZ (CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ/ 2)

// variables declared as extern in data_buffers.h

// Current resolution, update on format change
// number of bytes to be read from I2S every ms

/* data_in_buf is to temporarily store incoming data from I2S (mic).
   It is populated by I2S read function and read by the USB device callback function*/
/* tud_audio_tx_done_pre_cb() copies this data to the endpoint buffer */
size_t  data_in_buf_cnt  = 0;      // value is set based on sample rate etc. when the i2s is configured 
int16_t data_in_buf[I2S_DATA_IN_BUFSIZ] = {0};

volatile size_t data_out_buf_cnt = 0;
int16_t data_out_buf[I2S_DATA_OUT_BUFSIZ] = {0};

// end extern variables declared in data_buffers.h

TaskHandle_t spk_task_handle = NULL; 
TaskHandle_t usb_device_task_handle = NULL; 

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
void print_task_stats();
void app_main()
{
    BaseType_t ret_val ;

    /* USB out => I2S Speakers; USB in <= I2S Mic */
    usb_get_data = &bsp_i2s_read;
    i2s_get_data = &tud_audio_read;

    /* Use the following for USB loopback and I2S loopback. Comment out the two lines above.*/
    /* I2S Mic => I2S Speakers ; USB out => USB in 
    usb_get_data = &uad_processed_data;
    i2s_get_data = &bsp_i2s_read;
    */

    // ESP_LOGI(TAG, "I2S_DATA_IN_BUFSIZ: %d, I2S_DATA_OUT_BUFSIZ: %d", I2S_DATA_IN_BUFSIZ, I2S_DATA_OUT_BUFSIZ);

    init_gpio();

    sampFreq = sampleRatesList[0];
    clkValid = 1;

    ESP_ERROR_CHECK(bsp_i2s_init(I2S_NUM_1, sampFreq));

    // Create a task for tinyusb device stack

    //(void) xTaskCreateStatic( usb_device_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, usb_device_stack, &usb_device_taskdef);
    //ESP_ERROR_CHECK(usb_headset_init());
    usb_headset_init();

    /* This is for debug. We allocate memory for 12 tasks even before the tasks have been
       created to avoid a timing issue. I think in this program there will be 
       IDLE0, IDLE0, esp_timer, timer_svc, ipc0, ipc1, main plus two tasks created below.
    */
    // allocate_memory_for_status();

    ret_val = xTaskCreatePinnedToCore(usb_device_task, "usb_device_task", 3 * 1024, NULL, 2, &usb_device_task_handle,0);
    if (ret_val != pdPASS) {
        ESP_LOGE(TAG, "Failed to create usb_task");
        //return ESP_FAIL;
        return;
    }
    ESP_LOGI(TAG, "TinyUSB initialized");

    ret_val = xTaskCreatePinnedToCore(i2s_consumer_func, "i2s_consumer_func", 4*1024, (void*)& s_spk_active, 1, &spk_task_handle, 1);
    if (ret_val != pdPASS) {
        ESP_LOGE(TAG, "Failed to create i2s_read_write_task");
        //return ESP_FAIL;
        return;
    }
    ESP_LOGI(TAG, "I2S started");

    configure_led();

    blink_state = BLINK_NOT_MOUNTED;

    // print_task_list();

    while(1)
    {
        //int bytes_left = uxTaskGetStackHighWaterMark(usb_device_task_handle);

        // led_blinking_task();
        drive_led();
         vTaskDelay(pdMS_TO_TICKS(50));
    } 
}