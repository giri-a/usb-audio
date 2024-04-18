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

void configure_led(void);
void led_blinking_task(void);

/* entities defined in uad_callbacks.c */
extern uint32_t sampFreq;       // current sampling frequency (reqd in I2S programming)
extern uint8_t clkValid;
extern uint32_t sampleRatesList[]; // sampFreq is one of the vales in this array


//static ringbuf_handle_t rb_debug = NULL;

#define USBD_STACK_SIZE     4096
StackType_t  usb_device_stack[USBD_STACK_SIZE];
StaticTask_t usb_device_taskdef;

/*
// for i2s_read task
#define I2S_STACK_SIZE     4096
StackType_t  i2s_device_stack[I2S_STACK_SIZE];
StaticTask_t i2s_device_taskdef;

#ifdef DUMMY_I2S
StackType_t  i2s_dummy_device_stack[I2S_STACK_SIZE];
StaticTask_t i2s_dummy_device_taskdef;
#endif
static const char *TAG = "main";
*/

/*
typedef struct {
    int64_t delta_time;
    int     n_bytes;
} data_read_times_t;
data_read_times_t delta_times[100];
*/

static usb_phy_handle_t phy_hdl;
static void usb_phy_init(void)
{
    // Configure USB PHY
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .otg_mode = USB_OTG_MODE_DEVICE,
    };
    phy_conf.target = USB_PHY_TARGET_INT;
    usb_new_phy(&phy_conf, &phy_hdl);
}
// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
void usb_device_task(void *param)
{
    (void) param;

    // This should be called after scheduler/kernel is started.
    // Otherwise it could cause kernel issue since USB IRQ handler does use RTOS queue API.
    tusb_init();

    // RTOS forever loop
    while (1) {
        // tinyusb device task
        tud_task();
    }
    vTaskDelete(NULL);
}


void app_main()
{
    usb_phy_init();

    sampFreq = sampleRatesList[0];
    clkValid = 1;

    ESP_ERROR_CHECK(bsp_i2s_init(I2S_NUM_1, sampFreq));

    // Create a task for tinyusb device stack

    (void) xTaskCreateStatic( usb_device_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, usb_device_stack, &usb_device_taskdef);

    configure_led();

    while(1)
    {
         led_blinking_task();
         vTaskDelay(pdMS_TO_TICKS(50));
    } 
}
