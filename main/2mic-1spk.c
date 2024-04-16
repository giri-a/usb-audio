/* SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tusb.h"
#include "tusb_config.h"
#include "esp_private/usb_phy.h"
#include "usb_descriptors.h"
#include "esp_err.h"
#include "sdkconfig.h"
//#include "ringbuf.h"
#include "esp_log.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "driver/i2s_std.h"
#else
#include "driver/i2s.h"
#endif
#include "driver/gpio.h"

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_STREAMING = 25,
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void configure_led(void);
void led_blinking_task(void);


//#define DUMMY_I2S 1

#ifdef DUMMY_I2S
#include <rom/ets_sys.h>
#include "trig_table.h"
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static i2s_chan_handle_t                tx_handle = NULL;        // I2S tx channel handler
static i2s_chan_handle_t                rx_handle = NULL;        // I2S rx channel handler
#endif

#define I2S_GPIO_WS      GPIO_NUM_36
#define I2S_GPIO_DIN     GPIO_NUM_37
#define I2S_GPIO_DOUT    GPIO_NUM_35
#define I2S_GPIO_BCLK    GPIO_NUM_38

/*raw buffer to read data from I2S dma buffers*/
static char rx_sample_buf [CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ];
static size_t  rx_sample_buflen = 0;// value is set based on sample rate etc. when the i2s is configured 

/* data_in_buf is populated in tud_audio_tx_done_post_load_cb() from the circular buffer */
/* tud_audio_tx_done_pre_cb() copies this data to the endpoint buffer */
static size_t  data_in_buf_cnt = 0;      // value is set based on sample rate etc. when the i2s is configured 
static uint16_t data_in_buf[CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX/ 2];

static uint16_t data_out_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 2];
// Speaker data size received in the last frame
static size_t data_out_buf_cnt = 0;
static char tx_sample_buf [CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ];


// Resolution per format
const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX,
                                                                        CFG_TUD_AUDIO_FUNC_1_FORMAT_2_RESOLUTION_RX};
// Current resolution, update on format change
uint8_t current_resolution;

//static ringbuf_handle_t rb_debug = NULL;

#define USBD_STACK_SIZE     4096
StackType_t  usb_device_stack[USBD_STACK_SIZE];
StaticTask_t usb_device_taskdef;

// for i2s_read task
#define I2S_STACK_SIZE     4096
StackType_t  i2s_device_stack[I2S_STACK_SIZE];
StaticTask_t i2s_device_taskdef;

#ifdef DUMMY_I2S
StackType_t  i2s_dummy_device_stack[I2S_STACK_SIZE];
StaticTask_t i2s_dummy_device_taskdef;
#endif
static const char *TAG = "main";

// Audio controls
// Current states
bool mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];                                      // +1 for master channel 0
uint16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];                                // +1 for master channel 0
uint32_t sampFreq;
uint8_t clkValid;

// Range states
// List of supported sample rates
static const uint32_t sampleRatesList[] =
{
    16000, 24000, 32000, 48000
};

#define N_sampleRates  TU_ARRAY_SIZE(sampleRatesList)

// Range states
audio_control_range_2_n_t(1) volumeRng[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];         // Volume range state
audio_control_range_4_n_t(1) sampleFreqRng;                                             // Sample frequency range state


static esp_err_t bsp_i2s_init(i2s_port_t i2s_num, uint32_t sample_rate)
{
    esp_err_t ret_val = ESP_OK;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    i2s_slot_mode_t channel_fmt = I2S_SLOT_MODE_STEREO;
    /*
    if (CHANNEL_NUM == 1) {
        channel_fmt = I2S_SLOT_MODE_MONO;
    } else if (CHANNEL_NUM == 2) {
        channel_fmt = I2S_SLOT_MODE_STEREO;
    } else {
        ESP_LOGE(TAG, "Unable to configure channel_format %d", CHANNEL_NUM);
        channel_fmt = I2S_SLOT_MODE_MONO;
    }
    */
    /*
    if (bits_per_chan != 32) {
        ESP_LOGE(TAG, "Unable to configure bits_per_chan %d", bits_per_chan);
        bits_per_chan = 32;
    }
    */
    // default chan_cfg : 
    // { .id = <i2s_num>, .role = <I2S_ROLE_MASTER>, .dma_desc_num = 6, .dma_frame_num = 240, .auto_clear = 0, .intr_priority = 0, }
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_num, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 2;
    // dma_frame_num is changed from dafult value of 240 to reduce latency
    chan_cfg.dma_frame_num = sample_rate/1000;  // number of frames in 1mS; cannot handle sample_rate like 44.1kHz
    //ret_val |= i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle);
    ret_val |= i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle);
    //i2s_std_config_t std_cfg = I2S_CONFIG_DEFAULT(sample_rate, channel_fmt, bits_per_chan);
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, channel_fmt),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,    // some codecs may require mclk signal, this example doesn't need it
            .bclk = I2S_GPIO_BCLK,
            .ws   = I2S_GPIO_WS,
            .dout = I2S_GPIO_DOUT,
            .din  = I2S_GPIO_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ret_val |= i2s_channel_init_std_mode(tx_handle, &std_cfg);
    ret_val |= i2s_channel_init_std_mode(rx_handle, &std_cfg);

    //  dma_desc_num (6) dma buffers of each dma_buffer_size = (dma_frame_num * slot_num * slot_bit_width / 8) bytes
    // read or write will block till a dma buffer i.e., dma_frame_num frames are available or transmitted

    /* raw_buffer is statically allocated for its max required size; 
       but its usable capacity is set here based on the current sample rate. 
       This is required to re-set whenever sampFreq changes.
    */
    rx_sample_buflen  = chan_cfg.dma_frame_num * std_cfg.slot_cfg.slot_mode * std_cfg.slot_cfg.data_bit_width / 8;
    data_in_buf_cnt   = chan_cfg.dma_frame_num * std_cfg.slot_cfg.slot_mode ;
    //printf("rx_sample_buflen: %d, data_in_buf_cnt: %d\n", rx_sample_buflen, data_in_buf_cnt);


    ret_val |= i2s_channel_enable(tx_handle);
    ret_val |= i2s_channel_enable(rx_handle);
#else

    i2s_channel_fmt_t channel_fmt = I2S_CHANNEL_FMT_RIGHT_LEFT;
    if (channel_format == 1) {
        channel_fmt = I2S_CHANNEL_FMT_ONLY_LEFT;
    } else if (channel_format == 2) {
        channel_fmt = I2S_CHANNEL_FMT_RIGHT_LEFT;
    } else {
        ESP_LOGE(TAG, "Unable to configure channel_format %d", channel_format);
        channel_format = 1;
        channel_fmt = I2S_CHANNEL_FMT_ONLY_LEFT;
    }

    if (bits_per_chan != 16 && bits_per_chan != 32) {
        ESP_LOGE(TAG, "Unable to configure bits_per_chan %d", bits_per_chan);
        bits_per_chan = 16;
    }
#define I2S_MCLK_MULTIPLE_DEFAULT 0

    i2s_config_t i2s_config = I2S_CONFIG_DEFAULT(sample_rate, channel_fmt, bits_per_chan);

    i2s_pin_config_t pin_config = {
        .bck_io_num = GPIO_NUM_38, //GPIO_I2S_SCLK,
        .ws_io_num = GPIO_NUM_36, //GPIO_I2S_LRCK,
        .data_out_num = GPIO_I2S_DOUT,
        .data_in_num = GPIO_NUM_37, //GPIO_I2S_SDIN,
        .mck_io_num = GPIO_I2S_MCLK,
    };

    ret_val |= i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
    ret_val |= i2s_set_pin(i2s_num, &pin_config);

#endif
    return ret_val;
}

esp_err_t bsp_i2s_reconfig(uint32_t sample_rate)
{
    esp_err_t ret_val = ESP_OK;
    esp_err_t ret_val2 ;
    ret_val |= i2s_channel_disable(rx_handle);
    ret_val |= i2s_channel_disable(tx_handle);
    ret_val |= i2s_del_channel(rx_handle);
    ret_val |= i2s_del_channel(tx_handle);
    ESP_ERROR_CHECK(ret_val2 = bsp_i2s_init(I2S_NUM_1, sample_rate));
    ret_val |= ret_val2;
    //const i2s_std_clk_config_t clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
    
    //ret_val |= i2s_channel_reconfig_std_clock(rx_handle, &clk_cfg);
    //ret_val |= i2s_channel_enable(rx_handle);
    
    return ret_val;
}

/* 
    Knowles SPH0645 I2S mic's data are MSB aligned 24 bits in a 32-bit word,
    whose lower 6 bits are always 0.
    Optional GAIN amount is considered in determining amount of right-shift
    on the raw data to extract 16 bits. 
    The data has signnficant offset, which is removed by a offset canceller filter.

    The function here hardcodes 2-ch 16bit configuration of final data.
*/


#define MIC_RESOLUTION      18
#define GAIN                0      // in bits i.e., 1 =>2, 2=>4, 3=>8 etc.

/*
 This filter is an implementation of a 1st-order filter described in 
 https://docs.xilinx.com/v/u/en-US/wp279

 32bit raw_sample has 18bits of actual sample in MSB aligned way.
 The processed samples are returned in the same variable but as LSB aligned value.
 
 For faster settling of offset value, FILTER_RESPONSE_MULTIPLIER may be increased.
 The equivalent time constant (RC) = SAMPLING_PERIOD*2^(32-MIC_RESOLUTION-FILTER_RESPONSE_MULTIPLIER)
 For example, for fs=16kHz, MIC_RESOLUTION=18, and FILTER_RESPONSE_MULTIPLIER=0, 
 the time constant is 2^14/16000 ~= 1 second. Note that the reduction is time constant
 is in power of 2 for every increase in FILTER_RESPONSE_MULTIPLIER.

*/
#define  FILTER_RESPONSE_MULTIPLIER 2
void offset_canceller(int32_t *left_sample_p, int32_t *right_sample_p)
{
    static int32_t l_offset = 0;
    static int32_t r_offset = 0;
    int32_t final_value;

    if(left_sample_p != NULL)
    {
        if((*left_sample_p >> (32-MIC_RESOLUTION))> 32767 || (*left_sample_p >> (32 - MIC_RESOLUTION))< -32768){
            printf("left sample clips before offset\n");
        }
        final_value = *left_sample_p - (l_offset & (int32_t)(-1 << (32-MIC_RESOLUTION)));
        *left_sample_p = (final_value >> (32-MIC_RESOLUTION));
        l_offset += (*left_sample_p) << FILTER_RESPONSE_MULTIPLIER;

        if(*left_sample_p > 32767 || *left_sample_p < -32768){
            printf("left sample clips\n");
        }
    }

    if(right_sample_p != NULL)
    {
        if((*right_sample_p >> (32-MIC_RESOLUTION))> 32767 || (*right_sample_p >> (32 - MIC_RESOLUTION))< -32768){
            printf("right sample clips before offset\n");
        }
        final_value = *right_sample_p - (r_offset & (int32_t)(-1 << (32-MIC_RESOLUTION)));
        *right_sample_p = (final_value >> (32-MIC_RESOLUTION));
        r_offset += (*right_sample_p) << FILTER_RESPONSE_MULTIPLIER;

        if(*right_sample_p > 32767 || *right_sample_p < -32768){
            printf("right sample clips\n");
        }
    }
}

#ifdef DUMMY_I2S

#define OFFSET 1000

/*
  This function feeds synthetic data (sinusoid) to the receive channel 
  bypassing actual I2S receiver.
*/

size_t bsp_i2s_read(uint16_t *data_buf, size_t count)
{
    static int tabl_idx = 0;
    int L = sin_tabl_len;
    int i ;
    int32_t val;
    static int64_t t_earlier = 0;
    struct timeval tv_now;
    //for(i=0; i< i2s_read_buflen; i+=4, tabl_idx++)
    for(i=0; i< count; i+=2, tabl_idx++)
    {
        // The sin_table has sin_tabl_len (L) number of equally spaced entries between 0 and pi/2.
        // The table is read 0,1,...(L-1),L,(L-1),..0,1,..(L-1),L,(L-1)...1 order for a full
        // cycle of sin curve. For latter half, the sign of the value is reversed.
        if(tabl_idx > 39) tabl_idx = 0;
             if (tabl_idx < L)        val =  sin_qtr[tabl_idx];
        else if (tabl_idx <(L-1)*2+1) val =  sin_qtr[(L-1)*2-tabl_idx];
        else if (tabl_idx <(L-1)*3+1) val = -sin_qtr[tabl_idx-(L-1)*2];
        else                          val = -sin_qtr[(L-1)*4-tabl_idx];

        val += OFFSET;
        val <<= (32-MIC_RESOLUTION);
        offset_canceller(&val, NULL);

        // left and right channels are being given the same values
        *(data_buf+i)   = val;
        *(data_buf+i+1) = val;
    }

    gettimeofday(&tv_now, NULL);
    int64_t t_now = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    if((t_now - t_earlier) < 1000) { 
        ets_delay_us(1000-(t_now-t_earlier));			//Stalls execution for #uS
        t_earlier += 1000;
    }
    else  // probably first time
        t_earlier = t_now ;  

    return count;
}

#else

/*
  This function is called by tud_audio_tx_done_post_load_cb(). 
  It reads 32bits raw data for both left and right channels from I2S DMA buffers, 
  gets upper 18bits LSB aligned. After offset cancellation, 16 lower bits are 
  returned in data_buf.
*/
size_t bsp_i2s_read(uint16_t *data_buf, size_t count)
{
    int32_t d_left, d_right;
    size_t  n_bytes;
    int i = 0;
    //unsigned ccount;

    while (i<count) {
        n_bytes = 0;
        if(i2s_channel_read(rx_handle,rx_sample_buf, rx_sample_buflen, &n_bytes, 200) == ESP_OK) {

            // i2s_channel_read is blocking; it is expected to block till it gets enough number of
            // bytes (e.g., 128 bytes for 16KHz sampling rate every ms). This should keep time.
            assert(rx_sample_buflen==n_bytes);
            int j = 0;
            while(j < n_bytes) {
                if((n_bytes - j) >= 8) {
                    memcpy(&d_left,(rx_sample_buf+j),4);
                    memcpy(&d_right,(rx_sample_buf+j+4),4);
                    offset_canceller(&d_left, &d_right);
                    d_left <<= GAIN; d_right <<= GAIN;
                    *(data_buf+i)   = d_left;
                    *(data_buf+i+1) = d_right;
                    i += 2;
                    j += 8;
                }
                else {
                    // we have a problem
                    printf("%u : i2s_read_buffer failed (only %d bytes available; expected 8)\r\n", xthal_get_ccount(),(n_bytes-j));
                    break;
                }
            }
        }
        else {
            printf("i2s_channel_read failed \r\n");
            break;
        }
    } 
    return i;
}
#endif

//static int16_t data_dump[8192];

/*
  This function is called by tud_audio_rx_done_pre_read_cb() providing the data read from
  EP_OUT buffer in data_buf. The number of bytes in the data_buf is count (so count/2 int16_t type).
  This function formats the data (16 bits to MSB aligned 32 bits etc..) and writes to the 
  I2S DMA buffer to be sent out over I2S.
*/
void bsp_i2s_write(uint16_t *data_buf, size_t count){
    static int total_items_to_print = 8192;
    int j=0;
    static int i = 0;
    size_t n_bytes = 0;
    //if(total_items_to_print ==0){
    //    for(i=0,j=0;i<8192;i++,j++) {
    //        printf("%d,",data_dump[i]); j++;
    //        if(j==10){
    //            printf("\n");
    //            j = 0;
    //        }
    //    }
    //    i = 0;
    //total_items_to_print = 8192;
    //}
    // convert 16 bits to 32 bits
    if (current_resolution == 16)
    {
      int16_t *src   = (int16_t*)data_buf;
      int16_t *limit = (int16_t*)data_buf + count / 2;
      int32_t *dst   = (int32_t*)tx_sample_buf;
      while (src < limit)
      {
    //    data_dump[i++] = *src;
    //    total_items_to_print--;
        int32_t data = (*src++) << (32-MIC_RESOLUTION-1); 
        *dst++ = data;
        if(/*single channel*/ true)
            // copy the same data to both channels. NOTE This will depend on how the speaker is wired to I2S bus
            *dst++ = data;
      }
      // total number of bytes in tx_sample_buf is count*4 since each 16bit sample in data_buf
      // made into a 32bit value and then replicated in both left and right channel.
      assert(i2s_channel_write(tx_handle,tx_sample_buf, count*4, &n_bytes, 200) == ESP_OK) ;
    }
    else if (current_resolution == 24)
    {
      int32_t *src   = (int16_t*)data_buf;
      int32_t *limit = (int16_t*)data_buf + count / 4;
      int32_t *dst   = (int32_t*)tx_sample_buf;
      while (src < limit)
      {
        *dst++ = *src++;
      }
      assert(i2s_channel_write(tx_handle,tx_sample_buf, count, &n_bytes, 200) == ESP_OK) ;
    }

}
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


//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

void audio_task(void)
{
    // Yet to be filled - e.g. put meas data into TX FIFOs etc.
    // asm("nop");
}
//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific set request received for an EP
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff)
{
    (void) rhport;
    (void) pBuff;
    TU_LOG2("%s called\r\n",__func__);
    // We do not support any set range requests here, only current value requests
    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t ep = TU_U16_LOW(p_request->wIndex);

    (void) channelNum; (void) ctrlSel; (void) ep;
    
    return false;     // Yet not implemented
}

// Invoked when audio class specific set request received for an interface
bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff)
{
    (void) rhport;
    (void) pBuff;

    TU_LOG2("%s called\r\n",__func__);
    // We do not support any set range requests here, only current value requests
    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t itf = TU_U16_LOW(p_request->wIndex);

    (void) channelNum; (void) ctrlSel; (void) itf;

    return false;     // Yet not implemented
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff)
{
    (void) rhport;

    TU_LOG2("%s called\r\n",__func__);
    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t itf = TU_U16_LOW(p_request->wIndex);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

    (void) itf;

    // We do not support any set range requests here, only current value requests
    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

    // If request is for our feature unit
    if ( entityID == UAC2_ENTITY_SPK_FEATURE_UNIT ) {
        switch ( ctrlSel ) {
        case AUDIO_FU_CTRL_MUTE:
            // Request uses format layout 1
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));

            mute[channelNum] = ((audio_control_cur_1_t *) pBuff)->bCur;

            TU_LOG2("    Set Mute: %d of channel: %u\r\n", mute[channelNum], channelNum);
            return true;

        case AUDIO_FU_CTRL_VOLUME:
            // Request uses format layout 2
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));

            volume[channelNum] = ((audio_control_cur_2_t *) pBuff)->bCur;

            TU_LOG2("    Set Volume: %d dB of channel: %u\r\n", volume[channelNum], channelNum);
            return true;

        // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }
    // Clock Source unit
    if ( entityID == UAC2_ENTITY_CLOCK )
    {
        switch ( ctrlSel )
        {
            case AUDIO_CS_CTRL_SAM_FREQ:
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_4_t));

            sampFreq = (uint32_t)((audio_control_cur_4_t *)pBuff)->bCur;
            ESP_ERROR_CHECK(bsp_i2s_reconfig(sampFreq));
            printf("Clock set current freq: %lu Hz, channelNum: %d\n", sampFreq, channelNum);
            return true;
            break;

            // Unknown/Unsupported control
            default:
            TU_BREAKPOINT();
            return false;
        }
    }
    return false;    // Yet not implemented
}

// Invoked when audio class specific get request received for an EP
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void) rhport;

    TU_LOG2("%s called\r\n",__func__);
    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t ep = TU_U16_LOW(p_request->wIndex);

    (void) channelNum; (void) ctrlSel; (void) ep;

    //    return tud_control_xfer(rhport, p_request, &tmp, 1);

    return false;     // Yet not implemented
}

// Invoked when audio class specific get request received for an interface
bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void) rhport;

    TU_LOG2("%s called\r\n",__func__);
    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t itf = TU_U16_LOW(p_request->wIndex);

    (void) channelNum; (void) ctrlSel; (void) itf;

    return false;     // Yet not implemented
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void) rhport;

    TU_LOG2("%s called\r\n",__func__);
    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    // uint8_t itf = TU_U16_LOW(p_request->wIndex);           // Since we have only one audio function implemented, we do not need the itf value
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

    // Input terminal (Microphone input)
    if (entityID == UAC2_ENTITY_SPK_INPUT_TERMINAL) {
        switch ( ctrlSel ) {
        case AUDIO_TE_CTRL_CONNECTOR: {
            // The terminal connector control only has a get request with only the CUR attribute.
            audio_desc_channel_cluster_t ret;

            // Those are dummy values for now
            ret.bNrChannels = 1;
            ret.bmChannelConfig = 0;
            ret.iChannelNames = 0;

            TU_LOG2("    Get terminal connector\r\n");

            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void *) &ret, sizeof(ret));
        }
        break;

        // Unknown/Unsupported control selector
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    // Feature unit
    if (entityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        switch ( ctrlSel ) {
        case AUDIO_FU_CTRL_MUTE:
            // Audio control mute cur parameter block consists of only one byte - we thus can send it right away
            // There does not exist a range parameter block for mute
            TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
            return tud_control_xfer(rhport, p_request, &mute[channelNum], 1);

        case AUDIO_FU_CTRL_VOLUME:
            switch ( p_request->bRequest ) {
            case AUDIO_CS_REQ_CUR:
                TU_LOG2("    Get Volume of channel: %u\r\n", channelNum);
                return tud_control_xfer(rhport, p_request, &volume[channelNum], sizeof(volume[channelNum]));

            case AUDIO_CS_REQ_RANGE:
                TU_LOG2("    Get Volume range of channel: %u\r\n", channelNum);

                // Copy values - only for testing - better is version below
                audio_control_range_2_n_t(1)
                ret;

                ret.wNumSubRanges = 1;
                ret.subrange[0].bMin = -90;    // -90 dB
                ret.subrange[0].bMax = 90;      // +90 dB
                ret.subrange[0].bRes = 1;       // 1 dB steps

                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void *) &ret, sizeof(ret));

            // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
            }
            break;

        // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    // Clock Source unit
    if ( entityID == UAC2_ENTITY_CLOCK ) {
        switch ( ctrlSel ) {
        case AUDIO_CS_CTRL_SAM_FREQ:
            // channelNum is always zero in this case
            switch ( p_request->bRequest ) {
            case AUDIO_CS_REQ_CUR:
                TU_LOG2("    Get Sample Freq.\r\n");
                return tud_control_xfer(rhport, p_request, &sampFreq, sizeof(sampFreq));

            case AUDIO_CS_REQ_RANGE:
                TU_LOG2("    Get Sample Freq. range\r\n");
                audio_control_range_4_n_t(N_sampleRates) rangef =
                {
                    .wNumSubRanges = tu_htole16(N_sampleRates)
                };
                TU_LOG1("Clock get %d freq ranges\r\n", N_sampleRates);
                for(uint8_t i = 0; i < N_sampleRates; i++)
                {
                    rangef.subrange[i].bMin = (int32_t)sampleRatesList[i];
                    rangef.subrange[i].bMax = (int32_t)sampleRatesList[i];
                    rangef.subrange[i].bRes = 0;
                    TU_LOG1("Range %d (%d, %d, %d)\r\n", i, (int)rangef.subrange[i].bMin, (int)rangef.subrange[i].bMax, (int)rangef.subrange[i].bRes);
                }
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &rangef, sizeof(rangef));
            //    TU_LOG2("    Get Sample Freq. range\r\n");
            //    return tud_control_xfer(rhport, p_request, &sampleFreqRng, sizeof(sampleFreqRng));

            // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
            }
            break;

        case AUDIO_CS_CTRL_CLK_VALID:
            // Only cur attribute exists for this request
            TU_LOG2("    Get Sample Freq. valid\r\n");
            return tud_control_xfer(rhport, p_request, &clkValid, sizeof(clkValid));

        // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }

    TU_LOG2("  Unsupported entity: %d\r\n", entityID);
    return false;     // Yet not implemented
}
// 
bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting)
{
  (void)rhport;
  (void)func_id;
  (void)ep_out;
  (void)cur_alt_setting;

  data_out_buf_cnt = tud_audio_read(data_out_buf, n_bytes_received);
  bsp_i2s_write(data_out_buf, data_in_buf_cnt);

  return true;
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
    (void) rhport;
    (void) itf;
    (void) ep_in;
    (void) cur_alt_setting;

    /*** Here to send audio buffer, only use in audio transmission begin ***/
    tud_audio_write(data_in_buf, data_in_buf_cnt*2);

    return true;
}

bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
    (void) rhport;
    (void) n_bytes_copied;
    (void) itf;
    (void) ep_in;
    (void) cur_alt_setting;

    //static int64_t t_earlier = 0;
    //struct timeval tv_now;
    //gettimeofday(&tv_now, NULL);
    //int64_t t_now = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;

    /*** Here to fill audio buffer, only use in audio transmission begin ***/
    //n_bytes = rb_read(rb_debug, (char *)data_in_buf, i2s_read_buflen, 100) ;

    TU_ASSERT(bsp_i2s_read(data_in_buf, data_in_buf_cnt) == data_in_buf_cnt ) ;

    //delta_times[i].delta_time = t_now - t_earlier;

    //t_earlier = t_now;
    return true;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void) rhport;

    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt == 0)
      blink_interval_ms = BLINK_MOUNTED;

    TU_LOG2("%s called\r\n",__func__);
    return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void)rhport;
  uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  TU_LOG2("Set interface %d alt %d\r\n", itf, alt);
  if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt != 0)
      blink_interval_ms = BLINK_STREAMING;

  // Clear buffer when streaming format is changed
  data_out_buf_cnt = 0;
  if(alt != 0)
  {
    current_resolution = resolutions_per_format[alt-1];
  }

  return true;
}

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}