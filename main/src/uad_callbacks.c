/*
 * USB Audio Device callback routines
 */

#include <string.h>
#include <math.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "tusb.h"
#include "tusb_config.h"
#include "esp_private/usb_phy.h"
#include "usb_descriptors.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "i2s_functions.h"

#include "percent_table.h"

#include "blink.h"
  

//#define I2S_EXTERNAL_LOOPBACK

//#define TU_LOG2 printf

static const char *TAG = "uad_callbacks";

extern uint32_t blink_state;

/* data_in_buf is populated in tud_audio_tx_done_post_load_cb() from the circular buffer */
/* tud_audio_tx_done_pre_cb() copies this data to the endpoint buffer */
extern size_t  data_in_buf_cnt ;      // value is set based on sample rate etc. when the i2s is configured 
static int16_t data_in_buf[CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX/ 2] = {0};

// Buffer for speaker data
static int16_t data_out_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 2 ] = {0};

// Speaker data size received in the last frame
volatile static size_t data_out_buf_cnt = 0;
static TaskHandle_t spk_task_handle; 

// Speaker and microphone status
volatile static bool s_spk_active = false;
volatile static bool s_mic_active = false;
// Resolution per format, Note: due to the limitation of the codec, we currently just support one resolution
const uint8_t spk_resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX
                                                                           };
const uint8_t mic_resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_TX
                                                                           }; 

// Current resolution, update on format change
static uint8_t s_spk_resolution = spk_resolutions_per_format[0];
static uint8_t s_mic_resolution = mic_resolutions_per_format[0];
static size_t s_spk_bytes_ms = 0;
static size_t s_mic_bytes_ms = 0;

//static RingbufHandle_t rb_handle = {0};
//#define RB_LENGTH (3*CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_OUT)

// Audio controls

// Current states
static int8_t  spk_mute   [CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1] = {0};       // +1 for master channel 0
static int16_t spk_volume [CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];    // +1 for master channel 0
static int16_t spk_gain   [CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX];
static int8_t  mic_mute   [CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1] = {0};       // +1 for master channel 0
static int16_t mic_volume [CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1] = {32767,32767,32767};    // +1 for master channel 0
static int16_t mic_gain   [CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX];

// Volume control range
// From UAC2.0:
// The settings for the CUR, MIN, and MAX attributes can range from +127.9961 dB (0x7FFF)
// down to -127.9961 dB (0x8001) in steps (RES) of 1/256 dB or 0.00390625 dB (0x0001). 
// The settings for the RES attribute can only have positive values and range from 1/256 dB 
// (0x0001) to +127.9961 dB (0x7FFF). In addition, code 0x8000, representing silence (i.e., -∞ dB), 
// must always be implemented. However, it must never be reported as the MIN attribute value.

// For example: 50dB is 50*256 (each step is 1/256dB) = 12800; -50dB is 0xCE00 (0x8000+0x4E00=-128+78=-50)
// Host will send the dB value in the code set above.
enum {
    VOLUME_CTRL_0_DB = 0,
    VOLUME_CTRL_10_DB = 2560,
    VOLUME_CTRL_20_DB = 5120,
    VOLUME_CTRL_30_DB = 7680,
    VOLUME_CTRL_40_DB = 10240,
    VOLUME_CTRL_50_DB = 12800,
    VOLUME_CTRL_60_DB = 15360,
    VOLUME_CTRL_70_DB = 17920,
    VOLUME_CTRL_80_DB = 20480,
    VOLUME_CTRL_90_DB = 23040,
    VOLUME_CTRL_100_DB = 25600,
    VOLUME_CTRL_SILENCE = 0x8000,
};
static audio_control_range_2_n_t(1) range_vol = { 
    .wNumSubRanges = tu_htole16(1),
    .subrange[0] = { .bMin = tu_htole16(-VOLUME_CTRL_50_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(50) }
};

// List of supported sample rates
const uint32_t sampleRatesList[] = { 16000, 24000, 32000 };

uint32_t sampFreq;
uint8_t clkValid = 0;

#define N_sampleRates  TU_ARRAY_SIZE(sampleRatesList)

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+
/*
static void rb_init(void)
{
    rb_handle = xRingbufferCreate(RB_LENGTH, RINGBUF_TYPE_BYTEBUF);
    if (rb_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create ring buffer");
    }
}
void rb_write(int16_t *buf, size_t size)
{
    if (buf == NULL) {
        return;
    }
    xRingbufferSend(rb_handle, (void *)buf, size, 0);
}
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
static void usb_device_task(void *param)
{
    (void) param;

    usb_phy_init(); 

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

static size_t bytes_available_ary[512] = {0};

static void display_stats(size_t *data,int size){
    // find max
    size_t max = data[0];
    for(int i=0; i< size; i++){
        if(data[i]>max) max=data[i];
    }
    if(max == 0){
        printf("Maximum bytes available: 0\n");
        return;
    }
    printf("Statistics of the available bytes\n==============================================================\n");
    for(int i=0; i< size; i++){
        if(data[i] > 0){
            printf("%3d ",i);
            printf("[%6d] ",data[i]);
            for(int j=0;j<(int)(50*data[i]/max);j++){
                printf("+");
            }
            printf("\n");
        }
    }
}

int16_t q15_multiply(int16_t a, int16_t b){
    // Q15 format numbers are assumed to be in 1.15 format (ranges from -1 to 0.99999)
    // Multiplication is usual operation. However the result is in 2.30 format and requires 
    // 32 bits to contain. To convert this result to 1.15 format, a left shift is performed
    // and upper 16 bits are returned. 

    int32_t t = a * b;
    t = t<<1;
    int16_t *p = (int16_t*) &t;
    // p points to lower 16 bit, (p+1) points to higher 16 bits
    return *++p;
}

// process samples for mute and volume
static void amp (int16_t *s, size_t nframes, int16_t gain[]){
    //int16_t *t = s;

    for(int i=0; i<nframes; i++){
        *s = q15_multiply(*s, gain[0]);
        s++;
        *s = q15_multiply(*s, gain[1]);
        s++;
    }
}


static void usb_headset_spk(void *pvParam)
{
        data_out_buf_cnt = s_spk_bytes_ms;
    while (1) {
        gpio_set_level(GPIO_NUM_7, 0);
        // While the speaker stream is closed, we wait for the signal from tud_audio_set_itf_cb(), 
        // which does the job of opening and closing the interface (stream). 
        // NOTE that this task is suspended when ulTaskNotifyTake() is executed and remains suspended
        // till the stream is opened.
        if (s_spk_active == false) {
            ulTaskNotifyTake(pdFAIL, portMAX_DELAY);
            vTaskDelay(20); //1 tick is portTICK_PERIOD_MS ms (usually 10ms); We'll lost that much of audio
            continue;
        }
        // The timing of the while loop is dictated by i2s_write. A write to I2S DMA channel buffers
        // is a blocking operation. The value of data_out_buf_cnt and blocking nature of this operation
        // makes sure that the while loop runs once in a 1ms.
        // NOTE also the placement of i2s_write call here, the first thing in the loop, rather than after
        // tud_audio_read(). By writing this dummy (hopefully all 0) data first with this blocking call,
        // we make sure that there is sufficient data to read from the endpoint for the next i2s_write.
        // Otherwise, we loop unnecessarily many times taking up cycles and tripping watchdog timer.  
        bsp_i2s_write(data_out_buf, data_out_buf_cnt, s_spk_resolution);

        size_t bytes_available = tud_audio_available();
        bytes_available_ary[bytes_available]++;
        
        // See comments above. We do not except the condition in if() to ever evaluate to true.
        if (bytes_available < s_spk_bytes_ms) {
            ESP_LOGI(TAG,"Only %d bytes available; expecting >= %d bytes",bytes_available,s_spk_bytes_ms);
            //vTaskDelay(2);
            gpio_set_level(GPIO_NUM_7, 1);
            continue ;
        }   
        data_out_buf_cnt = tud_audio_read(data_out_buf, s_spk_bytes_ms);
        assert(data_out_buf_cnt == s_spk_bytes_ms);
#ifndef I2S_EXTERNAL_LOOPBACK
        amp(data_out_buf,data_out_buf_cnt/4, spk_gain);
#endif
        //bsp_i2s_write(data_out_buf, data_out_buf_cnt, s_spk_resolution);
        gpio_set_level(GPIO_NUM_7, 1);
    }
}

esp_err_t usb_headset_init(void)
{
    s_spk_bytes_ms = sampFreq / 1000 * s_spk_resolution * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX/ 8;
    s_mic_bytes_ms = sampFreq / 1000 * s_mic_resolution * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX/ 8;

    // we give the higher priority to playback task, to avoid the data pending in the ring buffer
    BaseType_t ret_val = xTaskCreate(usb_headset_spk, "usb_headset_spk", 4 * 1024, NULL, 8, &spk_task_handle);
    if (ret_val != pdPASS) {
        ESP_LOGE(TAG, "Failed to create usb_headset_spk task");
        return ESP_FAIL;
    }

    ret_val = xTaskCreatePinnedToCore(usb_device_task, "usb_device_task", 4 * 1024, NULL, 10, NULL, 1);
    if (ret_val != pdPASS) {
        ESP_LOGE(TAG, "Failed to create usb_task");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "TinyUSB initialized");
    return ESP_OK;
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

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

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void)rhport;
  uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  TU_LOG2("Set interface %d alt %d\r\n", itf, alt);
  if (ITF_NUM_AUDIO_STREAMING_SPK == itf) {
    if(alt != 0) {
        uint8_t spk_resolution = spk_resolutions_per_format[alt - 1];
        s_spk_resolution = spk_resolution;
        s_spk_bytes_ms = sampFreq / 1000 * s_spk_resolution * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX/ 8;
        //rx_bytes_required = (CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_MS - 1) * s_spk_bytes_ms;
        s_spk_active = true;
        // Clear buffer when streaming format is changed
        data_out_buf_cnt = 0;
        xTaskNotifyGive(spk_task_handle);
        TU_LOG1("Speaker interface %d-%d opened (%d bits)\n", itf, alt, s_spk_resolution);
        ESP_LOGI(TAG,"Speaker interface %d opened (alt=%d) : %d bits @%lu Hz", itf, alt, s_spk_resolution,sampFreq);
    }
    else {
        s_spk_active = false;
        ESP_LOGI(TAG,"Speaker interface %d closed (alt=%d)", itf, alt);
        display_stats(bytes_available_ary,512);
    }
  } else if (ITF_NUM_AUDIO_STREAMING_MIC == itf ) {
    if(alt != 0) {
        uint8_t mic_resolution = mic_resolutions_per_format[alt - 1];
        s_mic_resolution = mic_resolution;
        s_mic_bytes_ms = sampFreq / 1000 * s_mic_resolution * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX/ 8;
        //tx_bytes_required = (CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_MS - 1) * s_mic_bytes_ms;
        s_mic_active = true; 
        //xTaskNotifyGive(mic_task_handle);
        TU_LOG1("Microphone interface %d-%d opened (%d bits)\n", itf, alt, s_mic_resolution);
        ESP_LOGI(TAG,"Microphone interface %d opened (alt=%d) : %d bits @%lu Hz", itf, alt, s_mic_resolution,sampFreq);
    }
    else {
        s_mic_active = false;
        ESP_LOGI(TAG, "Microphone interface %d closed (alt=%d)", itf, alt);
    }
  }   

  if(alt != 0)
    blink_state = BLINK_STREAMING;
  else
    blink_state = BLINK_MOUNTED;

  return true;
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

    //audio_control_request_t const *request = (audio_control_request_t const *)p_request;

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
            ESP_LOGI(TAG,"Get Terminal connector");

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
            //TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
            //return tud_control_xfer(rhport, p_request, &mute[channelNum], 1);
            switch(p_request->bRequest){
                case AUDIO_CS_REQ_CUR:
                    audio_control_cur_1_t mute1 = { .bCur = spk_mute[channelNum]};
                    TU_LOG2("Get channel %u spk_mute %d\r\n", channelNum, mute1.bCur);
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &mute1, sizeof(mute1));
                default:
                    TU_BREAKPOINT();
                    return false;
            }

        case AUDIO_FU_CTRL_VOLUME:
            switch ( p_request->bRequest ) {
            case AUDIO_CS_REQ_CUR:
                //TU_LOG2("    Get Volume of channel: %u\r\n", channelNum);
                //return tud_control_xfer(rhport, p_request, &volume[channelNum], sizeof(volume[channelNum]));
                audio_control_cur_2_t cur_vol = { .bCur = tu_htole16(spk_volume[channelNum]) };
                TU_LOG1("Get channel %u spk_volume %d dB\r\n", channelNum, cur_vol.bCur / 256);
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &cur_vol, sizeof(cur_vol));

            case AUDIO_CS_REQ_RANGE:

                //return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void *) &ret, sizeof(ret));

                TU_LOG2("Get channel %u spk_volume range (%d, %d, %u) dB\r\n", channelNum,
                    range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &range_vol, sizeof(range_vol));

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

    if (entityID == UAC2_ENTITY_MIC_FEATURE_UNIT) {
        switch ( ctrlSel ) {
        case AUDIO_FU_CTRL_MUTE:
            // Audio control mute cur parameter block consists of only one byte - we thus can send it right away
            // There does not exist a range parameter block for mute
            //TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
            //return tud_control_xfer(rhport, p_request, &mute[channelNum], 1);
            switch(p_request->bRequest){
                case AUDIO_CS_REQ_CUR:
                    audio_control_cur_1_t mute1 = { .bCur = mic_mute[channelNum]};
                    TU_LOG2("Get channel %u mic_mute %d\r\n", channelNum, mute1.bCur);
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &mute1, sizeof(mute1));
                default:
                    TU_BREAKPOINT();
                    return false;
            }

        case AUDIO_FU_CTRL_VOLUME:
            switch ( p_request->bRequest ) {
            case AUDIO_CS_REQ_CUR:
                //TU_LOG2("    Get Volume of channel: %u\r\n", channelNum);
                //return tud_control_xfer(rhport, p_request, &volume[channelNum], sizeof(volume[channelNum]));
                audio_control_cur_2_t cur_vol = { .bCur = tu_htole16(mic_volume[channelNum]) };
                TU_LOG1("Get channel %u mic_volume %d dB\r\n", channelNum, cur_vol.bCur / 256);
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &cur_vol, sizeof(cur_vol));

            case AUDIO_CS_REQ_RANGE:

                //return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void *) &ret, sizeof(ret));

                TU_LOG2("Get channel %u mic_volume range (%d, %d, %u) dB\r\n", channelNum,
                    range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &range_vol, sizeof(range_vol));

            // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
            }
            break;

        // Unknown/Unsupported control
        default:
            TU_LOG2(" Unsupported control in Mic Feature Unit\r\n");
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

            spk_mute[channelNum] = ((audio_control_cur_1_t *) pBuff)->bCur;

            // recalculate the gain multiplier for the channel
            spk_gain[0] =  
                (spk_mute[0] || spk_mute[1]) ? 0 : q15_multiply (percent_table[spk_volume[0]],percent_table[spk_volume[1]]);
            spk_gain[1] =  
                (spk_mute[0] || spk_mute[2]) ? 0 : q15_multiply (percent_table[spk_volume[0]],percent_table[spk_volume[2]]);

            TU_LOG2("    Set speaker Mute: %d of channel: %u\r\n", spk_mute[channelNum], channelNum);
            return true;

        case AUDIO_FU_CTRL_VOLUME:
            // Request uses format layout 2
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));

            spk_volume[channelNum] = ((audio_control_cur_2_t *) pBuff)->bCur;

            //ESP_LOGI(TAG,"spk_volume[%d] = %d ", channelNum, spk_volume[channelNum]);
            int spk_volume_db = spk_volume[channelNum] / 256; // Convert to dB
            int volume = (spk_volume_db + 50) * 2; // Map to range 0 to 100
            spk_volume[channelNum] = volume;
            //ESP_LOGI(TAG,"spk_volume[%d] = %d ", channelNum, spk_volume[channelNum]);
            // recalculate the gain multiplier for the channel
            spk_gain[0] =  
                (spk_mute[0] || spk_mute[1]) ? 0 : q15_multiply (percent_table[spk_volume[0]],percent_table[spk_volume[1]]);
            spk_gain[1] =  
                (spk_mute[0] || spk_mute[2]) ? 0 : q15_multiply (percent_table[spk_volume[0]],percent_table[spk_volume[2]]);

            TU_LOG2("Set speaker channel %d volume: %d dB (gains: %d, %d)\r\n", channelNum, spk_volume_db, spk_gain[0], spk_gain[1]);
            //ESP_LOGI(TAG,"Set speaker channel %d volume: %d dB (gains: %d, %d)\r\n", channelNum, spk_volume_db, spk_gain[0], spk_gain[1]);

            //TU_LOG2("    Set Volume: %d dB of channel: %u\r\n", volume[channelNum], channelNum);
            return true;

        // Unknown/Unsupported control
        default:
            TU_BREAKPOINT();
            return false;
        }
    }
    if ( entityID == UAC2_ENTITY_MIC_FEATURE_UNIT ) {
        switch ( ctrlSel ) {
        case AUDIO_FU_CTRL_MUTE:
            // Request uses format layout 1
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));

            mic_mute[channelNum] = ((audio_control_cur_1_t *) pBuff)->bCur;

            // recalculate the gain multiplier for the channel
            mic_gain[0] =  
                (mic_mute[0] || mic_mute[1]) ? 0 : q15_multiply (percent_table[mic_volume[0]],percent_table[mic_volume[1]]);
            mic_gain[1] =  
                (mic_mute[0] || mic_mute[2]) ? 0 : q15_multiply (percent_table[mic_volume[0]],percent_table[mic_volume[2]]);
            TU_LOG2("    Set mic Mute: %d of channel: %u\r\n", mic_mute[channelNum], channelNum);
            return true;

        case AUDIO_FU_CTRL_VOLUME:
            // Request uses format layout 2
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));

            mic_volume[channelNum] = ((audio_control_cur_2_t *) pBuff)->bCur;
            int mic_volume_db = mic_volume[channelNum] / 50; // Convert to dB
            int volume = (mic_volume_db + 50) * 2; // Map to range 0 to 100
            mic_volume[channelNum] = volume;
            //ESP_LOGI(TAG,"spk_volume[%d] = %d ", channelNum, spk_volume[channelNum]);
            // recalculate the gain multiplier for the channel
            mic_gain[0] =  
                (mic_mute[0] || mic_mute[1]) ? 0 : q15_multiply (percent_table[mic_volume[0]],percent_table[mic_volume[1]]);
            mic_gain[1] =  
                (mic_mute[0] || mic_mute[2]) ? 0 : q15_multiply (percent_table[mic_volume[0]],percent_table[mic_volume[2]]);

            TU_LOG2("Set microphone channel %d volume: %d dB (%d)\r\n", channelNum, mic_volume_db, volume);
            //ESP_LOGI(TAG,"Set microphone channel %d volume: %d dB (%d)", channelNum, mic_volume_db, volume);

            //TU_LOG2("    Set Volume: %d dB of channel: %u\r\n", volume[channelNum], channelNum);
            return true;

        // Unknown/Unsupported control
        default:
            TU_LOG2(" Unsupported control in Mic Feature Unit\r\n");
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

            
            uint32_t target_sampFreq = (uint32_t)((audio_control_cur_4_t *)pBuff)->bCur;
            if( target_sampFreq != sampFreq){
                bool not_supported = true;
                for(int i=0; i < N_sampleRates; i++){
                    if(sampleRatesList[i] == target_sampFreq) {
                        not_supported = false;
                        break;
                    }
                }
                if(not_supported){
                    ESP_LOGI(TAG,"Selected sampling frequency not supported");
                    return false;
                }
                sampFreq = target_sampFreq;
                s_spk_bytes_ms = sampFreq / 1000 * s_spk_resolution * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX/ 8;
                s_mic_bytes_ms = sampFreq / 1000 * s_mic_resolution * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX/ 8;
                //rx_bytes_required = (CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_MS - 1) * s_spk_bytes_ms;
                //tx_bytes_required = (CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_MS - 1) * s_mic_bytes_ms;
                TU_LOG1("Mic/Speaker frequency %" PRIu32 ", resolution %d, ch %d", target_sampFreq, s_spk_resolution, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX);
                ESP_ERROR_CHECK(bsp_i2s_reconfig(sampFreq));
                //printf("Clock set current freq: %lu Hz, channelNum: %d\n", sampFreq, channelNum);
                ESP_LOGI(TAG,"Mic/Speaker frequency %" PRIu32 ", resolution %d, ch %d", target_sampFreq, s_spk_resolution, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX);
            }
            return true;

            // Unknown/Unsupported control
            default:
            TU_BREAKPOINT();
            return false;
        }
    }
    return false;    // Yet not implemented
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
    (void) rhport;
    (void) itf;
    (void) ep_in;
    (void) cur_alt_setting;

    /*** Here to send audio buffer, only use in audio transmission begin ***/
    if(data_in_buf_cnt > 0)
    tud_audio_write(data_in_buf, data_in_buf_cnt);

    return true;
}

bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
    (void) rhport;
    (void) n_bytes_copied;
    (void) itf;
    (void) ep_in;
    (void) cur_alt_setting;


    assert(data_in_buf_cnt > 0);
    TU_ASSERT(bsp_i2s_read(data_in_buf, data_in_buf_cnt) == data_in_buf_cnt ) ;

#ifndef I2S_EXTERNAL_LOOPBACK
    amp(data_in_buf,data_in_buf_cnt/4, mic_gain);
#endif

    return true;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void) rhport;

    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    TU_LOG2("%s called\r\n",__func__);

    if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt == 0) {
        blink_state = BLINK_MOUNTED;
        s_spk_active = false;
        ESP_LOGI(TAG, "Speaker interface %d closed (alt=%d)", itf, alt);
    }
    if (ITF_NUM_AUDIO_STREAMING_MIC == itf && alt == 0){
        blink_state = BLINK_MOUNTED;
        s_mic_active = false;
        ESP_LOGI(TAG, "Microphone interface %d closed (alt=%d)", itf, alt);
    }

    return true;
}

// Invoked when device is mounted
void tud_mount_cb(void)
{
    s_spk_active = false;
    s_mic_active = false;
    ESP_LOGI(TAG, "USB mounted");
    blink_state = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    ESP_LOGI(TAG, "USB unmounted");
    blink_state = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void) remote_wakeup_en;
    s_spk_active = false;
    s_mic_active = false;
    (void)remote_wakeup_en;
    ESP_LOGI(TAG, "USB suspended");
    blink_state = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    ESP_LOGI(TAG, "USB resumed");
    blink_state = BLINK_MOUNTED;
}