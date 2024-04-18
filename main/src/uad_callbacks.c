/*
 * USB Audio Device callback routines
 */

#include <string.h>
#include <math.h>
#include <sys/time.h>
#include "tusb.h"
#include "tusb_config.h"
#include "esp_private/usb_phy.h"
#include "usb_descriptors.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "i2s_functions.h"

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

/* data_in_buf is populated in tud_audio_tx_done_post_load_cb() from the circular buffer */
/* tud_audio_tx_done_pre_cb() copies this data to the endpoint buffer */
extern size_t  data_in_buf_cnt ;      // value is set based on sample rate etc. when the i2s is configured 
static uint16_t data_in_buf[CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX/ 2];

static uint16_t data_out_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 2];
// Speaker data size received in the last frame
static size_t data_out_buf_cnt = 0;


// Audio controls
// Current states
bool mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];                                      // +1 for master channel 0
uint16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];                                // +1 for master channel 0
uint32_t sampFreq;
uint8_t clkValid;

// Range states
// List of supported sample rates
extern const uint32_t sampleRatesList[] =
{
    16000, 24000, 32000, 48000
};

#define N_sampleRates  TU_ARRAY_SIZE(sampleRatesList)

// Range states
audio_control_range_2_n_t(1) volumeRng[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];         // Volume range state
audio_control_range_4_n_t(1) sampleFreqRng;                                             // Sample frequency range state

// Resolution per format
const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX,
                                                                        CFG_TUD_AUDIO_FUNC_1_FORMAT_2_RESOLUTION_RX};
// Current resolution, update on format change
uint8_t current_resolution;

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
  bsp_i2s_write(data_out_buf, data_out_buf_cnt);

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