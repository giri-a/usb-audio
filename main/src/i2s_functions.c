#include "i2s_functions.h"
#include "esp_err.h"
#include "usb_descriptors.h"
#include "esp_log.h"
#include "tusb.h"
#include "tusb_config.h"


extern void toggle_gpio();

//#define DUMMY_I2S 1


#ifdef DUMMY_I2S
#include <rom/ets_sys.h>
#include "trig_table.h"
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static i2s_chan_handle_t                tx_handle = NULL;        // I2S tx channel handler
static i2s_chan_handle_t                rx_handle = NULL;        // I2S rx channel handler
#endif

#define I2S_GPIO_DOUT    GPIO_NUM_14
#define I2S_GPIO_DIN     GPIO_NUM_15
#define I2S_GPIO_WS      GPIO_NUM_16
#define I2S_GPIO_BCLK    GPIO_NUM_17

// Current resolution, update on format change
extern uint8_t current_resolution;
size_t  data_in_buf_cnt = 0;   // value is set based on sample rate etc. when the i2s is configured 

/*raw buffer to read data from I2S dma buffers*/
static char rx_sample_buf [CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ];
static char rx_sample_buf_1 [CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ];
static size_t  rx_sample_buflen = 0;// value is set based on sample rate etc. when the i2s is configured 


esp_err_t bsp_i2s_init(i2s_port_t i2s_num, uint32_t sample_rate)
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
    assert(rx_sample_buflen <= sizeof(rx_sample_buf));
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

    // init the offset canceller filter on the read channel
    decode_and_cancel_offset(NULL, NULL, true);
    
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


#define MIC_RESOLUTION      16
#define GAIN                0      // in bits i.e., 1 =>2, 2=>4, 3=>8 etc.

/*
 This filter is an implementation of a 1st-order filter described in 
 https://docs.xilinx.com/v/u/en-US/wp279

 32bit raw_sample has 18bits of actual sample in MSB aligned way.
 The processed samples are returned in the same variable but as LSB aligned value.
 
 For faster settling of offset value, FILTER_RESPONSE_MULTIPLIER may be increased.
 The equivalent time constant (RC) = SAMPLING_PERIOD*2^(32-MIC_RESOLUTION-FILTER_RESPONSE_MULTIPLIER)
 For example, for fs=16kHz, MIC_RESOLUTION=18, and FILTER_RESPONSE_MULTIPLIER=0, 
 the time constant is 2^14/16000 ~= 1 second. Note that the reduction in time constant
 is in power of 2 for every increase in FILTER_RESPONSE_MULTIPLIER.

*/
#define  FILTER_RESPONSE_MULTIPLIER 2
#define  BIT_MASK ((0xFFFFFFFFUL)<<(32-MIC_RESOLUTION))
/* Call with reset=True once to initialize the filter OR when no offset cancellation is required */
void decode_and_cancel_offset(int32_t *left_sample_p, int32_t *right_sample_p, bool reset)
{
    static int32_t l_offset = 0;  // offset for L channel; (state of the filter)
    static int32_t r_offset = 0;  // offset for R channel; (state of the filter)
    int32_t final_value;

    if(reset){
        l_offset = 0;
        r_offset = 0;
    }
    if(left_sample_p != NULL)
    {
        if((*left_sample_p >> (32-MIC_RESOLUTION))> 32767 || (*left_sample_p >> (32 - MIC_RESOLUTION))< -32768){
            printf("left sample clips before offset\n");
        }
        //final_value = *left_sample_p - (l_offset & (int32_t)(-1 << (32-MIC_RESOLUTION)));
        final_value = *left_sample_p - (l_offset & BIT_MASK);
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
        //final_value = *right_sample_p - (r_offset & (int32_t)(-1 << (32-MIC_RESOLUTION)));
        final_value = *right_sample_p - (r_offset & BIT_MASK);
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
        decode_and_cancel_offset(&val, NULL, false);

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

    //toggle_gpio();
    while (i<count) {
        n_bytes = 0;
        if(i2s_channel_read(rx_handle,rx_sample_buf_1, rx_sample_buflen, &n_bytes, 200) == ESP_OK) {

            // i2s_channel_read is blocking; it is expected to block till it gets enough number of
            // bytes (e.g., 128 bytes for 16KHz sampling rate every ms). This should keep time.
            assert(rx_sample_buflen==n_bytes);
            int j = 0;
            while(j < n_bytes) {
                if((n_bytes - j) >= 8) {
                    memcpy(&d_left,(rx_sample_buf+j),4);
                    memcpy(&d_right,(rx_sample_buf+j+4),4);
                    //decode_and_cancel_offset(&d_left, &d_right, true);
                    //d_left <<= GAIN; d_right <<= GAIN;
                    d_left >>= 16; d_right >>=16;
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
            //if(i>=count) printf("i: %d, count: %d\n",i,count);
            //fflush(stdout);
            assert(i<=count);
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
  EP_OUT buffer in data_buf. The number of bytes provided in data_buf is n_bytes.
  This function formats the data (16 bits to MSB aligned 32 bits etc..) using a local buffer
  tx_sample_buf and writes to the I2S DMA buffer to be sent out over I2S.
  Each sample in tx_sample_buf is int32_t type. For an incoming (i.e. from host) stereo stream,
  tx_sample_buf needs to be n_bytes/2 * 32-bit. 
  Max val of n_bytes is CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ (see uad_callbacks.c).
*/
void bsp_i2s_write(uint16_t *data_buf, uint16_t n_bytes){
    //return;
    /* each sample is 32bits and there are 2 channels; so a EP buffer of CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ (N) 
     * bytes (each data is 16bits) will produce (N/2)*2=N 32bits total o/p samples for L+R  
     */

    static int32_t tx_sample_buf [CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ/2];

    gpio_set_level(GPIO_NUM_12, 0);

    int16_t *src   = (int16_t*)data_buf;
    int16_t *limit = (int16_t*)data_buf + n_bytes/2 ;
    int32_t *dst   = tx_sample_buf;
    int32_t data;

    size_t num_bytes = 0;

    // convert 16 bits to 32 bits (MSB aligned)
    //assert(current_resolution == 16);
    if (current_resolution == 16)
    {
      while (src < limit)
      {
        data = (int32_t)(int16_t)(*src++)<<16;  // MSB aligning
        *dst++ = data;
        if(CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX == 1 /* single channel*/)
            // copy the same data to both channels. NOTE This will depend on how the speaker is wired to I2S bus
            *dst++ = data;
      }
      assert(dst <= &tx_sample_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ-1]);
      // total number of bytes in tx_sample_buf is count*4 since each 16bit sample in data_buf
      // made into a 32bit value and then replicated in both left and right channel.
      if(CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX == 1 /* single channel*/) {
        //            memcpy(rx_sample_buf,tx_sample_buf,n_bytes*4);
        assert(i2s_channel_write(tx_handle,tx_sample_buf, n_bytes*4, &num_bytes, 200) == ESP_OK) ;
        //assert(num_bytes == n_bytes*4);
      }
      else {
        //            memcpy(rx_sample_buf,tx_sample_buf,n_bytes*2);
        assert(i2s_channel_write(tx_handle,tx_sample_buf, n_bytes*2, &num_bytes, 200) == ESP_OK) ;
        //assert(num_bytes == n_bytes*2);
      }
    }
    else if (current_resolution == 24)
    {
      while (src < limit)
      {
        data = *src++;
        *dst++ = data;
        if(CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX == 1 /* single channel*/)
            // copy the same data to both channels. NOTE This will depend on how the speaker is wired to I2S bus
            *dst++ = data;
      }
      if(CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX == 1 /* single channel*/)
        assert(i2s_channel_write(tx_handle,tx_sample_buf, n_bytes*2, &num_bytes, 200) == ESP_OK) ;
      else
        assert(i2s_channel_write(tx_handle,tx_sample_buf, n_bytes,   &num_bytes, 200) == ESP_OK) ;
    }
    gpio_set_level(GPIO_NUM_12, 1);

}