#include "i2s_functions.h"
#include "data_buffers.h"
#include "esp_err.h"
#include "usb_descriptors.h"
#include "esp_log.h"
#include "tusb.h"
#include "tusb_config.h"
#include "esp_task_wdt.h"
#include "utilities.h"

//#define DUMMY_I2S 1

/* 
    Knowles SPH0645 I2S mic's data are MSB aligned 24 bits in a 32-bit word,
    whose lower 6 bits are always 0.
    Optional GAIN amount is considered in determining amount of right-shift
    on the raw data to extract 16 bits. 
    The data has signnficant offset, which is removed by a offset canceller filter.

    The function here hardcodes 2-ch 16bit configuration of final data.
*/


//#define I2S_EXTERNAL_LOOPBACK

#ifdef I2S_EXTERNAL_LOOPBACK
#define MIC_RESOLUTION      16
#define I2S_PROCESS_READ_DATA(left, right) decode_and_cancel_offset(&(left), &(right), true)
#else
#define MIC_RESOLUTION      24
#define GAIN                4      // in bits i.e., 1 =>2, 2=>4, 3=>8 etc.
#define I2S_PROCESS_READ_DATA(left, right) decode_and_cancel_offset(&(left), &(right), false); left <<= GAIN; right <<= GAIN 
#endif

static const char* TAG = "i2s_functions";

#ifdef DUMMY_I2S
#include <rom/ets_sys.h>
#include "trig_table.h"
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static i2s_chan_handle_t                tx_handle = NULL;        // I2S tx channel handler
static i2s_chan_handle_t                rx_handle = NULL;        // I2S rx channel handler
#endif

#define I2S_GPIO_DOUT    GPIO_NUM_34
#define I2S_GPIO_DIN     GPIO_NUM_36
#define I2S_GPIO_WS      GPIO_NUM_35
#define I2S_GPIO_BCLK    GPIO_NUM_37

/*raw buffer to read data from I2S dma buffers*/
static char rx_sample_buf [CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ];
static size_t  rx_sample_buflen = 0;// value is set based on sample rate etc. when the i2s is configured 

extern int32_t mic_gain[2];
extern int32_t spk_gain[2];
#ifdef DISPLAY_STATS
extern size_t spk_bytes_available_ary[];
#endif

esp_err_t bsp_i2s_init(i2s_port_t i2s_num, uint32_t sample_rate)
{
    esp_err_t ret_val = ESP_OK;

    i2s_slot_mode_t channel_fmt = I2S_SLOT_MODE_STEREO;

    // default chan_cfg : 
    // { .id = <i2s_num>, .role = <I2S_ROLE_MASTER>, .dma_desc_num = 6, .dma_frame_num = 240, .auto_clear = 0, .intr_priority = 0, }
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_num, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 2;
    // dma_frame_num is changed from dafult value of 240 to reduce latency
    chan_cfg.dma_frame_num = sample_rate/1000;  // number of frames in 1mS; cannot handle sample_rate like 44.1kHz
    
    ret_val |= i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle);
    //i2s_std_config_t std_cfg = I2S_CONFIG_DEFAULT(sample_rate, channel_fmt, bits_per_chan);
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        //.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, channel_fmt), //this mode seemed to work for SPH0645
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, channel_fmt), // this is needed for INMP441
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
    // Even though 32 bits for each data samples are read from I2S (for the specific Mic used), only 16 bits
    // per sample is sent out over USB.
    data_in_buf_cnt   = chan_cfg.dma_frame_num * std_cfg.slot_cfg.slot_mode *2 ;
    ESP_LOGI(TAG,"rx_sample_buflen: %d, data_in_buf_cnt: %d", rx_sample_buflen, data_in_buf_cnt);


    ret_val |= i2s_channel_enable(tx_handle);
    ret_val |= i2s_channel_enable(rx_handle);

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
 This filter is an implementation of a 1st-order filter described in 
 https://docs.xilinx.com/v/u/en-US/wp279

 Data out of SPH645 has a significant -ve offset, which this high-pass filter attempts
 to filter out. 32bit raw_sample has 18bits of actual sample (in case of SPH645) in MSB aligned way.
 24bits of raw_data for INMP441 also is MSB aligned in 32bit, but it has no such offset.
 The last operand 'reset' can be set to 'true' to bypass the filtering operation.
 'reset' set to 'true' also resets the accumulator within the filter. This function needs 
 to be called once with 'reset' = 'true' before start of the filtering operation.

 The processed samples are returned in the same variable but as 16bit LSB aligned value.
 
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
        //    printf("left sample clips before offset\n");
        }
        //final_value = *left_sample_p - (l_offset & (int32_t)(-1 << (32-MIC_RESOLUTION)));
        final_value = (*left_sample_p & BIT_MASK) - (l_offset & BIT_MASK);
        *left_sample_p = mul_1p31x8p24(final_value,mic_gain[0]);
        l_offset += (*left_sample_p) << FILTER_RESPONSE_MULTIPLIER;

    }

    if(right_sample_p != NULL)
    {
        if((*right_sample_p >> (32-MIC_RESOLUTION))> 32767 || (*right_sample_p >> (32 - MIC_RESOLUTION))< -32768){
        //    printf("right sample clips before offset\n");
        }
        //final_value = *right_sample_p - (r_offset & (int32_t)(-1 << (32-MIC_RESOLUTION)));
        final_value = (*right_sample_p & BIT_MASK)  - (r_offset & BIT_MASK);
        *right_sample_p = mul_1p31x8p24(final_value,mic_gain[1]);
        r_offset += (*right_sample_p) << FILTER_RESPONSE_MULTIPLIER;

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
//    int t_i2s = 0; static int32_t raw_data[64]; static int32_t processed_data[64]; int k = 0;
//    static int16_t final_data[64];
/*
  This function is called by tud_audio_tx_done_post_load_cb(). 
  It reads 32bits raw data for both left and right channels from I2S DMA buffers, 
  gets upper 18bits LSB aligned. After offset cancellation, 16 lower bits are 
  returned in data_buf. count is the requested number of bytes. Actual number of
  bytes read is returned.
*/
size_t bsp_i2s_read(void *data_buf, size_t count)
{
    // t_i2s ++;
    int16_t *out_buf = (int16_t*)data_buf;
    static size_t  n_bytes;
    static int32_t d_left, d_right;

    int i = 0;
 
    while (i<count/2) {     // i keeps count of number of int16_t types processed into out_buf
        n_bytes = 0;
        if(i2s_channel_read(rx_handle,rx_sample_buf, rx_sample_buflen, &n_bytes, 200) == ESP_OK) {
            // i2s_channel_read is blocking; it is expected to block till it gets enough number of
            // bytes (e.g., 128 bytes for 16KHz sampling rate every ms). This should keep time.
            //assert(rx_sample_buflen==n_bytes);
            if(rx_sample_buflen != n_bytes)
            printf("rx_sample_buflen : %d, n_bytes: %d\n", rx_sample_buflen, n_bytes);
            int j = 0;
            while(j < n_bytes) {
                if((n_bytes - j) >= 8) {
                    memcpy(&d_left,(rx_sample_buf+j),4);
                    memcpy(&d_right,(rx_sample_buf+j+4),4);
                    //if(t_i2s>1000 && t_i2s<1003) raw_data[k] = d_left ;
                    decode_and_cancel_offset(&d_left, &d_right, true); // true: no offset cancelling; just the shifting 
                    *(out_buf+i)   = d_left;
                    //if(t_i2s>1000 && t_i2s<1003) final_data[k++] = *(out_buf+i);
                    *(out_buf+i+1) = d_right;
                    i += 2;
                    j += 8;
                }
                else {
                    // we have a problem
                    printf("%u : i2s_read_buffer failed (only %d bytes available; expected 8)\r\n", xthal_get_ccount(),(n_bytes-j));
                    break;
                }
            }
            //printf("k: %d ",k);
            //if(i>=count) printf("i: %d, count: %d\n",i,count);
            //fflush(stdout);
            assert(i<=count);
        }
        else {
            printf("i2s_channel_read failed \r\n");
            break;
        }
    } 
    /*
    if(t_i2s==1003){
        printf("mic_gain: %ld , %ld\n",mic_gain[0], mic_gain[1]);
        for(k=0;k<32;k++){
        //printf("%ld => %ld => %d\n",raw_data[k],processed_data[k],final_data[k]);
        printf("%08lx %ld \t %08lx %ld \t %08x %d\n",raw_data[k],raw_data[k],processed_data[k], processed_data[k], final_data[k], final_data[k]);
        }
    }
    */

    if(i*2 < count)
    ESP_LOGI(TAG,"returning %d bytes; was asked for %d bytes",i*2,count);
    return i*2;
}
#endif

//static int16_t data_dump[8192];

/*
  This function formats the data (16 bits to MSB aligned 32 bits etc..) using a local buffer
  tx_sample_buf and writes to the I2S DMA buffer to be sent out over I2S.
  Each sample in tx_sample_buf is int32_t type. For an incoming (i.e. from host) stereo stream,
  tx_sample_buf needs to be n_bytes/2 * 32-bit. 
  Max val of n_bytes is CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ (see uad_callbacks.c).
  This function is called by audio_consumer_func() task.
*/
void bsp_i2s_write(void *data_buf, size_t n_bytes){

    /* each sample is 32bits and there are 2 channels; so a EP buffer of CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ (N) 
     * bytes (each data is 16bits) will produce (N/2)*2=N 32bits total o/p samples for L+R  
     */

    static int32_t tx_sample_buf [CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ/2];

    int16_t *src   = (int16_t *)data_buf;
    int16_t *limit = (int16_t *)data_buf + n_bytes/2 ;
    int32_t *dst   = tx_sample_buf;
    int32_t data;

    size_t num_bytes = 0;

    // convert 16 bits to 32 bits (MSB aligned)
      while (src < limit)
      {
        data = (int32_t)(int16_t)(*src++)<<16;  // MSB aligning
        *dst++ = data;
        if(CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX == 1 /* single channel*/)
            // copy the same data to both channels. NOTE This will depend on how the speaker is wired to I2S bus
            *dst++ = data;
      }
      assert(dst <= &tx_sample_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ/2-1]);

      if(CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX == 1 /* single channel*/) {
        // total number of bytes in tx_sample_buf is count*4 since each 16bit sample in data_buf
        // made into a 32bit value and then replicated in both left and right channel.

        ESP_ERROR_CHECK(i2s_channel_write(tx_handle,tx_sample_buf, n_bytes*4, &num_bytes, 200)) ;

      }
      else {
        // total number of bytes in tx_sample_buf is count*2 since each 16bit sample in data_buf
        // made into a 32bit value.

        ESP_ERROR_CHECK(i2s_channel_write(tx_handle,tx_sample_buf, n_bytes*2, &num_bytes, 200)) ;

      }
}

static void speaker_amp (int16_t *s, size_t nframes, int32_t gain[] ){
    //int16_t *t = s;
    if(nframes == 0) return;
    int32_t sig;
    for(int i=0; i<nframes; i++){
        sig = ((int32_t)*s)<<16;
        *s = mul_1p31x8p24(sig, gain[0]);
        s++;
        sig = ((int32_t)*s)<<16;
        *s = mul_1p31x8p24(sig, gain[1]);
        s++;
    }
}

/* The following function is called by i2s_consumer_func to get data from USB
  to be sent to the I2S transmit DMA buffer.
  This function is expected to populate 'count' number of bytes in the buffer
  (pointed to by data_buf). It may occasionally send fewer bytes. The number of 
  bytes actually filled in the buffer is returned.
*/
uint16_t (*i2s_get_data)(void *data_buf, uint16_t count);

/* Each call of (*i2s_get_data)() requests 's_spk_bytes_ms' bytes from
   the USB. Value of this variable is set based on sampling frequency etc..
   in the USB related functions.
*/
extern size_t s_spk_bytes_ms;

/* This function is meant to be run as a task. It repeatedly calls a producer 
    function (i2s_get_data()) to get data and sends it along to I2S DMA.
    The 'active' flag is to force the loop to introduce a delay. Otherwise, 
    the loop spins too fast without a break and trips the watchdog timer.
*/
void i2s_consumer_func(bool *active){

    /* running flag is used to delay the start of reading the producer FIFOs. 
       Without a delay (*i2s_get_data)() starts reading the FIFO immediately
       and returns with 0 bytes and ends up spinning too fast to trip the WDT.
       With this delay, USB FIFO's is guaranteed to accumulate some data in the
       beginning and is expected to always run ahead of the reading function.
    */
    static bool running = false;
    while(1){
        if (*active == false) {
            // reset the 'running' flag when the channel is closed
            running = false;
            //ulTaskNotifyTake(pdFAIL, portMAX_DELAY);
            // check after some delay (100ms) not to block the CPU
            vTaskDelay(100/portTICK_PERIOD_MS); 
            continue;
        }
        else if(running == false) {
            // get to 'running' after some delay so that the following
            // read function gets called after some data accumulates in USB FIFO.
            vTaskDelay(50/portTICK_PERIOD_MS);
            running = true;
        }

        data_out_buf_cnt = (*i2s_get_data)(data_out_buf, s_spk_bytes_ms);
        if (data_out_buf_cnt < s_spk_bytes_ms) {
            ESP_LOGI(TAG,"Only %d bytes available; expecting >= %d bytes",data_out_buf_cnt,s_spk_bytes_ms);
        //    //vTaskDelay(2);
        //    //continue ;
        }   

        // make sure that data_out_buf_cnt is a multiple of 2*CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX
        if(data_out_buf_cnt != ((data_out_buf_cnt>>(2*CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX))<<(2*CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)))
        {
            ESP_LOGI(TAG,"tud_audio_read returned %d bytes (not multiples of one frame worth bytes)", data_out_buf_cnt);
        }    
        speaker_amp((int16_t *)data_out_buf, data_out_buf_cnt/(2*CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX), spk_gain);

#ifdef DISPLAY_STATS
        spk_bytes_available_ary[data_out_buf_cnt]++;
#endif

        if(data_out_buf_cnt > 0)
            bsp_i2s_write(data_out_buf, data_out_buf_cnt);
    }
}