// data_buffers.h
#ifndef _DATA_BUFFERS_H_
#define _DATA_BUFFERS_H_

typedef enum {PING, PONG, INVALID} ping_pong_flag ;

/* data_in_buf is to temporarily store incoming data from I2S (mic); it is read by the USB device callback function*/
/* data_in_buf is populated in tud_audio_tx_done_post_load_cb() from the circular buffer */
/* tud_audio_tx_done_pre_cb() copies this data to the endpoint buffer */
extern size_t  data_in_buf_cnt ;      // value is set based on sample rate etc. when the i2s is configured 
extern int16_t data_in_buf[];

extern volatile size_t data_out_buf_cnt ;
extern int16_t data_out_buf[];
#endif

//end data_buffers.h