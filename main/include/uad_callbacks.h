// uad_callbacks.h
#ifndef _USB_CALLBACKS_H_
#define _USB_CALLBACKS_H_

void usb_device_task(void *param);
void usb_headset_spk(void *pvParam);
void usb_headset_init(void);
void usb_phy_init(void);
uint16_t uad_processed_data(void *buf, uint16_t cnt);
extern size_t (*usb_get_data)(void *buf, size_t cnt);
extern volatile bool s_spk_active ;
extern volatile bool s_mic_active ;

#endif
//end uad_callbacks.h