#ifndef __BLINK_H__
#define __BLINK_H__
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

void configure_led(void);
void led_blinking_task(void);
void drive_led();

#endif // __BLINK_H__