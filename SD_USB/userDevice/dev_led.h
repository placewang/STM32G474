#ifndef DEVICE__LED__H
#define DEVICE__LED__H

#ifdef __cplusplus
extern "C" {
#endif



#include "gpio.h"




extern void dev_led_toggle(void);

extern void dev_led_on(void);

extern void dev_led_off(void);


extern int dev_led_set(int argc, char *argv[]);




#endif 


