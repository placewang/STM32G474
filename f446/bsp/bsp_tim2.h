#ifndef _BSP_TIM2_H__
#define _BSP_TIM2_H__

#ifdef __cplusplus
extern "C" {
#endif





typedef struct   timer_t timer_t;
extern  timer_t  TIM2_T1,TIM2_T2;

void bsp_tim2Start(timer_t *timer, unsigned int timeout_us);
unsigned char bsp_tim2GetSta(timer_t *timer);

#ifdef __cplusplus
}
#endif

#endif 
