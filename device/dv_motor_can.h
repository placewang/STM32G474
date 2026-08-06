#ifndef DV_MOTOR_CAN_H
#define DV_MOTOR_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platform/platform_can_bridge.h"

#include "dv_motor_base.h"


typedef struct  
{
	struct motor_base    base;    
	platform_can_client  client;    
}motor_can;

extern motor_can  s_moto_can1,s_moto_can2;

int dv_motor_can_init(motor_can *me,platform_can_device *bus);





#ifdef __cplusplus
}
#endif

#endif 



