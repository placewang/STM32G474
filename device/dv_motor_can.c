
#include "container_of.h"

#include "dv_motor_can.h"

#define _sendLen  8


static int dv_motor_can_Enable(struct motor_base *me)
{

	uint8_t data[_sendLen]={0};
	
	motor_can *self = container_of(me,motor_can,base);
		/*..data fill	....*/
	platform_can_send(&self->client,data,_sendLen);

	return 0;
}

static int dv_motor_can_Disable(struct motor_base *me)
{
	uint8_t data[_sendLen]={0};
	
	motor_can *self = container_of(me,motor_can,base);
	/*..data fill	....*/
	platform_can_send(&self->client,data,_sendLen);
	return 0;
}

static const struct motor_ops can_ops = 
{
	.Enable=dv_motor_can_Enable,
    .Disable=dv_motor_can_Disable
};


int dv_motor_can_init(motor_can *me,platform_can_device *bus)
{
	int rc=0;
	
	if (!me || !bus)
		return -1;
    rc = dv_motor_base_init(&me->base, &can_ops);
	if(rc!=0)
		return -1;
	me->client.bus = bus;

	return 0;
}









