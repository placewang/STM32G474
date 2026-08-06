#include <stdio.h>
#include "dv_motor_base.h"


int dv_motor_Enable(struct motor_base *me)
{
	if(!me)
	{
		return -1;
	}

	me->ops->Enable(me);
	return 0;
}

int dv_motor_Disable(struct motor_base *me)
{
	if(!me)
	{
		return -1;
	}
	me->ops->Disable(me);
	return 0;
}

int dv_motor_base_init(struct motor_base *me, 
            const struct motor_ops *ops)
{
	if (!me || !ops)
		return -1;

	me->ops   = ops;
	/*....*/

	return 0;
}




