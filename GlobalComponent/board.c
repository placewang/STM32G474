#include "initcall.h"
#include "platform/platform_can_bridge.h"
#include "dv_motor_can.h"


motor_can  s_moto_can1,s_moto_can2;



static int motor_board_init(void)
{
	int rc;
	
	platform_can_device *can1_bus,*can2_bus;
	
	_stm32_CAN_Init();
	
	platform_hw_can_init();

	/*
	 * 调 led_board_init 之前 main 已经调过 platform_init(), platform_pwm /
	 * platform_i2c 的 ops 注册已经做完, 这里只管拿 bus 句柄.
	 */
	can1_bus = platform_can_get_device(CAN1);
	can2_bus = platform_can_get_device(CAN2);
	if (!can1_bus) 
	{
		return -1;
	}
	if (!can2_bus) 
	{
		return -1;
	}
	
	rc = dv_motor_can_init(&s_moto_can1, can1_bus);
	if (rc != 0) 
	{
		return rc;
	}

	rc = dv_motor_can_init(&s_moto_can2, can2_bus);
	if (rc != 0) 
	{
		return rc;
	}
	
	return 0;
}



static void board_init(void)
{

	
	motor_board_init();

}


MODULE_INIT(board_init,INIT_LEVEL_DEVICE)


