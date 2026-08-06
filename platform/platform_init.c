/* SPDX-License-Identifier: MIT */
/*
 * platform_init.c - platform 层 ops 表注册
 * 注册, 注册的 ops 表不同而已.
 */


#include "initcall.h"
#include "platform_init.h"

extern void platform_hw_can_init(void);


void platform_init(void)
{
	platform_hw_can_init();
}

MODULE_INIT(platform_init,INIT_LEVEL_CORE)






