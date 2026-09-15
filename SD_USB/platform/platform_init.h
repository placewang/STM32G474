/* SPDX-License-Identifier: MIT */
/**
 * @file  platform_init.h
 * @brief platform 层 ops 表注册入口 (PC 端)
 *
 * @details
 * STM32 端这一步在 platform/arch/stm32/{pin,pwm,i2c}_board.c 的
 * platform_hw_xxx_init 
 * platform_init() 把两个一次调完.
 *
 * 启动顺序 (main): platform_init() -> led_board_init() -> 应用代码.
 *
 * platform_gpio (common/platform_pc.c) 是 ch01-ch14 一路用下来的
 */

#ifndef PLATFORM_INIT_H
#define PLATFORM_INIT_H





void platform_init(void);

#endif /* PLATFORM_INIT_H */



