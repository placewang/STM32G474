/* SPDX-License-Identifier: MIT */

/*
 * initcall.c - 启动期执行所有已注册的驱动初始化函数 (Keil AC6 版)
 *
 * 配合 initcall.h 的"注册与执行分离"设计
 * 
 * 工作流程：
 *   1. 编译期：每个 MODULE_INIT(fn) 生成一个 __attribute__((constructor))
 *      函数，在 main() 之前被 C 运行时库自动调用
 *   2. 构造函数阶段：所有驱动的函数指针被收集到 initcall_table[] 数组中
 *   3. main() 中：调用 do_initcalls()，遍历数组，逐个执行真正的初始化
 */

#include "initcall.h"
#include <stdio.h>


/*
 * 全局注册表定义
 * 这些变量在 .data 段（已初始化数据）不依赖任何链接脚本
 */

initcall_t initcall_table[MAX_INITCALLS];
int initcall_levels[MAX_INITCALLS];
int initcall_count = 0;

/* 简单的冒泡排序：按 level 升序排列 */
static void sort_initcalls(void)
{
	int tmp_level=0;
	
	for (int i = 0; i < initcall_count - 1; i++) 
	{
        for (int j = 0; j < initcall_count - 1 - i; j++)
		{
            if (initcall_levels[j] > initcall_levels[j + 1])
			{
                /* 交换函数指针 */
                initcall_t tmp_fn = initcall_table[j];
                initcall_table[j] = initcall_table[j + 1];
                initcall_table[j + 1] = tmp_fn;                
                /* 交换优先级 */
                tmp_level = initcall_levels[j];
                initcall_levels[j] = initcall_levels[j + 1];
                initcall_levels[j + 1] = tmp_level;
            }
        }
    }
}

/*
 * do_initcalls() - 执行所有已注册的驱动初始化函数
 * 1. 从 initcall_table[0] 开始遍历
 * 2. 遇到非空指针就调用它指向的函数
 * 3. 共执行 initcall_count 个函数
 * 仿真中
 *   1. 在 do_initcalls() 入口设断点
 *   2. 单步执行，观察 initcall_table[] 数组内容
 *   3. 查看 initcall_count 是否等于实际驱动数量
 *   4. 单步进入每个驱动初始化函数，验证执行顺序和正确性
 * 如果某个驱动初始化失败（比如硬件没响应）：
 *   1. 查看 initcall_table[] 中对应的函数指针
 *   2. 在函数入口设断点，重新调试
 *   3. 逐步跟踪，定位失败原因
 * 对于 MCU 项目，initcall_count 通常 < MAX_INITCALLS，遍历开销可以忽略
 * 如果追求极致性能（比如启动时间要求极高），可以：
 * 调试阶段使用当前方案(注册与执行分离)
 */

void do_initcalls(void)
{
    /* 先排序 */
    sort_initcalls();
    /*执行 */
    for (int i = 0; i < initcall_count; i++) 
	{
        if (initcall_table[i] != NULL)
		{// 调用驱动初始化函数
            initcall_table[i]();  
        }
    }
}
