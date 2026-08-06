/* SPDX-License-Identifier: MIT */
/*
 * initcall.h - Linux 内核 initcall 机制 (Keil AC6 移植版)
 *
 * 核心问题：每加一个驱动就得改 main.c 加一行 xxx_init() 调用
 * 解决方案：让驱动文件"自己注册自己"，main 永远不动
 *
 *   1. __attribute__((constructor))：告诉编译器，这个函数在 main() 之前
 *      由 C 运行时库自动调用，实现"编译期自动注册"
 *   2. 注册函数只存函数指针，不执行：把真正的执行延迟到 do_initcalls()
 *   3. do_initcalls() 由 main 主动调用，控制执行时机
 * 加新驱动 = 写新文件 + 一行 MODULE_INIT(xxx_init)，main.c 不需要 #include
 * 任何驱动头文件，也不需要知道任何驱动名字
 *
 * 注意：Linux 内核用 __attribute__((section)) + 链接器段收集来实现，
 * 但那需要修改链接脚本。Keil AC6 下使用 __attribute__((constructor))
 * 更简单，效果相同，且不需要动链接脚本。
 */
#ifndef INITCALL_H
#define INITCALL_H

#include <stdint.h>

/*
 * initcall_t - 驱动初始化函数的类型别名
 * 所有用 MODULE_INIT 注册的函数都必须是 void (void) 签名。
 * 如果想支持返回值（成功/失败）可以改为 int (*)(void)
 */
typedef void (*initcall_t)(void);
/* 定义优先级常量：数字越小，执行越早 */
#define INIT_LEVEL_EARLY   0   
#define INIT_LEVEL_CORE    1   
#define INIT_LEVEL_DEVICE  2   
#define INIT_LEVEL_LATE    3   

/*
 * 最大驱动数
 * 32 个槽位
 * 如果不够，改大这个数字即可。
 */
#define MAX_INITCALLS 128


/* 全局注册表
 * initcall_table[]：存放所有驱动初始化函数的指针
 * initcall_count：当前已注册的驱动数量
 *   - 0（默认）：只存指针，不执行（延迟到 do_initcalls）
 *   - 1：注册时立即执行（用于特殊场景一般不用）
 * 
 *   因为我们要在 main 中单步调试，控制执行时机。
 *   如果设为 1，构造函数在 main 前就执行了，无法手 run 调试
 */
extern initcall_t initcall_table[MAX_INITCALLS];
extern int initcall_levels[MAX_INITCALLS];
extern int initcall_count;

/*
 * MODULE_INIT(fn) - 驱动"自己注册自己"的核心宏
 * 宏展开详解
 * 假设驱动文件 drv_tim.c 中写：
 *   static void tim_init(void) { ... }
 *   MODULE_INIT(tim_init);
 * 核心技术点
 * 1. __attribute__((constructor))
 *    - GCC/AC6 编译器扩展，告诉编译器"这个函数需要在 main() 之前执行"
 *    - C 运行时库（__main）会在进入 main 之前遍历所有 constructor 函数
 *    - 每个驱动文件编译时，MODULE_INIT 都会生成一个 constructor 函数
 *    - 编译器自动收集所有 constructor 函数，形成一张表
 *    - 运行时，C 库依次调用表里的每个函数
 * 
 * 2. 静态函数 __reg_##fn
 *    - 用 ## 拼接生成唯一函数名：__reg_ + 原函数名
 *    - static 限制作用域在当前文件，避免多个驱动撞名
 *    - 每个驱动文件生成自己的 __reg_xxx 函数，互不干扰
 * 
 * 4. 为什么不需要链接脚本？
 *    - Linux 内核用 __attribute__((section)) 把函数指针塞进特殊段
 *    - 需要链接器生成 __start_xxx / __stop_xxx 边界符号
 *    - Keil AC6 的 __attribute__((constructor)) 由编译器/运行时处理
 *    - 不需要修改任何链接脚本，即插即用
 * 与 Linux 内核 initcall 的对比
 * Linux 内核方式：
 *   #define module_init(fn) \
 *       static initcall_t __initcall_##fn \
 *           __attribute__((section(".initcall6.init"))) = fn;
 *   - 依赖链接器收集段，生成边界符号
 *   - do_initcalls 遍历段区间，挨个调用
 *   - 完全在编译/链接期完成，运行时零开销
 * 
 * 本方案（Keil AC6 移植版）：
 *   #define MODULE_INIT(fn) \
 *       static void __reg_##fn(void) __attribute__((constructor)) { ... }
 *   - 依赖编译器 constructor 机制
 *   - 构造函数在 main 前执行，完成注册
 *   - do_initcalls 遍历数组，执行真正的初始化
 *   - 牺牲了一点运行时开销（注册表遍历），换来了无需改链接脚本
 * 两种方案异曲同工：main.c 都不知道驱动存在，驱动自己把自己挂上去。
 */
#define MODULE_INIT(fn, level)                                             \
    static void __attribute__((constructor)) __reg_##fn(void)              \
    {                                                                      \
        if (initcall_count < MAX_INITCALLS) {                              \
            initcall_table[initcall_count] = fn;                           \
            initcall_levels[initcall_count] = level;                       \
            initcall_count++;                                              \
        }                                                                  \
    }
/*
 * do_initcalls() - 执行所有已注册的驱动初始化函数
 * 1. main() 调用 do_initcalls()
 * 2. 遍历 initcall_table[0] 到 initcall_table[initcall_count - 1]
 * 3. 对每个非空指针，调用它指向的函数
 * 4. 所有驱动初始化完成，返回 main
 * 
 *   - 所有驱动初始化在 main 之前完成
 *   - 调试时一上电就全跑完了，没法单步跟踪
 *   - 无法控制初始化顺序，出问题不好定位
 *
 * 当前设计（constructor 只注册，do_initcalls 才执行）：
 *   - 上电后构造函数先跑一遍，把函数指针收集到表中
 *   - 然后在 main 中设断点，手动调用 do_initcalls()
 *   - 可以单步进入每个驱动初始化函数，观察执行过程
 *   - 发现问题可以立即停止，方便调试
 *
 * "注册与执行分离"的设计适合在嵌入式调试。
 * 产品发布时，可以在 main 开头直接调用 do_initcalls()，效果一样。
 */
void do_initcalls(void);

#endif
