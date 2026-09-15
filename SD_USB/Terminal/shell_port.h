#ifndef SHELL__PORT__H
#define SHELL__PORT__H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wgnu-zero-variadic-macro-arguments"
#  pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#  pragma clang diagnostic ignored "-Wcast-function-type"
#  pragma clang diagnostic ignored "-Wpadded"
#endif

#include "./letter-shell3.x/src/shell.h"
#include "./letter-shell3.x/extensions/log/log.h"












extern void shell_process(void);






#endif 










