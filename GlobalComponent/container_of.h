/* SPDX-License-Identifier: MIT */

#ifndef MY_CONTAINER_OF_H
#define MY_CONTAINER_OF_H

#include <stddef.h>

#define container_of(ptr, type, member)					\
	  ((type *)((uintptr_t)(ptr) - offsetof(type, member)))





#endif

