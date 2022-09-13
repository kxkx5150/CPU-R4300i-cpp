//
//

// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//
#ifndef CEN64_OS_POSIX_TIMER
#define CEN64_OS_POSIX_TIMER
#include "../common.h"
#define NS_PER_SEC 1000000000ULL
#if defined(CLOCK_MONOTONIC_PRECISE)
#define GETTIME_SOURCE CLOCK_MONOTONIC_PRECISE
#else
#define GETTIME_SOURCE CLOCK_MONOTONIC_RAW
#endif
#ifdef __APPLE__
#include <time.h>
typedef struct timeval n64_time;
#else
#include <time.h>
typedef struct timespec n64_time;
#endif
unsigned long long compute_time_difference(const n64_time *now, const n64_time *before);
void               get_time(n64_time *t);
#endif
