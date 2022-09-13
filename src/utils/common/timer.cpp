//
//

// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//
#include "../common.h"
#include "timer.h"
#include <time.h>
#include <sys/time.h>
// NS_PER_USEC is not defined on OS X
#ifndef NS_PER_USEC
#define NS_PER_USEC 1000
#endif
// Computes the difference, in ns, between two times.
unsigned long long compute_time_difference(const n64_time *now, const n64_time *before)
{
#if defined(__APPLE__)
    return (now->tv_sec - before->tv_sec) * NS_PER_SEC + (now->tv_usec - before->tv_usec) * NS_PER_USEC;
#else
    return (now->tv_sec - before->tv_sec) * NS_PER_SEC + (now->tv_nsec - before->tv_nsec);
#endif
}
// Gets the time from the most monotonic source possible.
void get_time(n64_time *t)
{
#if defined(__APPLE__)
    gettimeofday(t, NULL);
#else
    clock_gettime(GETTIME_SOURCE, t);
#endif
}
