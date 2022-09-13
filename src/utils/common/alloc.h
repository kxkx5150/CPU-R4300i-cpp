//
//

// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//
#ifndef CEN64_OS_COMMON_ALLOC
#define CEN64_OS_COMMON_ALLOC
#include "../common.h"
struct n64_mem
{
    size_t size;
    void  *ptr;
};
void  n64_alloc_cleanup(void);
int   n64_alloc_init(void);
void *n64_alloc(struct n64_mem *m, size_t size, bool exec);
void  n64_free(struct n64_mem *m);
#endif
