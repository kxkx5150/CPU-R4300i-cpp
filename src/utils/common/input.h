//
//

// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//
#ifndef CEN64_OS_COMMON_INPUT
#define CEN64_OS_COMMON_INPUT
#include "../common.h"
struct bus_controller;
void keyboard_press_callback(struct bus_controller *bus, unsigned key);
void keyboard_release_callback(struct bus_controller *bus, unsigned key);
#endif
