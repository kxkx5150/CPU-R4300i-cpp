#ifndef CEN64_OS_X11_GL_DISPLAY
#define CEN64_OS_X11_GL_DISPLAY
#include "gl_common.h"
#include <stddef.h>
#include <X11/Xlib.h>

#define CEN64_GL_DISPLAY_BAD (NULL)

typedef Display *n64_gl_display;

static inline n64_gl_display n64_gl_display_create(const char *source)
{
    return XOpenDisplay(source);
}
static inline void n64_gl_display_destroy(n64_gl_display display)
{
    XCloseDisplay(display);
}
static inline int n64_gl_display_get_num_screens(n64_gl_display display)
{
    return XScreenCount(display);
}
#endif
