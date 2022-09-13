#ifndef CEN64_OS_X11_GL_SCREEN
#define CEN64_OS_X11_GL_SCREEN
#include "gl_common.h"
#include <X11/Xlib.h>

#define CEN64_GL_SCREEN_BAD (-1)

typedef int n64_gl_screen;

static inline n64_gl_screen n64_gl_screen_create(n64_gl_display display, int which)
{
    if (which >= 0)
        return XScreenCount(display) > which ? which : CEN64_GL_SCREEN_BAD;
    return XDefaultScreen(display);
}
static inline void n64_gl_screen_destroy(n64_gl_screen screen)
{
}
#endif
