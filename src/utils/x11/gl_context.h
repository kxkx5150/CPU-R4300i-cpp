#ifndef CEN64_OS_X11_GL_CONTEXT
#define CEN64_OS_X11_GL_CONTEXT
#include "gl_common.h"
#include "gl_display.h"
#include "gl_screen.h"
#include "gl_window.h"
#include <stddef.h>

#define CEN64_GL_CONTEXT_BAD (NULL)
typedef GLXContext n64_gl_context;

static inline n64_gl_context n64_gl_context_create(n64_gl_window *window)
{
    n64_gl_context c;
    if ((c = glXCreateContext(window->display, window->visual_info, NULL, True)) == CEN64_GL_CONTEXT_BAD)
        return CEN64_GL_CONTEXT_BAD;
    if (glXMakeCurrent(window->display, window->window, c) != True) {
        glXDestroyContext(window->display, c);
        c = CEN64_GL_CONTEXT_BAD;
    }
    return c;
}
static inline void n64_gl_context_destroy(n64_gl_context context, n64_gl_window *window)
{
    glXMakeCurrent(window->display, None, NULL);
    glXDestroyContext(window->display, context);
}
#endif
