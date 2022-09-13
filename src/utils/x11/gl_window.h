#ifndef CEN64_OS_X11_GL_WINDOW
#define CEN64_OS_X11_GL_WINDOW
#include "../common.h"
#include "gl_common.h"
#include "gl_config.h"
#include "gl_display.h"
#include "gl_screen.h"
#include "thread.h"
#include <unistd.h>
#include <X11/Xlib.h>

#define FRAMEBUF_SZ         (640 * 474 * 4)
#define CEN64_GL_WINDOW_BAD (NULL)

struct n64_gl_window
{
    n64_gl_display       display;
    n64_gl_screen        screen;
    Window               window;
    Atom                 wm_delete_window;
    XSetWindowAttributes attr;
    XVisualInfo         *visual_info;
    int                  pipefds[2];
    n64_mutex            render_mutex;
    uint8_t              frame_buffer[FRAMEBUF_SZ];
    unsigned             frame_hres, frame_vres;
    unsigned             frame_hskip, frame_type;
    n64_mutex            event_mutex;
    bool                 exit_requested;
};

struct n64_device;

int            n64_gl_window_thread(struct n64_device *device);
n64_gl_window *n64_gl_window_create(n64_gl_display display, n64_gl_screen screen, const n64_gl_config *config,
                                    const char *title);

static inline void n64_gl_window_destroy(n64_gl_window *window)
{
    XDestroyWindow(window->display, window->window);
    XFreeColormap(window->display, window->attr.colormap);
    XFree(window->visual_info);
    close(window->pipefds[0]);
    close(window->pipefds[1]);
    n64_mutex_destroy(&window->render_mutex);
    n64_mutex_destroy(&window->event_mutex);
    free(window);
}
static inline void n64_gl_window_push_frame(n64_gl_window *window)
{
    auto _ = write(window->pipefds[1], &window, sizeof(window));
}
static inline void n64_gl_window_set_title(n64_gl_window *window, const char *title)
{
    XStoreName(window->display, window->window, title);
    XFlush(window->display);
}
static inline void n64_gl_window_swap_buffers(n64_gl_window *window)
{
    glXSwapBuffers(window->display, window->window);
}
static inline void n64_gl_window_unhide(n64_gl_window *window)
{
    XMapRaised(window->display, window->window);
    XFlush(window->display);
}
#endif
