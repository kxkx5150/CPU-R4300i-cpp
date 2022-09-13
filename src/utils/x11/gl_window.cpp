#include "../common.h"
#include "device.h"
#include "gl_common.h"
#include "gl_config.h"
#include "gl_display.h"
#include "gl_screen.h"
#include "gl_window.h"
#include "input.h"
#include "timer.h"
#include "vi.h"
#include <unistd.h>
#include <X11/Xlib.h>

static int  n64_gl_window_create_objects(n64_gl_window *window);
static bool n64_gl_window_pump_events(struct vi_controller *vi);

n64_gl_window *n64_gl_window_create(n64_gl_display display, n64_gl_screen screen, const n64_gl_config *config,
                                    const char *title)
{
    n64_gl_window *window;
    if ((window = (n64_gl_window *)malloc(sizeof(*window))) == NULL)
        return CEN64_GL_WINDOW_BAD;
    if ((window->visual_info = glXGetVisualFromFBConfig(display, *config)) == NULL) {
        free(window);
        return CEN64_GL_WINDOW_BAD;
    }
    if (n64_gl_window_create_objects(window)) {
        XFree(window->visual_info);
        free(window);
        return CEN64_GL_WINDOW_BAD;
    }
    window->attr.colormap =
        XCreateColormap(display, XRootWindow(display, screen), window->visual_info->visual, AllocNone);
    window->attr.event_mask = ExposureMask | KeyPressMask | KeyReleaseMask | ButtonPressMask | StructureNotifyMask;
    window->window =
        XCreateWindow(display, XRootWindow(display, screen), 0, 0, 640, 474, 0, window->visual_info->depth, InputOutput,
                      window->visual_info->visual, CWBorderPixel | CWColormap | CWEventMask, &window->attr);
    XSetStandardProperties(display, window->window, title, NULL, None, NULL, 0, NULL);
    window->wm_delete_window = XInternAtom(display, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(display, window->window, &window->wm_delete_window, 1);
    window->exit_requested = false;
    window->display        = display;
    window->screen         = screen;
    return window;
}
bool n64_gl_window_pump_events(struct vi_controller *vi)
{
    bool   released, exit_requested = false;
    XEvent event;
    if (!XPending(vi->display))
        return false;
    n64_mutex_lock(&vi->window->event_mutex);
    do {
        XNextEvent(vi->display, &event);
        switch (event.type) {
            case ClientMessage:
                vi->window->exit_requested = exit_requested = true;
                break;
            case ConfigureNotify:
                gl_window_resize_cb(event.xconfigure.width, event.xconfigure.height);
                break;
            case KeyPress:
                keyboard_press_callback(vi->bus, XLookupKeysym(&event.xkey, 0));
                break;
            case KeyRelease:
                released = true;
                if (XEventsQueued(vi->display, QueuedAfterReading)) {
                    XEvent next_event;
                    XPeekEvent(vi->display, &next_event);
                    if (next_event.type == KeyPress && next_event.xkey.time == event.xkey.time &&
                        next_event.xkey.keycode == event.xkey.keycode) {
                        XNextEvent(vi->display, &event);
                        released = false;
                    }
                }
                if (released)
                    keyboard_release_callback(vi->bus, XLookupKeysym(&event.xkey, 0));
                break;
        }
    } while (XPending(vi->display));
    n64_mutex_unlock(&vi->window->event_mutex);
    return exit_requested;
}
int n64_gl_window_create_objects(n64_gl_window *window)
{
    if (n64_mutex_create(&window->event_mutex)) {
        return 1;
    }
    if (n64_mutex_create(&window->render_mutex)) {
        n64_mutex_destroy(&window->event_mutex);
        return 1;
    }
    if (pipe(window->pipefds) < 0) {
        n64_mutex_destroy(&window->render_mutex);
        n64_mutex_destroy(&window->event_mutex);
        return 1;
    }
    return 0;
}
int n64_gl_window_thread(struct n64_device *device)
{
    struct vi_controller *vi = &device->vi;
    n64_time              last_update_time;
    n64_gl_window        *window;
    unsigned              frame_count;
    int                   max_fds, x11_fd;
    fd_set                fdset;
    x11_fd  = ConnectionNumber(vi->display);
    max_fds = x11_fd > vi->window->pipefds[0] ? x11_fd : vi->window->pipefds[0];
    max_fds++;
    FD_ZERO(&fdset);
    FD_SET(vi->window->pipefds[0], &fdset);
    FD_SET(x11_fd, &fdset);
    for (frame_count = 0, get_time(&last_update_time);;) {
        fd_set ready_to_read = fdset;
        if (select(max_fds, &ready_to_read, NULL, NULL, NULL) > 0) {
            if (unlikely(n64_gl_window_pump_events(vi)))
                break;
            if (FD_ISSET(vi->window->pipefds[0], &ready_to_read)) {
                auto _ = read(vi->window->pipefds[0], &window, sizeof(window));
                n64_mutex_lock(&window->render_mutex);
                gl_window_render_frame(vi, window->frame_buffer, window->frame_hres, window->frame_vres,
                                       window->frame_hskip, window->frame_type);
                n64_mutex_unlock(&window->render_mutex);
                if (++frame_count == 60) {
                    char     title[128];
                    n64_time current_time;
                    float    ns;
                    get_time(&current_time);
                    ns               = compute_time_difference(&current_time, &last_update_time);
                    last_update_time = current_time;
                    frame_count      = 0;
                    n64_gl_window_set_title(window, "");
                }
            }
        }
    }
    return 0;
}
