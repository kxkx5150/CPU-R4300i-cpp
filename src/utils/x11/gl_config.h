#ifndef CEN64_OS_X11_GL_CONFIG
#define CEN64_OS_X11_GL_CONFIG
#include "gl_common.h"
#include "gl_display.h"
#include "gl_hints.h"
#include "gl_screen.h"
#include <stddef.h>
#include <X11/Xlib.h>

#define CEN64_GL_CONFIG_BAD (NULL)
typedef GLXFBConfig n64_gl_config;

n64_gl_config *n64_gl_config_create(n64_gl_display display, n64_gl_screen screen, const n64_gl_hints *hints,
                                    int *matching);

static inline void n64_gl_config_destroy(n64_gl_config *config)
{
    XFree(config);
}
int n64_gl_config_fetch_attribute(n64_gl_display display, n64_gl_config *config, int what);
static inline enum n64_gl_context_type n64_gl_config_get_context_type(n64_gl_display display, n64_gl_config *config)
{
    return (n64_gl_context_type)n64_gl_config_fetch_attribute(display, config, GLX_RENDER_TYPE);
}
static inline enum n64_gl_drawable_type n64_gl_config_get_drawable_type(n64_gl_display display, n64_gl_config *config)
{
    return (n64_gl_drawable_type)n64_gl_config_fetch_attribute(display, config, GLX_DRAWABLE_TYPE);
}
static inline enum n64_gl_layer_type n64_gl_config_get_layer_type(n64_gl_display display, n64_gl_config *config)
{
    int level = n64_gl_config_fetch_attribute(display, config, GLX_LEVEL);
    if (level < 0)
        return CEN64_GL_LAYER_TYPE_UNDERLAY;
    else if (level > 0)
        return CEN64_GL_LAYER_TYPE_OVERLAY;
    return CEN64_GL_LAYER_TYPE_DEFAULT;
}
static inline int n64_gl_config_is_double_buffered(n64_gl_display display, n64_gl_config *config)
{
    return n64_gl_config_fetch_attribute(display, config, GLX_DOUBLEBUFFER) == True;
}
static inline int n64_gl_config_is_renderable(n64_gl_display display, n64_gl_config *config)
{
    return n64_gl_config_fetch_attribute(display, config, GLX_X_RENDERABLE) == True;
}
static inline int n64_gl_config_is_stereoscopic(n64_gl_display display, n64_gl_config *config)
{
    return n64_gl_config_fetch_attribute(display, config, GLX_STEREO) == True;
}
static inline int n64_gl_config_get_color_depth(n64_gl_display display, n64_gl_config *config)
{
    return n64_gl_config_fetch_attribute(display, config, GLX_BUFFER_SIZE);
}
static inline int n64_gl_config_get_red_color_depth(n64_gl_display display, n64_gl_config *config)
{
    return n64_gl_config_fetch_attribute(display, config, GLX_RED_SIZE);
}
static inline int n64_gl_config_get_green_color_depth(n64_gl_display display, n64_gl_config *config)
{
    return n64_gl_config_fetch_attribute(display, config, GLX_GREEN_SIZE);
}
static inline int n64_gl_config_get_blue_color_depth(n64_gl_display display, n64_gl_config *config)
{
    return n64_gl_config_fetch_attribute(display, config, GLX_BLUE_SIZE);
}
static inline int n64_gl_config_get_alpha_color_depth(n64_gl_display display, n64_gl_config *config)
{
    return n64_gl_config_fetch_attribute(display, config, GLX_ALPHA_SIZE);
}
static inline int n64_gl_config_get_depth_buffer_count(n64_gl_display display, n64_gl_config *config)
{
    return n64_gl_config_fetch_attribute(display, config, GLX_DEPTH_SIZE);
}
static inline int n64_gl_config_get_num_auxiliary_buffers(n64_gl_display display, n64_gl_config *config)
{
    return n64_gl_config_fetch_attribute(display, config, GLX_AUX_BUFFERS);
}
static inline int n64_gl_config_get_stencil_buffer_size(n64_gl_display display, n64_gl_config *config)
{
    return n64_gl_config_fetch_attribute(display, config, GLX_STENCIL_SIZE);
}
static inline int n64_gl_config_get_red_accum_buffer_bits(n64_gl_display display, n64_gl_config *config)
{
    return n64_gl_config_fetch_attribute(display, config, GLX_ACCUM_RED_SIZE);
}
static inline int n64_gl_config_get_blue_accum_buffer_bits(n64_gl_display display, n64_gl_config *config)
{
    return n64_gl_config_fetch_attribute(display, config, GLX_ACCUM_BLUE_SIZE);
}
static inline int n64_gl_config_get_green_accum_buffer_bits(n64_gl_display display, n64_gl_config *config)
{
    return n64_gl_config_fetch_attribute(display, config, GLX_ACCUM_GREEN_SIZE);
}
static inline int n64_gl_config_get_alpha_accum_buffer_bits(n64_gl_display display, n64_gl_config *config)
{
    return n64_gl_config_fetch_attribute(display, config, GLX_ACCUM_ALPHA_SIZE);
}
#endif
