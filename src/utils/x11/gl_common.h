#ifndef CEN64_OS_X11_GL_COMMON
#define CEN64_OS_X11_GL_COMMON
#include "../common.h"
#include <GL/gl.h>
#include <GL/glx.h>

enum n64_gl_context_type
{
    CEN64_GL_CONTEXT_TYPE_RGBA        = GLX_RGBA_BIT,
    CEN64_GL_CONTEXT_TYPE_COLOR_INDEX = GLX_COLOR_INDEX_BIT
};
enum n64_gl_drawable_type
{
    CEN64_GL_DRAWABLE_TYPE_WINDOW = GLX_WINDOW_BIT,
    CEN64_GL_DRAWABLE_TYPE_BITMAP = GLX_PIXMAP_BIT
};
enum n64_gl_layer_type
{
    CEN64_GL_LAYER_TYPE_DEFAULT  = 0,
    CEN64_GL_LAYER_TYPE_OVERLAY  = 1,
    CEN64_GL_LAYER_TYPE_UNDERLAY = 2
};
#endif
