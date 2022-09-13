#include "../common.h"
#include "gl_common.h"
#include "gl_config.h"
#include "gl_hints.h"

n64_gl_config *n64_gl_config_create(n64_gl_display display, n64_gl_screen screen, const n64_gl_hints *hints,
                                    int *matching)
{
    int idx = 0;
    int attribute_list[64];
    attribute_list[idx++] = GLX_X_RENDERABLE;
    attribute_list[idx++] = True;
    attribute_list[idx++] = GLX_X_VISUAL_TYPE;
    attribute_list[idx++] = GLX_TRUE_COLOR;
    attribute_list[idx++] = GLX_RENDER_TYPE;
    attribute_list[idx++] = hints->context_type;
    attribute_list[idx++] = GLX_DRAWABLE_TYPE;
    attribute_list[idx++] = hints->drawable_type;
    if (hints->double_buffered != -1) {
        attribute_list[idx++] = GLX_DOUBLEBUFFER;
        attribute_list[idx++] = hints->double_buffered ? True : False;
    }
    if (hints->stereoscopic != -1) {
        attribute_list[idx++] = GLX_STEREO;
        attribute_list[idx++] = hints->stereoscopic ? True : False;
    }
    if (hints->rgb_color_depth != -1) {
        int component_depth   = hints->rgb_color_depth > 0 ? hints->rgb_color_depth / 3 : 0;
        attribute_list[idx++] = GLX_RED_SIZE;
        attribute_list[idx++] = component_depth;
        attribute_list[idx++] = GLX_GREEN_SIZE;
        attribute_list[idx++] = component_depth;
        attribute_list[idx++] = GLX_BLUE_SIZE;
        attribute_list[idx++] = component_depth;
    }
    if (hints->alpha_color_depth != -1) {
        attribute_list[idx++] = GLX_ALPHA_SIZE;
        attribute_list[idx++] = hints->alpha_color_depth;
    }
    if (hints->depth_buffer_size != -1) {
        attribute_list[idx++] = GLX_DEPTH_SIZE;
        attribute_list[idx++] = hints->depth_buffer_size;
    }
    if (hints->num_aux_buffers != -1) {
        attribute_list[idx++] = GLX_AUX_BUFFERS;
        attribute_list[idx++] = hints->num_aux_buffers;
    }
    if (hints->stencil_buffer_size != -1) {
        attribute_list[idx++] = GLX_STENCIL_SIZE;
        attribute_list[idx++] = hints->stencil_buffer_size;
    }
    if (hints->accum_buffer_red_bits != -1) {
        attribute_list[idx++] = GLX_ACCUM_RED_SIZE;
        attribute_list[idx++] = hints->accum_buffer_red_bits;
    }
    if (hints->accum_buffer_green_bits != -1) {
        attribute_list[idx++] = GLX_ACCUM_GREEN_SIZE;
        attribute_list[idx++] = hints->accum_buffer_green_bits;
    }
    if (hints->accum_buffer_blue_bits != -1) {
        attribute_list[idx++] = GLX_ACCUM_BLUE_SIZE;
        attribute_list[idx++] = hints->accum_buffer_blue_bits;
    }
    if (hints->accum_buffer_alpha_bits != -1) {
        attribute_list[idx++] = GLX_ACCUM_ALPHA_SIZE;
        attribute_list[idx++] = hints->accum_buffer_alpha_bits;
    }
    attribute_list[idx++] = None;
    return glXChooseFBConfig(display, screen, attribute_list, matching);
}
int n64_gl_config_fetch_attribute(n64_gl_display display, n64_gl_config *config, int what)
{
    int value, status;
    status = glXGetFBConfigAttrib(display, *config, what, &value);
    if (status == GLX_NO_EXTENSION || status == GLX_BAD_ATTRIBUTE)
        return -1;
    return value;
}
