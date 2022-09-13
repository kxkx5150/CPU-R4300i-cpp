#ifndef CEN64_VI_CONTROLLER_H
#define CEN64_VI_CONTROLLER_H

#include "../../utils/common.h"
#include "gl_common.h"
#include "gl_context.h"
#include "gl_display.h"
#include "gl_screen.h"
#include "timer.h"


#define MAX_FRAME_DATA_SIZE (640 * 480 * 4)

struct bus_controller;
enum vi_register
{
    VI_STATUS_REG,
    VI_ORIGIN_REG,
    VI_WIDTH_REG,
    VI_INTR_REG,
    VI_CURRENT_REG,
    VI_BURST_REG,
    VI_V_SYNC_REG,
    VI_H_SYNC_REG,
    VI_LEAP_REG,
    VI_H_START_REG,
    VI_V_START_REG,
    VI_V_BURST_REG,
    VI_X_SCALE_REG,
    VI_Y_SCALE_REG,
    NUM_VI_REGISTERS
};
struct render_area
{
    struct
    {
        unsigned start;
        unsigned end;
    } x;
    struct
    {
        unsigned start;
        unsigned end;
    } y;
    unsigned height;
    unsigned width;
    int      hskip;
};
struct vi_controller
{
    struct bus_controller *bus;
    uint32_t               regs[NUM_VI_REGISTERS];
    uint32_t               counter;
    n64_gl_display         display;
    n64_gl_screen          screen;
    n64_gl_window         *window;
    n64_gl_context         context;
    struct render_area     render_area;
    float                  viuv[8];
    float                  quad[8];
    n64_time               last_update_time;
    unsigned               intr_counter;
    unsigned               frame_count;
    unsigned               field;
};
struct gl_window
{
    void *window;
};
struct gl_window_hints
{
    unsigned width, height;
    char     fullscreen;
    char     double_buffered;
    char     color_bits;
    char     alpha_bits;
    char     depth_bits;
    char     stencil_bits;
    char     accum_color_bits;
    char     accum_alpha_bits;
    char     auxiliary_buffers;
};

int  vi_init(struct vi_controller *vi, struct bus_controller *bus, bool no_interface);
void vi_cycle(struct vi_controller *vi);
int  read_vi_regs(void *opaque, uint32_t address, uint32_t *word);
int  write_vi_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);

void gl_window_init(struct vi_controller *vi);
void gl_window_render_frame(struct vi_controller *vi, const uint8_t *buffer, unsigned hres, unsigned vres,
                            unsigned hskip, unsigned type);
void gl_window_resize_cb(int width, int height);

int  vi_create_window(struct vi_controller *vi);
void vi_destroy_window(struct vi_controller *vi);

void get_default_gl_window_hints(struct gl_window_hints *hints);
int  destroy_gl_window(struct gl_window *window);
int  create_gl_window(struct bus_controller *bus, struct gl_window *window, const struct gl_window_hints *hints);
int  gl_window_thread(struct gl_window *window, struct bus_controller *bus);
int  gl_swap_buffers(const struct gl_window *window);
void gl_window_resize_cb(int width, int height);
#endif
