#include "bus.h"
#include "device.h"
#include "rom_file.h"
#include "ri.h"
#include "vi.h"
#include "cpu.h"
#include "rom_file.h"

extern vr4300 *g_vr4300;

#define VI_COUNTER_START ((62500000.0 / 60.0) + 1)
#define VI_BLANKING_DONE (unsigned)((VI_COUNTER_START - VI_COUNTER_START / 525.0 * 39))


int read_vi_regs(void *opaque, uint32_t address, uint32_t *word)
{
    struct vi_controller *vi     = (struct vi_controller *)opaque;
    unsigned              offset = address - VI_REGS_BASE_ADDRESS;
    enum vi_register      reg    = (vi_register)(offset >> 2);
    vi->regs[VI_CURRENT_REG]     = 0;
    if (vi->regs[VI_V_SYNC_REG] >= 0x2) {
        vi->regs[VI_CURRENT_REG] =
            (VI_COUNTER_START - (vi->counter)) / (VI_COUNTER_START / (vi->regs[VI_V_SYNC_REG] >> 1));
        vi->regs[VI_CURRENT_REG] = (vi->regs[VI_CURRENT_REG] << 1);
        if (!(vi->regs[VI_V_SYNC_REG] & 0x1))
            vi->regs[VI_CURRENT_REG] |= vi->field;
    }
    *word = vi->regs[reg];
    debug_mmio_read(vi, vi_register_mnemonics[reg], *word);
    return 0;
}
void vi_cycle(struct vi_controller *vi)
{
    n64_gl_window         *window;
    size_t                 copy_size;
    unsigned               counter;
    struct render_area    *ra = &vi->render_area;
    struct bus_controller *bus;
    float                  hcoeff, vcoeff;
    counter = --(vi->counter);
    if (unlikely(counter == 0))
        vi->counter = VI_COUNTER_START;
    if (unlikely(counter == vi->intr_counter))
        g_vr4300->signal_rcp_interrupt(MI_INTR_VI);
    if (likely(counter != VI_BLANKING_DONE))
        return;
    vi->field   = !vi->field;
    window      = vi->window;
    ra->x.start = vi->regs[VI_H_START_REG] >> 16 & 0x3FF;
    ra->x.end   = vi->regs[VI_H_START_REG] & 0x3FF;
    ra->y.start = vi->regs[VI_V_START_REG] >> 16 & 0x3FF;
    ra->y.end   = vi->regs[VI_V_START_REG] & 0x3FF;
    hcoeff      = (float)(vi->regs[VI_X_SCALE_REG] & 0xFFF) / (1 << 10);
    vcoeff      = (float)(vi->regs[VI_Y_SCALE_REG] & 0xFFF) / (1 << 10);
    if (likely((long int)window)) {
        n64_mutex_lock(&window->event_mutex);
        if (unlikely(window->exit_requested)) {
            n64_mutex_unlock(&window->event_mutex);
            device_exit(vi->bus);
        }
        n64_mutex_unlock(&window->event_mutex);
        n64_mutex_lock(&window->render_mutex);
        window->frame_vres = ra->height = ((ra->y.end - ra->y.start) >> 1) * vcoeff;
        window->frame_hres = ra->width = ((ra->x.end - ra->x.start)) * hcoeff;
        window->frame_hskip = ra->hskip = vi->regs[VI_WIDTH_REG] - ra->width;
        window->frame_type              = vi->regs[VI_STATUS_REG] & 0x3;
        if (window->frame_hres <= 0 || window->frame_vres <= 0)
            window->frame_type = 0;
        copy_size = sizeof(bus->ri->ram) - (vi->regs[VI_ORIGIN_REG] & 0xFFFFFF);
        if (copy_size > sizeof(vi->window->frame_buffer))
            copy_size = sizeof(vi->window->frame_buffer);
        memcpy(&bus, vi, sizeof(bus));
        memcpy(vi->window->frame_buffer, bus->ri->ram + (vi->regs[VI_ORIGIN_REG] & 0xFFFFFF), copy_size);
        n64_mutex_unlock(&vi->window->render_mutex);
        n64_gl_window_push_frame(window);
    } else if (++(vi->frame_count) == 60) {
        n64_time current_time;
        float    ns;
        get_time(&current_time);
        ns                   = compute_time_difference(&current_time, &vi->last_update_time);
        vi->last_update_time = current_time;
        vi->frame_count      = 0;
        printf("VI/s: %.2f\n", (60 / (ns / NS_PER_SEC)));
    }
}
int vi_init(struct vi_controller *vi, struct bus_controller *bus, bool no_interface)
{
    vi->counter = VI_COUNTER_START;
    vi->bus     = bus;
    if (!no_interface) {
        if (vi_create_window(vi))
            return -1;
        gl_window_init(vi);
    }
    return 0;
}
int write_vi_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    struct vi_controller *vi     = (struct vi_controller *)opaque;
    unsigned              offset = address - VI_REGS_BASE_ADDRESS;
    enum vi_register      reg    = (vi_register)(offset >> 2);
    debug_mmio_write(vi, vi_register_mnemonics[reg], word, dqm);
    if (reg == VI_CURRENT_REG)
        g_vr4300->clear_rcp_interrupt(MI_INTR_VI);
    else if (reg == VI_INTR_REG) {
        vi->intr_counter = VI_COUNTER_START - VI_COUNTER_START / 525 * (word >> 1);
        vi->regs[reg] &= ~dqm;
        vi->regs[reg] |= word;
    } else {
        vi->regs[reg] &= ~dqm;
        vi->regs[reg] |= word;
    }
    return 0;
}
void gl_window_init(struct vi_controller *vi)
{
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_BLEND);
    glDisable(GL_DITHER);
    glEnable(GL_TEXTURE_2D);
    glBlendFunc(GL_ONE, GL_ONE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(2, GL_FLOAT, 0, vi->viuv);
    glVertexPointer(2, GL_FLOAT, 0, vi->quad);
    vi->quad[0] = vi->quad[5] = vi->quad[6] = vi->quad[7] = -1;
    vi->quad[1] = vi->quad[2] = vi->quad[3] = vi->quad[4] = 1;
    vi->viuv[2] = vi->viuv[4] = vi->viuv[5] = vi->viuv[7] = 1;
    glPixelStorei(GL_UNPACK_SWAP_BYTES, GL_TRUE);
}
void gl_window_render_frame(struct vi_controller *vi, const uint8_t *buffer, unsigned hres, unsigned vres,
                            unsigned hskip, unsigned type)
{
    float aspect;
    glClear(GL_COLOR_BUFFER_BIT);
    switch (type) {
        case 0:
            return;
        case 1:
            assert(0 && "Attempted to use reserved frame type.");
            return;
        case 2:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, hres + hskip, vres, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, buffer);
            break;
        case 3:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, hres + hskip, vres, 0, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
            break;
    }
    if ((vi->regs[VI_STATUS_REG] & 0x300) == 0x300) {
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    } else {
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }
    aspect      = (float)hres / (hres + hskip);
    vi->viuv[2] = vi->viuv[4] = aspect;
    glDrawArrays(GL_QUADS, 0, 4);
    if (vi->regs[VI_STATUS_REG] & 0x8) {
        glEnable(GL_BLEND);
        glColor3f(0.15f, 0.15f, 0.15f);
        glDrawArrays(GL_QUADS, 0, 4);
        glDisable(GL_TEXTURE_2D);
        glColor3f(0.1f, 0.1f, 0.1f);
        glDrawArrays(GL_QUADS, 0, 4);
        glDisable(GL_BLEND);
        glEnable(GL_TEXTURE_2D);
        glColor3f(1.0f, 1.0f, 1.0f);
    }
    n64_gl_window_swap_buffers(vi->window);
}
void gl_window_resize_cb(int width, int height)
{
    float aspect = 640.0 / 474.0;
    if (height <= 0)
        height = 1;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if ((float)width / (float)height > aspect) {
        aspect = 474.0 / 640.0;
        aspect *= (float)width / (float)height;
        glOrtho(-aspect, aspect, -1, 1, -1, 1);
    } else {
        aspect *= (float)height / (float)width;
        glOrtho(-1, 1, -aspect, aspect, -1, 1);
    }
    glClear(GL_COLOR_BUFFER_BIT);
}
int vi_create_window(struct vi_controller *vi)
{
    struct n64_gl_hints hints = n64_default_gl_hints;
    n64_gl_config      *config;
    int                 num_matching;
    if ((vi->display = n64_gl_display_create(NULL)) == CEN64_GL_DISPLAY_BAD)
        return -1;
    if ((vi->screen = n64_gl_screen_create(vi->display, -1)) == CEN64_GL_SCREEN_BAD) {
        n64_gl_display_destroy(vi->display);
        return -1;
    }
    if ((config = n64_gl_config_create(vi->display, vi->screen, &hints, &num_matching)) == CEN64_GL_CONFIG_BAD) {
        n64_gl_screen_destroy(vi->screen);
        n64_gl_display_destroy(vi->display);
        return -1;
    }
    vi->window = n64_gl_window_create(vi->display, vi->screen, config, "");
    n64_gl_config_destroy(config);
    if (vi->window == CEN64_GL_WINDOW_BAD) {
        n64_gl_screen_destroy(vi->screen);
        n64_gl_display_destroy(vi->display);
        return -1;
    }
    if ((vi->context = n64_gl_context_create(vi->window)) == CEN64_GL_CONTEXT_BAD) {
        n64_gl_window_destroy(vi->window);
        n64_gl_screen_destroy(vi->screen);
        n64_gl_display_destroy(vi->display);
        return -1;
    }
    n64_gl_window_unhide(vi->window);
    return 0;
}
void vi_destroy_window(struct vi_controller *vi)
{
    n64_gl_context_destroy(vi->context, vi->window);
    n64_gl_window_destroy(vi->window);
    n64_gl_screen_destroy(vi->screen);
    n64_gl_display_destroy(vi->display);
}
