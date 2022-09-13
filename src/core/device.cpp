#include <cstddef>
#include <cstdint>
#include <setjmp.h>
#include <limits.h>
#include "device.h"

vr4300 *g_vr4300 = nullptr;

int        angrylion_rdp_init(struct n64_device *device);
static int device_spin(struct n64_device *device);

struct n64_device *device_create(struct n64_device *device, const struct rom_file *ddipl,
                                 const struct dd_variant *dd_variant, const struct rom_file *ddrom,
                                 const struct rom_file *pifrom, const struct rom_file *cart,
                                 const struct save_file *eeprom, const struct save_file *sram,
                                 const struct save_file *flashram, struct is_viewer *is,
                                 const struct controller *controller, bool no_audio, bool no_video, bool profiling)
{
    g_vr4300 = new vr4300();
    if ((device->vr4300 = g_vr4300) == NULL) {
        return NULL;
    }

    device->bus.ai     = &device->ai;
    device->bus.dd     = &device->dd;
    device->bus.pi     = &device->pi;
    device->bus.ri     = &device->ri;
    device->bus.si     = &device->si;
    device->bus.vi     = &device->vi;
    device->bus.rdp    = &device->rdp;
    device->bus.rsp    = &device->rsp;
    device->bus.vr4300 = device->vr4300;
    if (bus_init(&device->bus, dd_variant != NULL)) {
        return NULL;
    }
    if (ai_init(&device->ai, &device->bus, no_audio)) {
        return NULL;
    }
    if (dd_init(&device->dd, &device->bus, (uint8_t *)ddipl->ptr, (uint8_t *)ddrom->ptr, ddrom->size)) {
        return NULL;
    }
    if (pi_init(&device->pi, &device->bus, (uint8_t *)cart->ptr, cart->size, sram, flashram, is)) {
        return NULL;
    }
    if (ri_init(&device->ri, &device->bus)) {
        return NULL;
    }
    if (si_init(&device->si, &device->bus, (uint8_t *)pifrom->ptr, (uint8_t *)cart->ptr, dd_variant,
                (uint8_t *)eeprom->ptr, eeprom->size, controller)) {
        return NULL;
    }
    if (vi_init(&device->vi, &device->bus, no_video)) {
        return NULL;
    }
    if (rdp_init(&device->rdp, &device->bus)) {
        return NULL;
    }
    if (rsp_init(&device->rsp, &device->bus)) {
        return NULL;
    }
    if (g_vr4300->vr4300_init(&device->bus, profiling)) {
        return NULL;
    }
    angrylion_rdp_init(device);
    return device;
}
void device_destroy(struct n64_device *device, const char *cart_path)
{
    rsp_destroy(&device->rsp);
    if (cart_path && g_vr4300->has_profile_samples()) {
        char path[PATH_MAX];
        snprintf(path, PATH_MAX, "%s.profile", cart_path);
        path[PATH_MAX - 1] = '\0';
        FILE *f            = fopen(path, "w");
        if (!f) {
            printf("Can't open %s\n", path);
            return;
        }
        uint32_t i;
        for (i = 0; i < 8 * 1024 * 1024; i++) {
            const uint64_t sample     = g_vr4300->get_profile_sample(i);
            const uint64_t l1d_sample = g_vr4300->get_profile_sample(i + (8 * 1024 * 1024));
            if (sample < 10 && l1d_sample < 10)
                continue;
            fprintf(f, "%x %lu %lu\n", i + 0x80000000, sample, l1d_sample);
        }
        fclose(f);
    }
}
void device_exit(struct bus_controller *bus)
{
    longjmp(bus->unwind_data, 1);
}
void device_run(struct n64_device *device)
{
    g_vr4300->vr4300_cp1_init();
    rsp_late_init(&device->rsp);
    device_spin(device);
}
int device_spin(struct n64_device *device)
{
    if (setjmp(device->bus.unwind_data))
        return 1;
    while (likely(device->running)) {
        unsigned i;
        for (i = 0; i < 2; i++) {
            g_vr4300->vr4300_cycle();
            rsp_cycle(&device->rsp);
            ai_cycle(&device->ai);
            pi_cycle(&device->pi);
            vi_cycle(&device->vi);
        }
        g_vr4300->vr4300_cycle();
    }
    return 0;
}
