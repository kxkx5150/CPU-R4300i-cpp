#ifndef __device_h__
#define __device_h__
#include "../../utils/common.h"
#include "../../utils/common/rom_file.h"
#include "../../utils/common/save_file.h"
#include "../../utils/common/thread.h"
#include "ai.h"
#include "bus.h"
#include "dd.h"
#include "pi.h"
#include "ri.h"
#include "si.h"
#include "rdp.h"
#include "rsp.h"
#include "vi.h"
#include "cpu.h"

extern bool device_exit_requested;

struct n64_device
{
    struct bus_controller bus;
    struct vr4300        *vr4300;
    struct ai_controller  ai;
    struct dd_controller  dd;
    struct pi_controller  pi;
    struct ri_controller  ri;
    struct si_controller  si;
    struct vi_controller  vi;
    struct rdp            rdp;
    struct rsp            rsp;

    int       debug_sfd;
    bool      multithread;
    bool      other_thread_is_waiting;
    n64_mutex sync_mutex;
    n64_cv    sync_cv;
    bool      running;
};

void               device_destroy(struct n64_device *device, const char *cart_path);
struct n64_device *device_create(struct n64_device *device, const struct rom_file *ddipl,
                                 const struct dd_variant *dd_variant, const struct rom_file *ddrom,
                                 const struct rom_file *pifrom, const struct rom_file *cart,
                                 const struct save_file *eeprom, const struct save_file *sram,
                                 const struct save_file *flashram, struct is_viewer *is,
                                 const struct controller *controller, bool no_audio, bool no_video, bool profiling);

void device_exit(struct bus_controller *bus);
void device_run(struct n64_device *device);

#endif
