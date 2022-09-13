#ifndef __ai_controller_h__
#define __ai_controller_h__
#include <al.h>
#include <alc.h>
#include "../../utils/common.h"


struct n64_ai_context
{
    ALuint      buffers[3];
    ALuint      unqueued_buffers;
    ALuint      cur_frequency;
    ALuint      frequency;
    ALuint      source;
    ALCdevice  *dev;
    ALCcontext *ctx;
};
enum ai_register
{
    AI_DRAM_ADDR_REG,
    AI_LEN_REG,
    AI_CONTROL_REG,
    AI_STATUS_REG,
    AI_DACRATE_REG,
    AI_BITRATE_REG,
    NUM_AI_REGISTERS
};
struct ai_fifo_entry
{
    uint32_t address;
    uint32_t length;
};
struct ai_controller
{
    struct bus_controller *bus;
    uint32_t               regs[NUM_AI_REGISTERS];
    struct n64_ai_context  ctx;
    uint64_t               counter;
    unsigned               fifo_count, fifo_wi, fifo_ri;
    struct ai_fifo_entry   fifo[2];
    bool                   no_output;
};

int  ai_init(struct ai_controller *ai, struct bus_controller *bus, bool no_interface);
void ai_cycle_(struct ai_controller *ai);
int  ai_context_create(struct n64_ai_context *context);
void ai_context_destroy(struct n64_ai_context *context);
int  ai_switch_frequency(struct n64_ai_context *context, ALint frequency);
int  read_ai_regs(void *opaque, uint32_t address, uint32_t *word);
int  write_ai_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);

static inline void ai_cycle(struct ai_controller *ai)
{
    if (unlikely(ai->counter-- == 0))
        ai_cycle_(ai);
}

#endif
