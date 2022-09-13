#include "../../utils/common.h"
#include "ai.h"
#include "bus.h"
#include "ri.h"
#include "rsp.h"
#include "cpu.h"


extern vr4300 *g_vr4300;

#define NTSC_DAC_FREQ 48681812
static n64_align(uint8_t buf[0x40000], 16);
static void           ai_dma(struct ai_controller *ai);
static const uint8_t *byteswap_audio_buffer(const uint8_t *input, uint8_t *output, uint32_t length);

void ai_cycle_(struct ai_controller *ai)
{
    if (ai->fifo_count > 0) {
        struct bus_controller *bus;
        memcpy(&bus, ai, sizeof(bus));
        g_vr4300->signal_rcp_interrupt(MI_INTR_AI);
        ai->fifo_ri ^= 0x1;
        ai->regs[AI_STATUS_REG] &= ~0xC0000001;
        ai->fifo_count--;
        if (ai->fifo_count > 0) {
            ai->regs[AI_STATUS_REG] = 0x40000000;
            ai_dma(ai);
        }
    }
}
void ai_dma(struct ai_controller *ai)
{
    struct bus_controller *bus;
    memcpy(&bus, ai, sizeof(bus));
    if (ai->fifo[ai->fifo_ri].length > 0) {
        unsigned freq    = (double)NTSC_DAC_FREQ / (ai->regs[AI_DACRATE_REG] + 1);
        unsigned samples = ai->fifo[ai->fifo_ri].length / 4;
        if (ai->no_output)
            ai->counter = (62500000.0 / freq) * samples;
        else {
            ALuint buffer;
            ALint  val;
            alGetSourcei(ai->ctx.source, AL_BUFFERS_PROCESSED, &val);
            if (ai->ctx.cur_frequency != freq) {
                if (val == 0) {
                    printf("OpenAL: Switching context buffer frequency to: %u\n", freq);
                    ai_switch_frequency(&ai->ctx, freq);
                }
            }
            if (val) {
                alSourceUnqueueBuffers(ai->ctx.source, val, ai->ctx.buffers + ai->ctx.unqueued_buffers);
                ai->ctx.unqueued_buffers += val;
            }
            switch (ai->ctx.unqueued_buffers) {
                case 0:
                case 1:
                    ai->counter = (62500000.0 / freq) * samples;
                    break;
                case 2:
                    ai->counter = (62500000.0 / freq) * (samples - 10);
                    break;
                case 3:
                    ai->counter = 1;
                    break;
            }
            if (ai->ctx.unqueued_buffers > 0) {
                uint32_t       length  = ai->fifo[ai->fifo_ri].length;
                uint8_t       *input   = bus->ri->ram + ai->fifo[ai->fifo_ri].address;
                const uint8_t *buf_ptr = byteswap_audio_buffer(input, buf, length);
                ai->ctx.unqueued_buffers--;
                buffer = ai->ctx.buffers[ai->ctx.unqueued_buffers];
                alBufferData(buffer, AL_FORMAT_STEREO16, buf_ptr, length, freq);
                alSourceQueueBuffers(ai->ctx.source, 1, &buffer);
            }
            alGetSourcei(ai->ctx.source, AL_SOURCE_STATE, &val);
            if (val != AL_PLAYING)
                alSourcePlay(ai->ctx.source);
        }
    } else {
        g_vr4300->signal_rcp_interrupt(MI_INTR_AI);
        ai->fifo_ri ^= 0x1;
        ai->regs[AI_STATUS_REG] &= ~0xC0000001;
        ai->fifo_count--;
        if (ai->fifo_count > 0) {
            ai->regs[AI_STATUS_REG] |= 0x40000000;
            ai->counter = 1;
        }
    }
}
int ai_init(struct ai_controller *ai, struct bus_controller *bus, bool no_interface)
{
    ai->bus       = bus;
    ai->no_output = no_interface;
    if (!no_interface) {
        alGetError();
        if (ai_context_create(&ai->ctx)) {
            ai->no_output = 1;
            return 1;
        }
    }
    return 0;
}
int read_ai_regs(void *opaque, uint32_t address, uint32_t *word)
{
    struct ai_controller *ai     = (struct ai_controller *)opaque;
    unsigned              offset = address - AI_REGS_BASE_ADDRESS;
    enum ai_register      reg    = (ai_register)(offset >> 2);
    *word                        = ai->regs[reg];
    debug_mmio_read(ai, ai_register_mnemonics[reg], *word);
    if (reg == AI_LEN_REG) {
        *word = 0;
        if (ai->regs[AI_STATUS_REG] & 0x80000001)
            *word = ai->regs[AI_LEN_REG];
        else if (ai->regs[AI_STATUS_REG] & 0x40000000) {
        }
    }
    return 0;
}
int write_ai_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    struct ai_controller *ai     = (struct ai_controller *)opaque;
    unsigned              offset = address - AI_REGS_BASE_ADDRESS;
    enum ai_register      reg    = (ai_register)(offset >> 2);
    debug_mmio_write(ai, ai_register_mnemonics[reg], word, dqm);
    if (reg == AI_DRAM_ADDR_REG)
        ai->regs[AI_DRAM_ADDR_REG] = word & 0xFFFFF8;
    else if (reg == AI_LEN_REG) {
        ai->regs[AI_LEN_REG] = word & 0x3FFF8;
        if (ai->fifo_count == 2)
            return 0;
        ai->fifo[ai->fifo_wi].address = ai->regs[AI_DRAM_ADDR_REG];
        ai->fifo[ai->fifo_wi].length  = ai->regs[AI_LEN_REG];
        ai->fifo_wi ^= 0x1;
        ai->fifo_count++;
        if (ai->fifo_count == 2)
            ai->regs[AI_STATUS_REG] |= 0x80000001U;
        if (!(ai->regs[AI_STATUS_REG] & 0x40000000U)) {
            ai->regs[AI_STATUS_REG] = 0x40000000;
            ai_dma(ai);
        }
    } else if (reg == AI_STATUS_REG) {
        struct bus_controller *bus;
        memcpy(&bus, ai, sizeof(bus));
        g_vr4300->clear_rcp_interrupt(MI_INTR_AI);
    } else if (reg == AI_DACRATE_REG) {
        ai->regs[AI_DACRATE_REG] = word & 0x3FFF;
        ai->ctx.frequency        = (double)NTSC_DAC_FREQ / (ai->regs[AI_DACRATE_REG] + 1);
    } else if (reg == AI_BITRATE_REG)
        ai->regs[AI_BITRATE_REG] = word & 0xF;
    else
        ai->regs[reg] = word;
    return 0;
}
const uint8_t *byteswap_audio_buffer(const uint8_t *input, uint8_t *output, uint32_t length)
{
    uint32_t i = 0;
    for (i = 0; i < length >> 1; ++i) {
        output[i << 1]       = input[(i << 1) + 1];
        output[(i << 1) + 1] = input[i << 1];
    }
    return (const uint8_t *)output;
}
int ai_context_create(struct n64_ai_context *context)
{
    uint8_t  buf[8];
    unsigned i;
    context->cur_frequency = 31985;
    if ((context->dev = alcOpenDevice(NULL)) == NULL) {
        printf("Failed to open the OpenAL device.\n");
        return 1;
    }
    if ((context->ctx = alcCreateContext(context->dev, NULL)) == NULL) {
        printf("Failed to create an OpenAL context.\n");
        alcCloseDevice(context->dev);
        return 1;
    }
    alcMakeContextCurrent(context->ctx);
    alGenBuffers(sizeof(context->buffers) / sizeof(*context->buffers), context->buffers);
    if (alGetError() != AL_NO_ERROR) {
        alcMakeContextCurrent(NULL);
        alcDestroyContext(context->ctx);
        alcCloseDevice(context->dev);
        return 1;
    }
    alGenSources(1, &context->source);
    if (alGetError() != AL_NO_ERROR) {
        alDeleteBuffers(sizeof(context->buffers) / sizeof(*context->buffers), context->buffers);
        alcMakeContextCurrent(NULL);
        alcDestroyContext(context->ctx);
        alcCloseDevice(context->dev);
        return 1;
    }
    memset(buf, 0x0, sizeof(buf));
    for (i = 0; i < sizeof(context->buffers) / sizeof(*context->buffers); i++)
        alBufferData(context->buffers[i], AL_FORMAT_STEREO16, buf, sizeof(buf), context->cur_frequency);
    context->unqueued_buffers = sizeof(context->buffers) / sizeof(*context->buffers);
    return 0;
}
void ai_context_destroy(struct n64_ai_context *context)
{
    alDeleteSources(1, &context->source);
    alDeleteBuffers(sizeof(context->buffers) / sizeof(*context->buffers), context->buffers);
    alcMakeContextCurrent(NULL);
    alcDestroyContext(context->ctx);
    alcCloseDevice(context->dev);
}
int ai_switch_frequency(struct n64_ai_context *context, ALint frequency)
{
    alDeleteSources(1, &context->source);
    alDeleteBuffers(sizeof(context->buffers) / sizeof(*context->buffers), context->buffers);
    alGenBuffers(sizeof(context->buffers) / sizeof(*context->buffers), context->buffers);
    alGenSources(1, &context->source);
    context->cur_frequency    = frequency;
    context->unqueued_buffers = sizeof(context->buffers) / sizeof(*context->buffers);
    return 0;
}
