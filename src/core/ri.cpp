#include "../../utils/common.h"
#include "bus.h"
#include "ri.h"


int ri_init(struct ri_controller *ri, struct bus_controller *bus)
{
    ri->bus                  = bus;
    ri->regs[RI_MODE_REG]    = 0xE;
    ri->regs[RI_CONFIG_REG]  = 0x40;
    ri->regs[RI_SELECT_REG]  = 0x14;
    ri->regs[RI_REFRESH_REG] = 0x63634;
    return 0;
}
int read_rdram(void *opaque, uint32_t address, uint32_t *word)
{
    struct ri_controller *ri     = (struct ri_controller *)opaque;
    unsigned              offset = address - RDRAM_BASE_ADDRESS;
    memcpy(word, ri->ram + offset, sizeof(*word));
    *word = byteswap_32(*word);
    return 0;
}
int read_rdram_regs(void *opaque, uint32_t address, uint32_t *word)
{
    struct ri_controller *ri     = (struct ri_controller *)opaque;
    unsigned              offset = address - RDRAM_REGS_BASE_ADDRESS;
    enum rdram_register   reg    = (rdram_register)(offset >> 2);
    *word                        = ri->rdram_regs[reg];
    debug_mmio_read(rdram, rdram_register_mnemonics[reg], *word);
    return 0;
}
int read_ri_regs(void *opaque, uint32_t address, uint32_t *word)
{
    struct ri_controller *ri     = (struct ri_controller *)opaque;
    unsigned              offset = address - RI_REGS_BASE_ADDRESS;
    enum ri_register      reg    = (ri_register)(offset >> 2);
    *word                        = ri->regs[reg];
    debug_mmio_read(ri, ri_register_mnemonics[reg], *word);
    return 0;
}
int write_rdram(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    struct ri_controller *ri     = (struct ri_controller *)opaque;
    unsigned              offset = address - RDRAM_BASE_ADDRESS;
    uint32_t              orig_word;
    memcpy(&orig_word, ri->ram + offset, sizeof(orig_word));
    orig_word = byteswap_32(orig_word) & ~dqm;
    word      = byteswap_32(orig_word | word);
    memcpy(ri->ram + offset, &word, sizeof(word));
    return 0;
}
int write_rdram_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    struct ri_controller *ri     = (struct ri_controller *)opaque;
    unsigned              offset = address - RDRAM_REGS_BASE_ADDRESS;
    enum rdram_register   reg    = (rdram_register)(offset >> 2);
    debug_mmio_write(rdram, rdram_register_mnemonics[reg], word, dqm);
    ri->rdram_regs[reg] &= ~dqm;
    ri->rdram_regs[reg] |= word;
    return 0;
}
int write_ri_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    struct ri_controller *ri     = (struct ri_controller *)opaque;
    unsigned              offset = address - RI_REGS_BASE_ADDRESS;
    enum ri_register      reg    = (ri_register)(offset >> 2);
    debug_mmio_write(ri, ri_register_mnemonics[reg], word, dqm);
    ri->regs[reg] &= ~dqm;
    ri->regs[reg] |= word;
    return 0;
}
