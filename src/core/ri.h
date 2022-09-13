#ifndef __ri_controller_h__
#define __ri_controller_h__
#include "../../utils/common.h"

#define MAX_RDRAM_SIZE 0x800000U

enum rdram_register
{
    RDRAM_CONFIG_REG,
    RDRAM_DEVICE_ID_REG,
    RDRAM_DELAY_REG,
    RDRAM_MODE_REG,
    RDRAM_REF_INTERVAL_REG,
    RDRAM_REF_ROW_REG,
    RDRAM_RAS_INTERVAL_REG,
    RDRAM_MIN_INTERVAL_REG,
    RDRAM_ADDR_SELECT_REG,
    NUM_RDRAM_REGISTERS
};
enum ri_register
{
    RI_MODE_REG,
    RI_CONFIG_REG,
    RI_CURRENT_LOAD_REG,
    RI_SELECT_REG,
    RI_REFRESH_REG,
    RI_LATENCY_REG,
    RI_RERROR_REG,
    RI_WERROR_REG,
    NUM_RI_REGISTERS
};
struct ri_controller
{
    struct bus_controller *bus;
    uint32_t               rdram_regs[NUM_RDRAM_REGISTERS];
    uint32_t               regs[NUM_RI_REGISTERS];
    uint64_t               force_ram_alignment;
    uint8_t                ram[MAX_RDRAM_SIZE];
};

int ri_init(struct ri_controller *ri, struct bus_controller *bus);
int read_rdram(void *opaque, uint32_t address, uint32_t *word);
int write_rdram(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);
int read_rdram_regs(void *opaque, uint32_t address, uint32_t *word);
int read_ri_regs(void *opaque, uint32_t address, uint32_t *word);
int write_rdram_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);
int write_ri_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);
#endif
