#ifndef __rdp_cpu_h__
#define __rdp_cpu_h__
#include "../../utils/common.h"

enum dp_register
{
    DPC_START_REG,
    DPC_END_REG,
    DPC_CURRENT_REG,
    DPC_STATUS_REG,
    DPC_CLOCK_REG,
    DPC_BUFBUSY_REG,
    DPC_PIPEBUSY_REG,
    DPC_TMEM_REG,
    NUM_DP_REGISTERS
};
struct rdp
{
    uint32_t               regs[NUM_DP_REGISTERS];
    struct bus_controller *bus;
};

int rdp_init(struct rdp *rdp, struct bus_controller *bus);
int read_dp_regs(void *opaque, uint32_t address, uint32_t *word);
int write_dp_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);
#endif
