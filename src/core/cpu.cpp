#include <cstdint>
#include "cpu.h"

extern vr4300 *g_vr4300;
#define CALL_MEMBER_FN(object, ptrToMember) ((object).*(ptrToMember))



void vr4300::check_for_interrupts(uint32_t flags)
{
    if (flags & g_vr4300->mi_regs[MI_INTR_MASK_REG])
        raise_rcp_interrupt();
    else
        lower_rcp_interrupt();
}
void vr4300::clear_rcp_interrupt(enum rcp_interrupt_mask mask)
{
    uint32_t flags = __sync_and_and_fetch(&g_vr4300->mi_regs[MI_INTR_REG], ~mask);
    check_for_interrupts(flags);
}
void vr4300::lower_rcp_interrupt()
{
    __sync_and_and_fetch(&g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE], ~0x400);
}
void vr4300::raise_rcp_interrupt()
{
    __sync_or_and_fetch(&g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE], 0x400);
}
void vr4300::clear_dd_interrupt()
{
    __sync_and_and_fetch(&g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE], ~0x800);
}
void vr4300::signal_dd_interrupt()
{
    __sync_or_and_fetch(&g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE], 0x800);
}
void vr4300::signal_rcp_interrupt(enum rcp_interrupt_mask mask)
{
    uint32_t flags = __sync_or_and_fetch(&g_vr4300->mi_regs[MI_INTR_REG], mask);
    check_for_interrupts(flags);
}
int vr4300::read_mi_regs(uint32_t address, uint32_t *word)
{
    uint32_t         offset = address - 0x04300000;
    enum mi_register reg    = (mi_register)(offset >> 2);
    *word                   = g_vr4300->mi_regs[reg];
    do {
    } while (0);
    return 0;
}
int vr4300::write_mi_regs(uint32_t address, uint32_t word, uint32_t dqm)
{
    uint32_t         offset = address - 0x04300000;
    enum mi_register reg    = (mi_register)(offset >> 2);
    uint32_t         result;
    uint32_t         flags;
    do {
    } while (0);
    if (reg == MI_INIT_MODE_REG) {
        result = word & 0x3FF;
        if (word & 0x0080)
            result &= ~0x0100;
        else if (word & 0x0100)
            result |= 0x0100;
        if (word & 0x0200)
            result &= ~0x0080;
        else if (word & 0x0400)
            result |= 0x0080;
        if (word & 0x0800) {
            flags = __sync_and_and_fetch(&g_vr4300->mi_regs[MI_INTR_REG], ~MI_INTR_DP);
            check_for_interrupts(flags);
        }
        if (word & 0x1000)
            result &= ~0x0200;
        else if (word & 0x2000)
            result |= 0x0200;
        g_vr4300->mi_regs[MI_INIT_MODE_REG] = result;
    } else if (reg == MI_INTR_MASK_REG) {
        if (word & 0x0001)
            g_vr4300->mi_regs[MI_INTR_MASK_REG] &= ~MI_INTR_SP;
        else if (word & 0x0002)
            g_vr4300->mi_regs[MI_INTR_MASK_REG] |= MI_INTR_SP;
        if (word & 0x0004)
            g_vr4300->mi_regs[MI_INTR_MASK_REG] &= ~MI_INTR_SI;
        else if (word & 0x0008)
            g_vr4300->mi_regs[MI_INTR_MASK_REG] |= MI_INTR_SI;
        if (word & 0x0010)
            g_vr4300->mi_regs[MI_INTR_MASK_REG] &= ~MI_INTR_AI;
        else if (word & 0x0020)
            g_vr4300->mi_regs[MI_INTR_MASK_REG] |= MI_INTR_AI;
        if (word & 0x0040)
            g_vr4300->mi_regs[MI_INTR_MASK_REG] &= ~MI_INTR_VI;
        else if (word & 0x0080)
            g_vr4300->mi_regs[MI_INTR_MASK_REG] |= MI_INTR_VI;
        if (word & 0x0100)
            g_vr4300->mi_regs[MI_INTR_MASK_REG] &= ~MI_INTR_PI;
        else if (word & 0x0200)
            g_vr4300->mi_regs[MI_INTR_MASK_REG] |= MI_INTR_PI;
        if (word & 0x0400)
            g_vr4300->mi_regs[MI_INTR_MASK_REG] &= ~MI_INTR_DP;
        else if (word & 0x0800)
            g_vr4300->mi_regs[MI_INTR_MASK_REG] |= MI_INTR_DP;
        check_for_interrupts(g_vr4300->mi_regs[MI_INTR_REG]);
    } else {
        g_vr4300->mi_regs[reg] &= ~dqm;
        g_vr4300->mi_regs[reg] |= word;
    }
    return 0;
}
int vr4300::has_profile_samples()
{
    return g_vr4300->profile_samples != __null;
}
uint64_t vr4300::get_profile_sample(size_t i)
{
    return g_vr4300->profile_samples[i];
}
bool vr4300::vr4300_read_word_vaddr(uint64_t vaddr, uint32_t *result)
{
    if (vaddr & 0x3) {
        return false;
    }
    const struct segment *segment = get_segment(vaddr, g_vr4300->regs[VR4300_CP0_REGISTER_STATUS]);
    if (!segment) {
        return false;
    }
    uint32_t paddr;
    bool     cached;
    if (segment->mapped) {
        unsigned asid = g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYHI] & 0xFF;
        unsigned select, tlb_miss, index;
        uint32_t page_mask;
        tlb_miss  = tlb_probe(&g_vr4300->cp0.tlb, vaddr, asid, &index);
        page_mask = g_vr4300->cp0.page_mask[index];
        select    = ((page_mask + 1) & vaddr) != 0;
        if (__builtin_expect((tlb_miss || !(g_vr4300->cp0.state[index][select] & 2)), 0)) {
            return false;
        }
        cached = ((g_vr4300->cp0.state[index][select] & 0x38) != 0x10);
        paddr  = (g_vr4300->cp0.pfn[index][select]) | (vaddr & page_mask);
    } else {
        paddr  = vaddr - segment->offset;
        cached = segment->cached;
    }
    if (cached) {
        struct vr4300_dcache_line *line = vr4300_dcache_probe(&g_vr4300->dcache, vaddr, paddr);
        if (line) {
            memcpy(result, line->data + ((paddr & 0xf) ^ 4), sizeof(uint32_t));
        } else {
            bus_read_word(g_vr4300->bus, paddr, result);
        }
    } else {
        bus_read_word(g_vr4300->bus, paddr, result);
    }
    return true;
}
void vr4300::vr4300_cycle()
{
    struct vr4300_pipeline *pipeline = &g_vr4300->pipeline;
    g_vr4300->regs[VR4300_CP0_REGISTER_COUNT]++;
    if (pipeline->cycles_to_stall > 0)
        pipeline->cycles_to_stall--;
    else
        vr4300_cycle_();
    if ((g_vr4300->regs[VR4300_CP0_REGISTER_COUNT] & 1) == 1 &&
        (uint32_t)(g_vr4300->regs[VR4300_CP0_REGISTER_COUNT] >> 1) ==
            (uint32_t)g_vr4300->regs[VR4300_CP0_REGISTER_COMPARE]) {
        g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE] |= 0x8000;
    }
}
void vr4300::vr4300_connect_bus(struct bus_controller *bus)
{
    g_vr4300->bus = bus;
}
int vr4300::vr4300_init(struct bus_controller *bus, bool profiling)
{
    vr4300_connect_bus(bus);
    vr4300_cp0_init();
    vr4300_cp1_init();
    vr4300_dcache_init(&g_vr4300->dcache);
    vr4300_icache_init(&g_vr4300->icache);
    vr4300_pipeline_init(&g_vr4300->pipeline);
    g_vr4300->signals                   = VR4300_SIGNAL_COLDRESET;
    g_vr4300->mi_regs[MI_VERSION_REG]   = 0x01010101;
    g_vr4300->mi_regs[MI_INIT_MODE_REG] = 0x80;
    if (profiling)
        g_vr4300->profile_samples = (uint64_t *)calloc(2 * 8 * 1024 * 1024, sizeof(uint64_t));
    else
        g_vr4300->profile_samples = __null;
    return 0;
}
void vr4300::vr4300_print_summary(struct vr4300_stats *stats)
{
}
uint64_t vr4300::vr4300_get_register(size_t i)
{
    return g_vr4300->regs[i];
}
uint64_t vr4300::vr4300_get_pc()
{
    return g_vr4300->pipeline.dcwb_latch.common.pc;
}
void vr4300::vr4300_signal_break()
{
}
void vr4300::vr4300_set_breakpoint(uint64_t at)
{
}
void vr4300::vr4300_remove_breakpoint(uint64_t at)
{
}
uint64_t vr4300::mask_reg(unsigned reg, uint64_t data)
{
    return vr4300_cp0_reg_masks[reg] & data;
};
int vr4300::VR4300_DMFC0(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 16 & 0x1F);
    unsigned                  src        = ((iw) >> 11 & 0x1F);
    if (src == (VR4300_CP0_REGISTER_COUNT - VR4300_REGISTER_CP0_0)) {
        exdc_latch->result = (uint32_t)(g_vr4300->regs[VR4300_CP0_REGISTER_COUNT] >> 1);
    } else if (vr4300_cp0_reg_masks[src] == 0x0000000000000BADULL)
        exdc_latch->result = g_vr4300->regs[VR4300_REGISTER_CP0_0 + 7];
    else if (src + VR4300_REGISTER_CP0_0 == VR4300_CP0_REGISTER_PRID)
        exdc_latch->result = 0xb22;
    else {
        exdc_latch->result = mask_reg(src, g_vr4300->regs[VR4300_REGISTER_CP0_0 + src]);
    }
    exdc_latch->dest = dest;
    return 0;
}
int vr4300::VR4300_DMTC0(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_icrf_latch *icrf_latch = &g_vr4300->pipeline.icrf_latch;
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    switch (dest + VR4300_REGISTER_CP0_0) {
        case VR4300_CP0_REGISTER_COUNT:
            rt <<= 1;
            break;
        case VR4300_CP0_REGISTER_CAUSE:
            g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE] &= ~0x0300;
            g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE] |= rt & 0x0300;
            return 0;
        case VR4300_CP0_REGISTER_COMPARE:
            g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE] &= ~0x8000;
            break;
        case VR4300_CP0_REGISTER_STATUS:
            icrf_latch->segment = get_segment(icrf_latch->common.pc, rt);
            exdc_latch->segment = get_default_segment();
            break;
    }
    if (vr4300_cp0_reg_masks[dest] == 0x0000000000000BADULL)
        g_vr4300->regs[VR4300_REGISTER_CP0_0 + 7] = rt;
    else
        g_vr4300->regs[VR4300_REGISTER_CP0_0 + dest] = rt;
    return 0;
}
int vr4300::VR4300_ERET(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_icrf_latch *icrf_latch = &g_vr4300->pipeline.icrf_latch;
    struct vr4300_rfex_latch *rfex_latch = &g_vr4300->pipeline.rfex_latch;
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    struct vr4300_pipeline   *pipeline   = &g_vr4300->pipeline;
    int32_t                   status     = g_vr4300->regs[VR4300_CP0_REGISTER_STATUS];
    if (status & 0x4) {
        icrf_latch->pc = g_vr4300->regs[VR4300_CP0_REGISTER_ERROREPC];
        status &= ~0x4;
    } else {
        icrf_latch->pc = g_vr4300->regs[VR4300_CP0_REGISTER_EPC];
        status &= ~0x2;
    }
    exdc_latch->common.fault                   = (vr4300_fault_id)~0;
    rfex_latch->common.fault                   = (vr4300_fault_id)~0;
    g_vr4300->regs[PIPELINE_CYCLE_TYPE]        = 4;
    pipeline->exception_history                = 0;
    pipeline->fault_present                    = true;
    g_vr4300->regs[VR4300_CP0_REGISTER_STATUS] = status;
    pipeline->icrf_latch.segment               = get_segment(icrf_latch->pc, status);
    pipeline->exdc_latch.segment               = get_default_segment();
    return 1;
}
int vr4300::VR4300_MFC0(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 16 & 0x1F);
    unsigned                  src        = ((iw) >> 11 & 0x1F);
    if (src == (VR4300_CP0_REGISTER_COUNT - VR4300_REGISTER_CP0_0)) {
        exdc_latch->result = (int32_t)(g_vr4300->regs[VR4300_CP0_REGISTER_COUNT] >> 1);
    } else if (vr4300_cp0_reg_masks[src] == 0x0000000000000BADULL)
        exdc_latch->result = (int32_t)g_vr4300->regs[VR4300_REGISTER_CP0_0 + 7];
    else if (src + VR4300_REGISTER_CP0_0 == VR4300_CP0_REGISTER_PRID)
        exdc_latch->result = 0xb22;
    else if (src == (VR4300_CP0_REGISTER_RANDOM - VR4300_REGISTER_CP0_0)) {
        unsigned index     = g_vr4300->regs[VR4300_CP0_REGISTER_WIRED] & 0x1F;
        exdc_latch->result = rand() % (32 - index) + index;
    } else {
        exdc_latch->result = (int32_t)mask_reg(src, g_vr4300->regs[VR4300_REGISTER_CP0_0 + src]);
    }
    exdc_latch->dest = (int32_t)dest;
    return 0;
}
int vr4300::VR4300_MTC0(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_icrf_latch *icrf_latch = &g_vr4300->pipeline.icrf_latch;
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    switch (dest + VR4300_REGISTER_CP0_0) {
        case VR4300_CP0_REGISTER_COUNT:
            rt <<= 1;
            break;
        case VR4300_CP0_REGISTER_CAUSE:
            g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE] &= ~0x0300;
            g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE] |= (int32_t)rt & 0x0300;
            return 0;
        case VR4300_CP0_REGISTER_COMPARE:
            g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE] &= ~0x8000;
            break;
        case VR4300_CP0_REGISTER_STATUS:
            icrf_latch->segment = get_segment(icrf_latch->common.pc, rt);
            exdc_latch->segment = get_default_segment();
            break;
    }
    if (vr4300_cp0_reg_masks[dest] == 0x0000000000000BADULL)
        g_vr4300->regs[VR4300_REGISTER_CP0_0 + 7] = (int32_t)rt;
    else
        g_vr4300->regs[VR4300_REGISTER_CP0_0 + dest] = (int32_t)rt;
    return 0;
}
int vr4300::VR4300_TLBP(uint32_t iw, uint64_t rs, uint64_t rt)
{
    uint64_t entry_hi = mask_reg(10, g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYHI]);
    unsigned index;
    g_vr4300->regs[VR4300_CP0_REGISTER_INDEX] |= 0x80000000U;
    if (!tlb_probe(&g_vr4300->cp0.tlb, entry_hi, entry_hi & 0xFF, &index))
        g_vr4300->regs[VR4300_CP0_REGISTER_INDEX] = index;
    return 0;
}
int vr4300::VR4300_TLBR(uint32_t iw, uint64_t rs, uint64_t rt)
{
    unsigned index = g_vr4300->regs[VR4300_CP0_REGISTER_INDEX] & 0x1F;
    uint64_t entry_hi;
    uint32_t page_mask = (g_vr4300->cp0.page_mask[index] << 1) & 0x1FFE000U;
    uint32_t pfn0      = g_vr4300->cp0.pfn[index][0];
    uint32_t pfn1      = g_vr4300->cp0.pfn[index][1];
    uint8_t  state0    = g_vr4300->cp0.state[index][0];
    uint8_t  state1    = g_vr4300->cp0.state[index][1];
    tlb_read(&g_vr4300->cp0.tlb, index, &entry_hi);
    g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYHI]  = entry_hi;
    g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYLO0] = (pfn0 >> 6) | state0;
    g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYLO1] = (pfn1 >> 6) | state1;
    g_vr4300->regs[VR4300_CP0_REGISTER_PAGEMASK] = page_mask;
    return 0;
}
int vr4300::VR4300_TLBWI(uint32_t iw, uint64_t rs, uint64_t rt)
{
    uint64_t entry_hi   = mask_reg(10, g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYHI]);
    uint64_t entry_lo_0 = mask_reg(2, g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYLO0]);
    uint64_t entry_lo_1 = mask_reg(3, g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYLO1]);
    uint32_t page_mask  = mask_reg(5, g_vr4300->regs[VR4300_CP0_REGISTER_PAGEMASK]);
    unsigned index      = g_vr4300->regs[VR4300_CP0_REGISTER_INDEX] & 0x1F;
    tlb_write(&g_vr4300->cp0.tlb, index, entry_hi, entry_lo_0, entry_lo_1, page_mask);
    g_vr4300->cp0.page_mask[index] = (page_mask | 0x1FFF) >> 1;
    g_vr4300->cp0.pfn[index][0]    = (entry_lo_0 << 6) & ~0xFFFU;
    g_vr4300->cp0.pfn[index][1]    = (entry_lo_1 << 6) & ~0xFFFU;
    g_vr4300->cp0.state[index][0]  = entry_lo_0 & 0x3F;
    g_vr4300->cp0.state[index][1]  = entry_lo_1 & 0x3F;
    return 0;
}
int vr4300::VR4300_TLBWR(uint32_t iw, uint64_t rs, uint64_t rt)
{
    uint64_t entry_hi   = mask_reg(10, g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYHI]);
    uint64_t entry_lo_0 = mask_reg(2, g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYLO0]);
    uint64_t entry_lo_1 = mask_reg(3, g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYLO1]);
    uint32_t page_mask  = mask_reg(5, g_vr4300->regs[VR4300_CP0_REGISTER_PAGEMASK]);
    unsigned index      = g_vr4300->regs[VR4300_CP0_REGISTER_WIRED] & 0x1F;
    index               = rand() % (32 - index) + index;
    tlb_write(&g_vr4300->cp0.tlb, index, entry_hi, entry_lo_0, entry_lo_1, page_mask);
    g_vr4300->cp0.page_mask[index] = (page_mask | 0x1FFF) >> 1;
    g_vr4300->cp0.pfn[index][0]    = (entry_lo_0 << 6) & ~0xFFFU;
    g_vr4300->cp0.pfn[index][1]    = (entry_lo_1 << 6) & ~0xFFFU;
    g_vr4300->cp0.state[index][0]  = entry_lo_0 & 0x3F;
    g_vr4300->cp0.state[index][1]  = entry_lo_1 & 0x3F;
    return 0;
}
void vr4300::vr4300_cp0_init()
{
    tlb_init(&g_vr4300->cp0.tlb);
}
int vr4300::vr4300_do_mci(unsigned cycles)
{
    g_vr4300->pipeline.cycles_to_stall  = cycles - 1;
    g_vr4300->regs[PIPELINE_CYCLE_TYPE] = 3;
    return 1;
}
int vr4300::VR4300_CP1_ABS(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32, fd32;
    uint64_t                  result;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            fpu_abs_32(&fs32, &fd32);
            result = fd32;
            break;
        case VR4300_FMT_D:
            fpu_abs_64(&fs, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(3);
}
int vr4300::VR4300_CP1_ADD(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32, ft32, fd32;
    uint64_t                  result;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            ft32 = ft;
            fpu_add_32(&fs32, &ft32, &fd32);
            result = fd32;
            break;
        case VR4300_FMT_D:
            fpu_add_64(&fs, &ft, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(3);
}
int vr4300::VR4300_BC1(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_icrf_latch *icrf_latch = &g_vr4300->pipeline.icrf_latch;
    struct vr4300_rfex_latch *rfex_latch = &g_vr4300->pipeline.rfex_latch;
    unsigned                  opcode     = (iw >> 16) & 0x3;
    uint64_t                  offset     = (uint64_t)((int16_t)iw) << 2;
    uint64_t                  taken_pc   = rfex_latch->common.pc + (offset + 4);
    uint32_t                  cond       = g_vr4300->regs[VR4300_CP1_FCR31];
    if (g_vr4300->pipeline.dcwb_latch.dest == VR4300_CP1_FCR31)
        cond = g_vr4300->pipeline.dcwb_latch.result;
    switch (opcode) {
        case 0x0:
            if (!(cond >> 23 & 0x1))
                icrf_latch->pc = taken_pc;
            break;
        case 0x1:
            if (cond >> 23 & 0x1)
                icrf_latch->pc = taken_pc;
            break;
        case 0x2:
            if (!(cond >> 23 & 0x1))
                icrf_latch->pc = taken_pc;
            else
                rfex_latch->iw_mask = 0;
            break;
        case 0x3:
            if (cond >> 23 & 0x1)
                icrf_latch->pc = taken_pc;
            else
                rfex_latch->iw_mask = 0;
            break;
    }
    return 0;
}
int vr4300::VR4300_CP1_C_EQ_C_SEQ(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = VR4300_CP1_FCR31;
    uint64_t                  result     = g_vr4300->regs[dest];
    uint32_t                  fs32, ft32;
    uint8_t                   flag;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            ft32 = ft;
            result &= ~(1 << 23);
            flag = fpu_cmp_eq_32(&fs32, &ft32);
            break;
        case VR4300_FMT_D:
            result &= ~(1 << 23);
            flag = fpu_cmp_eq_64(&fs, &ft);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result | (flag << 23);
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_CP1_C_F_C_SF(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = VR4300_CP1_FCR31;
    uint64_t                  result     = g_vr4300->regs[dest];
    uint32_t                  fs32, ft32;
    uint8_t                   flag;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            ft32 = ft;
            result &= ~(1 << 23);
            flag = fpu_cmp_f_32(&fs32, &ft32);
            break;
        case VR4300_FMT_D:
            result &= ~(1 << 23);
            flag = fpu_cmp_f_64(&fs, &ft);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result | (flag << 23);
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_CP1_C_OLE_C_LE(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = VR4300_CP1_FCR31;
    uint64_t                  result     = g_vr4300->regs[dest];
    uint32_t                  fs32, ft32;
    uint8_t                   flag;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            ft32 = ft;
            result &= ~(1 << 23);
            flag = fpu_cmp_ole_32(&fs32, &ft32);
            break;
        case VR4300_FMT_D:
            result &= ~(1 << 23);
            flag = fpu_cmp_ole_64(&fs, &ft);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result | (flag << 23);
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_CP1_C_OLT_C_LT(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = VR4300_CP1_FCR31;
    uint64_t                  result     = g_vr4300->regs[dest];
    uint32_t                  fs32, ft32;
    uint8_t                   flag;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            ft32 = ft;
            result &= ~(1 << 23);
            flag = fpu_cmp_olt_32(&fs32, &ft32);
            break;
        case VR4300_FMT_D:
            result &= ~(1 << 23);
            flag = fpu_cmp_olt_64(&fs, &ft);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result | (flag << 23);
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_CP1_C_UEQ_C_NGL(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = VR4300_CP1_FCR31;
    uint64_t                  result     = g_vr4300->regs[dest];
    uint32_t                  fs32, ft32;
    uint8_t                   flag;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            ft32 = ft;
            result &= ~(1 << 23);
            flag = fpu_cmp_ueq_32(&fs32, &ft32);
            break;
        case VR4300_FMT_D:
            result &= ~(1 << 23);
            flag = fpu_cmp_ueq_64(&fs, &ft);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result | (flag << 23);
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_CP1_C_ULE_C_NGT(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = VR4300_CP1_FCR31;
    uint64_t                  result     = g_vr4300->regs[dest];
    uint32_t                  fs32, ft32;
    uint8_t                   flag;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            ft32 = ft;
            result &= ~(1 << 23);
            flag = fpu_cmp_ule_32(&fs32, &ft32);
            break;
        case VR4300_FMT_D:
            result &= ~(1 << 23);
            flag = fpu_cmp_ule_64(&fs, &ft);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result | (flag << 23);
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_CP1_C_ULT_C_NGE(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = VR4300_CP1_FCR31;
    uint64_t                  result     = g_vr4300->regs[dest];
    uint32_t                  fs32, ft32;
    uint8_t                   flag;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            ft32 = ft;
            result &= ~(1 << 23);
            flag = fpu_cmp_ult_32(&fs32, &ft32);
            break;
        case VR4300_FMT_D:
            result &= ~(1 << 23);
            flag = fpu_cmp_ult_64(&fs, &ft);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result | (flag << 23);
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_CP1_C_UN_C_NGLE(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = VR4300_CP1_FCR31;
    uint64_t                  result     = g_vr4300->regs[dest];
    uint32_t                  fs32, ft32;
    uint8_t                   flag;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            ft32 = ft;
            result &= ~(1 << 23);
            flag = fpu_cmp_un_32(&fs32, &ft32);
            break;
        case VR4300_FMT_D:
            result &= ~(1 << 23);
            flag = fpu_cmp_un_64(&fs, &ft);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result | (flag << 23);
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_CP1_CEIL_L(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32;
    uint64_t                  result;
    uint32_t                  saved_state = fpu_get_state();
    fpu_set_state((saved_state & ~0x6000) | 0x4000);
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            fpu_cvt_i64_f32(&fs32, &result);
            break;
        case VR4300_FMT_D:
            fpu_cvt_i64_f64(&fs, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    fpu_set_state((saved_state & 0x6000) | (fpu_get_state() & ~0x6000));
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(5);
}
int vr4300::VR4300_CP1_CEIL_W(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32;
    uint32_t                  result;
    uint32_t                  saved_state = fpu_get_state();
    fpu_set_state((saved_state & ~0x6000) | 0x4000);
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            fpu_cvt_i32_f32(&fs32, &result);
            break;
        case VR4300_FMT_D:
            fpu_cvt_i32_f64(&fs, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    fpu_set_state((saved_state & 0x6000) | (fpu_get_state() & ~0x6000));
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(5);
}
int vr4300::VR4300_CFC1(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 16 & 0x1F);
    unsigned                  src        = ((iw) >> 11 & 0x1F);
    uint64_t                  result;
    switch (src) {
        case 0:
            src = VR4300_CP1_FCR0;
            break;
        case 31:
            src = VR4300_CP1_FCR31;
            break;
        default:
            src = 0;
            break;
    }
    result = g_vr4300->regs[src];
    if (g_vr4300->pipeline.dcwb_latch.dest == VR4300_CP1_FCR31)
        result = g_vr4300->pipeline.dcwb_latch.result;
    exdc_latch->result = (int32_t)result;
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_CTC1(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    if (dest == 31)
        dest = VR4300_CP1_FCR31;
    else {
        dest = 0;
        rt   = 0;
    }
    exdc_latch->result = rt;
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_CP1_CVT_D(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32;
    uint64_t                  result;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            fpu_cvt_f64_f32(&fs32, &result);
            break;
        case VR4300_FMT_W:
            fs32 = fs;
            fpu_cvt_f64_i32(&fs32, &result);
            break;
        case VR4300_FMT_L:
            fpu_cvt_f64_i64(&fs, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return fmt != VR4300_FMT_S ? vr4300_do_mci(5) : 0;
}
int vr4300::VR4300_CP1_CVT_L(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32;
    uint64_t                  result;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            fpu_cvt_i64_f32(&fs32, &result);
            break;
        case VR4300_FMT_D:
            fpu_cvt_i64_f64(&fs, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(5);
}
int vr4300::VR4300_CP1_CVT_S(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32;
    uint32_t                  result;
    switch (fmt) {
        case VR4300_FMT_D:
            fpu_cvt_f32_f64(&fs, &result);
            break;
        case VR4300_FMT_W:
            fs32 = fs;
            fpu_cvt_f32_i32(&fs32, &result);
            break;
        case VR4300_FMT_L:
            fpu_cvt_f32_i64(&fs, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(fmt == VR4300_FMT_D ? 2 : 5);
}
int vr4300::VR4300_CP1_CVT_W(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32;
    uint32_t                  result;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            fpu_cvt_i32_f32(&fs32, &result);
            break;
        case VR4300_FMT_D:
            fpu_cvt_i32_f64(&fs, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(5);
}
int vr4300::VR4300_CP1_DIV(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32, ft32, fd32;
    uint64_t                  result;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            ft32 = ft;
            fpu_div_32(&fs32, &ft32, &fd32);
            result = fd32;
            break;
        case VR4300_FMT_D:
            fpu_div_64(&fs, &ft, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(fmt == VR4300_FMT_D ? 58 : 29);
}
int vr4300::VR4300_DMFC1(uint32_t iw, uint64_t fs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 16 & 0x1F);
    exdc_latch->result                   = fs;
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_DMTC1(uint32_t iw, uint64_t fs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = (((iw) >> 11 & 0x1F) + VR4300_REGISTER_CP1_0);
    exdc_latch->result                   = rt;
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_CP1_FLOOR_L(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32;
    uint64_t                  result;
    uint32_t                  saved_state = fpu_get_state();
    fpu_set_state((saved_state & ~0x6000) | 0x2000);
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            fpu_cvt_i64_f32(&fs32, &result);
            break;
        case VR4300_FMT_D:
            fpu_cvt_i64_f64(&fs, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    fpu_set_state((saved_state & 0x6000) | (fpu_get_state() & ~0x6000));
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(5);
}
int vr4300::VR4300_CP1_FLOOR_W(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32;
    uint32_t                  result;
    uint32_t                  saved_state = fpu_get_state();
    fpu_set_state((saved_state & ~0x6000) | 0x2000);
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            fpu_cvt_i32_f32(&fs32, &result);
            break;
        case VR4300_FMT_D:
            fpu_cvt_i32_f64(&fs, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    fpu_set_state((saved_state & 0x6000) | (fpu_get_state() & ~0x6000));
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(5);
}
int vr4300::VR4300_LDC1(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = (((iw) >> 16 & 0x1F) + VR4300_REGISTER_CP1_0);
    exdc_latch->request.vaddr            = rs + (int16_t)iw;
    exdc_latch->request.data             = ~0ULL;
    exdc_latch->request.wdqm             = 0ULL;
    exdc_latch->request.postshift        = 0;
    exdc_latch->request.access_type      = VR4300_ACCESS_DWORD;
    exdc_latch->request.type             = VR4300_BUS_REQUEST_READ;
    exdc_latch->request.size             = 8;
    exdc_latch->dest                     = dest;
    exdc_latch->result                   = 0;
    return 0;
}
int vr4300::VR4300_LWC1(uint32_t iw, uint64_t rs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint32_t                  status     = g_vr4300->regs[VR4300_CP0_REGISTER_STATUS];
    uint64_t                  address    = (rs + (int16_t)iw);
    unsigned                  dest       = (((iw) >> 16 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint64_t                  result     = 0;
    unsigned                  postshift  = 0;
    if (!(status & 0x04000000)) {
        result    = dest & 0x1 ? ft & 0x00000000FFFFFFFFULL : ft & 0xFFFFFFFF00000000ULL;
        postshift = (dest & 0x1) << 5;
        dest &= ~0x1;
    }
    exdc_latch->request.vaddr       = address;
    exdc_latch->request.data        = ~0U;
    exdc_latch->request.wdqm        = 0ULL;
    exdc_latch->request.postshift   = postshift;
    exdc_latch->request.access_type = VR4300_ACCESS_WORD;
    exdc_latch->request.type        = VR4300_BUS_REQUEST_READ;
    exdc_latch->request.size        = 4;
    exdc_latch->result              = result;
    exdc_latch->dest                = dest;
    return 0;
}
int vr4300::VR4300_CP1_MUL(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32, ft32, fd32;
    uint64_t                  result;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            ft32 = ft;
            fpu_mul_32(&fs32, &ft32, &fd32);
            result = fd32;
            break;
        case VR4300_FMT_D:
            fpu_mul_64(&fs, &ft, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(fmt == VR4300_FMT_D ? 8 : 5);
}
int vr4300::VR4300_MFC1(uint32_t iw, uint64_t fs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint32_t                  status     = g_vr4300->regs[VR4300_CP0_REGISTER_STATUS];
    unsigned                  dest       = ((iw) >> 16 & 0x1F);
    uint64_t                  result;
    if (status & 0x04000000)
        result = (int32_t)fs;
    else {
        result = ((((iw) >> 11 & 0x1F) + VR4300_REGISTER_CP1_0) & 0x1) ? (int32_t)(fs >> 32) : (int32_t)(fs);
    }
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_CP1_MOV(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    exdc_latch->result                   = fs;
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_MTC1(uint32_t iw, uint64_t fs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint32_t                  status     = g_vr4300->regs[VR4300_CP0_REGISTER_STATUS];
    uint64_t                  result     = (int32_t)rt;
    unsigned                  dest       = (((iw) >> 11 & 0x1F) + VR4300_REGISTER_CP1_0);
    if (!(status & 0x04000000)) {
        result = (dest & 0x1) ? ((uint32_t)fs) | (rt << 32) : (fs & ~0xFFFFFFFFULL) | ((uint32_t)rt);
        dest &= ~0x1;
    }
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_CP1_NEG(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32, fd32;
    uint64_t                  result;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            fpu_neg_32(&fs32, &fd32);
            result = fd32;
            break;
        case VR4300_FMT_D:
            fpu_neg_64(&fs, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_CP1_ROUND_L(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32;
    uint64_t                  result;
    uint32_t                  saved_state = fpu_get_state();
    fpu_set_state((saved_state & ~0x6000) | 0x0000);
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            fpu_cvt_i64_f32(&fs32, &result);
            break;
        case VR4300_FMT_D:
            fpu_cvt_i64_f64(&fs, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    fpu_set_state((saved_state & 0x6000) | (fpu_get_state() & ~0x6000));
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(5);
}
int vr4300::VR4300_CP1_ROUND_W(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32;
    uint32_t                  result;
    uint32_t                  saved_state = fpu_get_state();
    fpu_set_state((saved_state & ~0x6000) | 0x0000);
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            fpu_cvt_i32_f32(&fs32, &result);
            break;
        case VR4300_FMT_D:
            fpu_cvt_i32_f64(&fs, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    fpu_set_state((saved_state & 0x6000) | (fpu_get_state() & ~0x6000));
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(5);
}
int vr4300::VR4300_SDC1(uint32_t iw, uint64_t rs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    exdc_latch->request.vaddr            = rs + (int16_t)iw;
    exdc_latch->request.data             = ft;
    exdc_latch->request.wdqm             = ~0ULL;
    exdc_latch->request.access_type      = VR4300_ACCESS_DWORD;
    exdc_latch->request.type             = VR4300_BUS_REQUEST_WRITE;
    exdc_latch->request.size             = 8;
    return 0;
}
int vr4300::VR4300_CP1_SQRT(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32, fd32;
    uint64_t                  result;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            fpu_sqrt_32(&fs32, &fd32);
            result = fd32;
            break;
        case VR4300_FMT_D:
            fpu_sqrt_64(&fs, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(fmt == VR4300_FMT_D ? 58 : 29);
}
int vr4300::VR4300_CP1_SUB(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32, ft32, fd32;
    uint64_t                  result;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            ft32 = ft;
            fpu_sub_32(&fs32, &ft32, &fd32);
            result = fd32;
            break;
        case VR4300_FMT_D:
            fpu_sub_64(&fs, &ft, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(3);
}
int vr4300::VR4300_SWC1(uint32_t iw, uint64_t rs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint32_t                  status     = g_vr4300->regs[VR4300_CP0_REGISTER_STATUS];
    unsigned                  ft_reg     = (((iw) >> 16 & 0x1F) + VR4300_REGISTER_CP1_0);
    if (!(status & 0x04000000))
        ft >>= ((ft_reg & 0x1) << 5);
    exdc_latch->request.vaddr       = rs + (int16_t)iw;
    exdc_latch->request.data        = ft;
    exdc_latch->request.wdqm        = ~0U;
    exdc_latch->request.access_type = VR4300_ACCESS_WORD;
    exdc_latch->request.type        = VR4300_BUS_REQUEST_WRITE;
    exdc_latch->request.size        = 4;
    return 0;
}
int vr4300::VR4300_CP1_TRUNC_L(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32;
    uint64_t                  result;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            fpu_trunc_i64_f32(&fs32, &result);
            break;
        case VR4300_FMT_D:
            fpu_trunc_i64_f64(&fs, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(5);
}
int vr4300::VR4300_CP1_TRUNC_W(uint32_t iw, uint64_t fs, uint64_t ft)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    enum vr4300_fmt           fmt        = ((vr4300_fmt)((iw) >> 21 & 0x1F));
    unsigned                  dest       = (((iw) >> 6 & 0x1F) + VR4300_REGISTER_CP1_0);
    uint32_t                  fs32;
    uint32_t                  result;
    switch (fmt) {
        case VR4300_FMT_S:
            fs32 = fs;
            fpu_trunc_i32_f32(&fs32, &result);
            break;
        case VR4300_FMT_D:
            fpu_trunc_i32_f64(&fs, &result);
            break;
        default:
            VR4300_INV();
            return 1;
    }
    exdc_latch->result = result;
    exdc_latch->dest   = dest;
    return vr4300_do_mci(5);
}
void vr4300::vr4300_cp1_init()
{
    fpu_set_state(0x0000 | 0x1F80);
    g_vr4300->regs[VR4300_CP1_FCR0] = 0xa00;
}
struct vr4300_dcache_line *vr4300::get_line(struct vr4300_dcache *dcache, uint64_t vaddr)
{
    return dcache->lines + (vaddr >> 4 & 0x1FF);
}
uint32_t vr4300::get_tag(const struct vr4300_dcache_line *line)
{
    return line->metadata & ~0xFFFU;
}
void vr4300::invalidate_line(struct vr4300_dcache_line *line)
{
    line->metadata &= ~0x1;
}
bool vr4300::is_dirty(const struct vr4300_dcache_line *line)
{
    return (line->metadata & 0x2) == 0x2;
}
bool vr4300::is_valid(const struct vr4300_dcache_line *line)
{
    return (line->metadata & 0x1) == 0x1;
}
void vr4300::set_dirty(struct vr4300_dcache_line *line)
{
    line->metadata |= 0x2;
}
void vr4300::set_tag(struct vr4300_dcache_line *line, uint32_t tag)
{
    line->metadata = tag | (line->metadata & 0x1);
}
void vr4300::set_taglo(struct vr4300_dcache_line *line, uint32_t taglo)
{
    line->metadata = (taglo << 4 & 0xFFFFF000U) | (taglo >> 7 & 0x1) | (taglo >> 5 & 0x2);
}
void vr4300::validate_line(struct vr4300_dcache_line *line, uint32_t tag)
{
    line->metadata = tag | 0x1;
}
void vr4300::vr4300_dcache_create_dirty_exclusive(struct vr4300_dcache *dcache, uint64_t vaddr, uint32_t paddr)
{
    struct vr4300_dcache_line *line = get_line(dcache, vaddr);
    set_tag(line, paddr & ~0xFFFU);
    line->metadata |= 0x3;
}
void vr4300::vr4300_dcache_fill(struct vr4300_dcache *dcache, uint64_t vaddr, uint32_t paddr, const void *data)
{
    struct vr4300_dcache_line *line = get_line(dcache, vaddr);
    memcpy(line->data, data, sizeof(line->data));
    validate_line(line, paddr & ~0xFFFU);
}
uint32_t vr4300::vr4300_dcache_get_tag(const struct vr4300_dcache_line *line, uint64_t vaddr)
{
    return get_tag(line) | (vaddr & 0xFF0U);
}
uint32_t vr4300::vr4300_dcache_get_taglo(struct vr4300_dcache *dcache, uint64_t vaddr)
{
    struct vr4300_dcache_line *line  = get_line(dcache, vaddr);
    uint32_t                   taglo = ((line->metadata & 0x1) << 1) | ((line->metadata & 0x2) >> 1);
    return (taglo << 6) | (line->metadata >> 4 & 0x0FFFFF00U);
}
void vr4300::vr4300_dcache_init(struct vr4300_dcache *dcache)
{
}
void vr4300::vr4300_dcache_invalidate(struct vr4300_dcache_line *line)
{
    invalidate_line(line);
}
void vr4300::vr4300_dcache_invalidate_hit(struct vr4300_dcache *dcache, uint64_t vaddr, uint32_t paddr)
{
    struct vr4300_dcache_line *line = get_line(dcache, vaddr);
    uint32_t                   ptag = get_tag(line);
    if (ptag == (paddr & ~0xFFFU) && is_valid(line))
        invalidate_line(line);
}
struct vr4300_dcache_line *vr4300::vr4300_dcache_probe(struct vr4300_dcache *dcache, uint64_t vaddr, uint32_t paddr)
{
    struct vr4300_dcache_line *line = get_line(dcache, vaddr);
    uint32_t                   ptag = get_tag(line);
    if (ptag == (paddr & ~0xFFFU) && is_valid(line))
        return line;
    return __null;
}
void vr4300::vr4300_dcache_set_dirty(struct vr4300_dcache_line *line)
{
    set_dirty(line);
}
void vr4300::vr4300_dcache_set_taglo(struct vr4300_dcache *dcache, uint64_t vaddr, uint32_t taglo)
{
    struct vr4300_dcache_line *line = get_line(dcache, vaddr);
    set_taglo(line, taglo);
}
struct vr4300_dcache_line *vr4300::vr4300_dcache_should_flush_line(struct vr4300_dcache *dcache, uint64_t vaddr)
{
    struct vr4300_dcache_line *line = get_line(dcache, vaddr);
    return is_dirty(line) && is_valid(line) ? line : __null;
}
struct vr4300_dcache_line *vr4300::vr4300_dcache_wb_invalidate(struct vr4300_dcache *dcache, uint64_t vaddr)
{
    struct vr4300_dcache_line *line = get_line(dcache, vaddr);
    if (is_valid(line)) {
        invalidate_line(line);
        return line;
    }
    return __null;
}
void vr4300::vr4300_common_interlocks(unsigned cycles_to_stall, unsigned skip_stages)
{
    struct vr4300_pipeline *pipeline    = &g_vr4300->pipeline;
    pipeline->cycles_to_stall           = cycles_to_stall;
    g_vr4300->regs[PIPELINE_CYCLE_TYPE] = skip_stages;
}
void vr4300::vr4300_dc_fault(enum vr4300_fault_id fault)
{
    struct vr4300_pipeline *pipeline  = &g_vr4300->pipeline;
    pipeline->dcwb_latch.common.fault = fault;
    pipeline->exdc_latch.common.fault = fault;
    pipeline->rfex_latch.common.fault = fault;
    pipeline->icrf_latch.common.fault = fault;
}
void vr4300::vr4300_ex_fault(enum vr4300_fault_id fault)
{
    struct vr4300_pipeline *pipeline  = &g_vr4300->pipeline;
    pipeline->exdc_latch.common.fault = fault;
    pipeline->rfex_latch.common.fault = fault;
    pipeline->icrf_latch.common.fault = fault;
}
void vr4300::vr4300_rf_fault(enum vr4300_fault_id fault)
{
    struct vr4300_pipeline *pipeline  = &g_vr4300->pipeline;
    pipeline->rfex_latch.common.fault = fault;
    pipeline->icrf_latch.common.fault = fault;
}
void vr4300::vr4300_exception_prolog(const struct vr4300_latch *l, uint32_t *cause, uint32_t *status, uint64_t *epc)
{
    bool in_bd_slot = l->cause_data >> 31;
    *status         = g_vr4300->regs[VR4300_CP0_REGISTER_STATUS];
    *cause          = g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE];
    *epc            = g_vr4300->regs[VR4300_CP0_REGISTER_EPC];
    if (!(*status & 0x2)) {
        if (in_bd_slot) {
            *cause |= 0x80000000U;
            *epc = l->pc - 4;
        } else {
            *cause &= ~0x80000000U;
            *epc = l->pc;
        }
    }
}
void vr4300::vr4300_tlb_exception_prolog(const struct vr4300_latch *l, uint32_t *cause, uint32_t *status, uint64_t *epc,
                                         const struct segment *segment, uint64_t vaddr, unsigned *offs)
{
    bool     in_bd_slot = l->cause_data >> 31;
    uint64_t entryhi;
    uint64_t context;
    *status = g_vr4300->regs[VR4300_CP0_REGISTER_STATUS];
    *cause  = g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE];
    *epc    = g_vr4300->regs[VR4300_CP0_REGISTER_EPC];
    if (in_bd_slot) {
        if (!(*status & 0x2)) {
            *cause |= 0x80000000U;
            *epc  = l->pc - 4;
            *offs = (*status & segment->xmode_mask) ? 0x080 : 0x000;
        } else
            *offs = 0x180;
    } else {
        if (!(*status & 0x2)) {
            *cause &= ~0x80000000U;
            *epc  = l->pc;
            *offs = (*status & segment->xmode_mask) ? 0x080 : 0x000;
        } else
            *offs = 0x180;
    }
    if (*status & segment->xmode_mask) {
        uint64_t vpn2   = vaddr >> 13 & 0x7FFFFFF;
        uint8_t  asid   = g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYHI];
        uint8_t  region = vaddr >> 62 & 0x3;
        entryhi         = ((uint64_t)region << 62) | (vpn2 << 13) | asid;
        context         = g_vr4300->regs[VR4300_CP0_REGISTER_XCONTEXT];
        context &= ~(0x1FFFFFFFULL << 4);
        context |= (uint64_t)region << 31;
        context |= vpn2 << 4;
        g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYHI]  = entryhi;
        g_vr4300->regs[VR4300_CP0_REGISTER_XCONTEXT] = context;
    } else {
        uint32_t vpn2 = vaddr >> 13 & 0x7FFFF;
        uint8_t  asid = g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYHI];
        entryhi       = (int32_t)((vpn2 << 13) | asid);
        context       = g_vr4300->regs[VR4300_CP0_REGISTER_CONTEXT];
        context &= ~(0x7FFFFULL << 4);
        context |= vpn2 << 4;
        g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYHI] = entryhi;
        g_vr4300->regs[VR4300_CP0_REGISTER_CONTEXT] = context;
    }
    g_vr4300->regs[VR4300_CP0_REGISTER_BADVADDR] = vaddr;
}
void vr4300::vr4300_exception_epilogue(uint32_t cause, uint32_t status, uint64_t epc, uint64_t offs)
{
    struct vr4300_pipeline *pipeline    = &g_vr4300->pipeline;
    pipeline->icrf_latch.segment        = get_default_segment();
    pipeline->exdc_latch.segment        = get_default_segment();
    g_vr4300->regs[PIPELINE_CYCLE_TYPE] = 0;
    pipeline->exception_history         = 0;
    pipeline->fault_present             = true;
    pipeline->cycles_to_stall           = 2;
    if (g_vr4300->regs[VR4300_CP0_REGISTER_STATUS] & 0x2) {
        g_vr4300->regs[VR4300_CP0_REGISTER_STATUS]   = status | 0x4;
        g_vr4300->regs[VR4300_CP0_REGISTER_ERROREPC] = epc;
    } else {
        g_vr4300->regs[VR4300_CP0_REGISTER_STATUS] = status | 0x2;
        g_vr4300->regs[VR4300_CP0_REGISTER_EPC]    = epc;
    }
    g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE] = cause;
    g_vr4300->pipeline.icrf_latch.pc =
        ((status & 0x400000) ? (0xFFFFFFFFBFC00200ULL + offs) : (0xFFFFFFFF80000000ULL + offs));
}
void vr4300::VR4300_CPU()
{
    struct vr4300_latch *common = &g_vr4300->pipeline.exdc_latch.common;
    uint32_t             cause, status;
    uint64_t             epc;
    vr4300_ex_fault(VR4300_FAULT_CPU);
    vr4300_exception_prolog(common, &cause, &status, &epc);
    vr4300_exception_epilogue((cause & ~0xFF) | (1 << 28) | 0x2C, status, epc, 0x180);
}
void vr4300::VR4300_DADE()
{
    struct vr4300_latch *common = &g_vr4300->pipeline.exdc_latch.common;
    uint32_t             cause, status;
    uint64_t             epc;
    vr4300_ex_fault(VR4300_FAULT_CPU);
    vr4300_exception_prolog(common, &cause, &status, &epc);
    vr4300_exception_epilogue((cause & ~0xFF) | (1 << 28) | 0x10, status, epc, 0x180);
}
void vr4300::VR4300_DCB()
{
    g_vr4300->pipeline.dcwb_latch.last_op_was_cache_store = false;
    vr4300_common_interlocks(0, 1);
}
void vr4300::VR4300_DCM()
{
    struct vr4300_dcwb_latch  *dcwb_latch = &g_vr4300->pipeline.dcwb_latch;
    struct vr4300_exdc_latch  *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    struct vr4300_bus_request *request    = &exdc_latch->request;
    uint64_t                   vaddr      = request->vaddr;
    uint32_t                   paddr      = request->paddr;
    struct vr4300_dcache_line *line;
    uint32_t                   data[4];
    unsigned                   i;
    if (!exdc_latch->cached) {
        unsigned mask = request->access_type == VR4300_ACCESS_DWORD ? 0x7 : 0x3;
        if (exdc_latch->request.type == VR4300_BUS_REQUEST_READ) {
            unsigned rshiftamt = (8 - request->size) << 3;
            unsigned lshiftamt = (paddr & mask) << 3;
            uint32_t hiword, loword;
            int64_t  sdata;
            paddr &= ~mask;
            bus_read_word(g_vr4300->bus, paddr, &hiword);
            if (request->access_type != VR4300_ACCESS_DWORD)
                sdata = (uint64_t)hiword << (lshiftamt + 32);
            else {
                bus_read_word(g_vr4300->bus, paddr + 4, &loword);
                sdata = ((uint64_t)hiword << 32) | loword;
                sdata = sdata << lshiftamt;
            }
            dcwb_latch->result |= (sdata >> rshiftamt & request->data) << request->postshift;
        } else {
            uint64_t data = request->data;
            uint64_t dqm  = request->wdqm;
            paddr &= ~mask;
            if (request->access_type == VR4300_ACCESS_DWORD) {
                bus_write_word(g_vr4300->bus, paddr, data >> 32, dqm >> 32);
                paddr += 4;
            }
            bus_write_word(g_vr4300->bus, paddr, data, dqm);
        }
        vr4300_common_interlocks(38, 2);
        return;
    }
    if ((line = vr4300_dcache_should_flush_line(&g_vr4300->dcache, vaddr)) != __null) {
        uint32_t bus_address;
        bus_address = vr4300_dcache_get_tag(line, vaddr);
        memcpy(data, line->data, sizeof(data));
        for (i = 0; i < 4; i++)
            bus_write_word(g_vr4300->bus, bus_address + i * 4, data[i ^ (4 >> 2)], ~0);
    }
    vr4300_common_interlocks((46 - 2), 1);
    paddr &= ~0xF;
    for (i = 0; i < 4; i++)
        bus_read_word(g_vr4300->bus, paddr + i * 4, data + (i ^ (4 >> 2)));
    vr4300_dcache_fill(&g_vr4300->dcache, vaddr, paddr, data);
}
void vr4300::VR4300_DTLB(unsigned miss, unsigned inv, unsigned mod)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    struct vr4300_latch      *common     = &exdc_latch->common;
    uint32_t                  cause, status;
    uint64_t                  epc;
    unsigned                  offs, type;
    if (miss | inv)
        type = (exdc_latch->request.type == VR4300_BUS_REQUEST_WRITE) ? 0x3 : 0x2;
    else
        type = 0x1;
    vr4300_dc_fault(VR4300_FAULT_DTLB);
    vr4300_tlb_exception_prolog(common, &cause, &status, &epc, exdc_latch->segment, exdc_latch->request.vaddr, &offs);
    if (!miss && !mod)
        offs = 0x180;
    vr4300_exception_epilogue((cause & ~0xFF) | (type << 2), status, epc, offs);
}
void vr4300::VR4300_IADE()
{
    abort();
}
void vr4300::VR4300_ICB()
{
    struct vr4300_icrf_latch *icrf_latch = &g_vr4300->pipeline.icrf_latch;
    struct vr4300_rfex_latch *rfex_latch = &g_vr4300->pipeline.rfex_latch;
    uint64_t                  vaddr      = icrf_latch->common.pc;
    uint32_t                  paddr      = rfex_latch->paddr;
    unsigned                  delay;
    if (!rfex_latch->cached) {
        bus_read_word(g_vr4300->bus, paddr, &rfex_latch->iw);
        delay = 38;
    } else {
        uint32_t line[8];
        unsigned i;
        paddr &= ~0x1C;
        for (i = 0; i < 8; i++)
            bus_read_word(g_vr4300->bus, paddr + i * 4, line + i);
        memcpy(&rfex_latch->iw, line + (vaddr >> 2 & 0x7), sizeof(rfex_latch->iw));
        vr4300_icache_fill(&g_vr4300->icache, icrf_latch->common.pc, paddr, line);
        delay = (50 - 2);
    }
    vr4300_common_interlocks(delay, 4);
}
void vr4300::VR4300_INTR()
{
    struct vr4300_pipeline *pipeline = &g_vr4300->pipeline;
    struct vr4300_latch    *common   = &pipeline->dcwb_latch.common;
    uint32_t                cause, status;
    uint64_t                epc;
    vr4300_exception_prolog(common, &cause, &status, &epc);
    vr4300_exception_epilogue(cause & ~0xFF, status, epc, 0x180);
    vr4300_dc_fault(VR4300_FAULT_INTR);
}
void vr4300::VR4300_INV()
{
    abort();
}
void vr4300::VR4300_ITLB(unsigned miss)
{
    struct vr4300_icrf_latch *icrf_latch = &g_vr4300->pipeline.icrf_latch;
    struct vr4300_latch      *common     = &icrf_latch->common;
    uint32_t                  cause, status;
    uint64_t                  epc;
    unsigned                  offs;
    vr4300_rf_fault(VR4300_FAULT_ITLB);
    vr4300_tlb_exception_prolog(common, &cause, &status, &epc, icrf_latch->segment, icrf_latch->common.pc, &offs);
    if (!miss)
        offs = 0x180;
    vr4300_exception_epilogue((cause & ~0xFF) | (0x2 << 2), status, epc, offs);
}
void vr4300::VR4300_LDI()
{
    struct vr4300_pipeline   *pipeline   = &g_vr4300->pipeline;
    struct vr4300_exdc_latch *exdc_latch = &pipeline->exdc_latch;
    exdc_latch->request.type             = VR4300_BUS_REQUEST_NONE;
    vr4300_common_interlocks(0, 2);
}
void vr4300::VR4300_RST()
{
    if (g_vr4300->signals & VR4300_SIGNAL_COLDRESET) {
        g_vr4300->signals &= ~VR4300_SIGNAL_COLDRESET;
        g_vr4300->regs[VR4300_CP0_REGISTER_STATUS] &= ~0x08300000ULL;
        g_vr4300->regs[VR4300_CP0_REGISTER_CONFIG] &= ~0xFFFF7FF0ULL;
        g_vr4300->regs[VR4300_CP0_REGISTER_STATUS] |= 0x00400004ULL;
        g_vr4300->regs[VR4300_CP0_REGISTER_CONFIG] |= 0x7006E460ULL;
        g_vr4300->regs[VR4300_CP0_REGISTER_RANDOM] = 31;
    } else
        abort();
    vr4300_dc_fault(VR4300_FAULT_RST);
    vr4300_exception_epilogue(g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE], g_vr4300->regs[VR4300_CP0_REGISTER_STATUS],
                              g_vr4300->regs[VR4300_CP0_REGISTER_EPC], -0x200ULL);
}
void vr4300::VR4300_SYSC()
{
    struct vr4300_latch *common = &g_vr4300->pipeline.exdc_latch.common;
    uint32_t             cause, status;
    uint64_t             epc;
    vr4300_ex_fault(VR4300_FAULT_SYSC);
    vr4300_exception_prolog(common, &cause, &status, &epc);
    vr4300_exception_epilogue((cause & ~0xFF) | (8 << 2), status, epc, 0x180);
}
void vr4300::VR4300_BRPT()
{
    struct vr4300_latch *common = &g_vr4300->pipeline.exdc_latch.common;
    uint32_t             cause, status;
    uint64_t             epc;
    vr4300_ex_fault(VR4300_FAULT_BRPT);
    vr4300_exception_prolog(common, &cause, &status, &epc);
    vr4300_exception_epilogue((cause & ~0xFF) | (9 << 2), status, epc, 0x180);
}
void vr4300::VR4300_TRAP()
{
    struct vr4300_latch *common = &g_vr4300->pipeline.exdc_latch.common;
    uint32_t             cause, status;
    uint64_t             epc;
    vr4300_ex_fault(VR4300_FAULT_TRAP);
    vr4300_exception_prolog(common, &cause, &status, &epc);
    vr4300_exception_epilogue((cause & ~0xFF) | (13 << 2), status, epc, 0x180);
}
void vr4300::VR4300_RI()
{
    struct vr4300_latch *common = &g_vr4300->pipeline.exdc_latch.common;
    uint32_t             cause, status;
    uint64_t             epc;
    vr4300_ex_fault(VR4300_FAULT_RI);
    vr4300_exception_prolog(common, &cause, &status, &epc);
    vr4300_exception_epilogue((cause & ~0xFF) | (10 << 2), status, epc, 0x180);
}
void vr4300::VR4300_WAT()
{
    struct vr4300_pipeline *pipeline = &g_vr4300->pipeline;
    struct vr4300_latch    *common   = &pipeline->dcwb_latch.common;
    uint32_t                cause, status;
    uint64_t                epc;
    vr4300_exception_prolog(common, &cause, &status, &epc);
    vr4300_exception_epilogue((cause & ~0xFF) | (23 << 2), status, epc, 0x180);
    vr4300_dc_fault(VR4300_FAULT_WAT);
}
void vr4300::vr4300_ic_stage()
{
    struct vr4300_rfex_latch *rfex_latch = &g_vr4300->pipeline.rfex_latch;
    struct vr4300_icrf_latch *icrf_latch = &g_vr4300->pipeline.icrf_latch;
    const struct segment     *segment    = icrf_latch->segment;
    struct vr4300_opcode     *opcode     = &rfex_latch->opcode;
    uint64_t                  pc         = icrf_latch->pc;
    uint32_t                  decode_iw;
    decode_iw                     = rfex_latch->iw &= rfex_latch->iw_mask;
    *opcode                       = *vr4300_decode_instruction(decode_iw);
    rfex_latch->iw_mask           = ~0U;
    icrf_latch->common.pc         = pc;
    icrf_latch->pc                = pc + 4;
    icrf_latch->common.cause_data = (opcode->flags & (1U << 31));
    if ((pc - segment->start) >= segment->length) {
        uint32_t cp0_status = g_vr4300->regs[VR4300_CP0_REGISTER_STATUS];
        if (__builtin_expect(((segment = get_segment(pc, cp0_status)) == __null), 0))
            VR4300_IADE();
        icrf_latch->segment = segment;
    }
}
int vr4300::vr4300_rf_stage()
{
    const struct vr4300_icrf_latch  *icrf_latch = &g_vr4300->pipeline.icrf_latch;
    struct vr4300_rfex_latch        *rfex_latch = &g_vr4300->pipeline.rfex_latch;
    const struct segment            *segment    = icrf_latch->segment;
    const struct vr4300_icache_line *line;
    uint64_t                         vaddr = icrf_latch->common.pc;
    uint32_t                         paddr;
    bool                             cached;
    rfex_latch->common = icrf_latch->common;
    paddr              = vaddr - segment->offset;
    cached             = segment->cached;
    if (segment->mapped) {
        unsigned asid = g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYHI] & 0xFF;
        unsigned select, tlb_miss, index;
        uint32_t page_mask;
        tlb_miss  = tlb_probe(&g_vr4300->cp0.tlb, vaddr, asid, &index);
        page_mask = g_vr4300->cp0.page_mask[index];
        select    = ((page_mask + 1) & vaddr) != 0;
        if (__builtin_expect((tlb_miss || !(g_vr4300->cp0.state[index][select] & 2)), 0)) {
            VR4300_ITLB(tlb_miss);
            return 1;
        }
        cached = (g_vr4300->cp0.state[index][select] & 0x38) != 0x10;
        paddr  = (g_vr4300->cp0.pfn[index][select]) | (vaddr & page_mask);
    }
    line = vr4300_icache_probe(&g_vr4300->icache, vaddr, paddr);
    if (!(line && cached)) {
        rfex_latch->paddr  = paddr;
        rfex_latch->cached = cached;
        VR4300_ICB();
        return 1;
    }
    memcpy(&rfex_latch->iw, line->data + (paddr & 0x1C), sizeof(rfex_latch->iw));
    return 0;
}
int vr4300::vr4300_ex_stage()
{
    const struct vr4300_rfex_latch *rfex_latch = &g_vr4300->pipeline.rfex_latch;
    const struct vr4300_dcwb_latch *dcwb_latch = &g_vr4300->pipeline.dcwb_latch;
    struct vr4300_exdc_latch       *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint32_t                        cp0_status = g_vr4300->regs[VR4300_CP0_REGISTER_STATUS];
    unsigned                        rs, rt, rslutidx, rtlutidx;
    uint64_t                        rs_reg, rt_reg, temp;
    uint32_t                        flags, iw;
    exdc_latch->common = rfex_latch->common;
    flags              = rfex_latch->opcode.flags;
    iw                 = rfex_latch->iw;
    rs                 = ((iw) >> 21 & 0x1F);
    rt                 = ((iw) >> 16 & 0x1F);
    if (flags & (1U << 2)) {
        const unsigned fpu_select_lut[2] = {21, 11};
        unsigned       fr;
        if (__builtin_expect((!(cp0_status & 0x20000000U)), 0)) {
            VR4300_CPU();
            return 1;
        }
        fr       = (cp0_status >> 26 & 0x1) ^ 0x1;
        rtlutidx = flags & 0x2;
        rslutidx = flags & 0x1;
        rs       = (iw >> fpu_select_lut[rslutidx] & 0x1F) | (rslutidx << 6);
        rt |= (rtlutidx << 5);
        rs &= ~((rslutidx)&fr);
        rt &= ~((rtlutidx >> 1) & fr);
    }
    if (__builtin_expect(
            (exdc_latch->request.type == VR4300_BUS_REQUEST_READ &&
             (((exdc_latch->dest == rs) && (flags & (1U << 3))) || ((exdc_latch->dest == rt) && (flags & (1U << 4))))),
            0)) {
        VR4300_LDI();
        return 1;
    }
    temp                               = g_vr4300->regs[dcwb_latch->dest];
    g_vr4300->regs[dcwb_latch->dest]   = dcwb_latch->result;
    g_vr4300->regs[VR4300_REGISTER_R0] = 0x0000000000000000ULL;
    rs_reg                             = g_vr4300->regs[rs];
    rt_reg                             = g_vr4300->regs[rt];
    g_vr4300->regs[dcwb_latch->dest]   = temp;
    if (g_vr4300->profile_samples) {
        uint32_t idx = rfex_latch->common.pc - 0x80000000;
        idx &= (8 * 1024 * 1024) - 1;
        g_vr4300->profile_samples[idx]++;
    }
    exdc_latch->dest         = VR4300_REGISTER_R0;
    exdc_latch->request.type = VR4300_BUS_REQUEST_NONE;
    return CALL_MEMBER_FN(*this, vr4300_function_table[rfex_latch->opcode.id])(iw, rs_reg, rt_reg);
}
int vr4300::vr4300_dc_stage()
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    struct vr4300_dcwb_latch *dcwb_latch = &g_vr4300->pipeline.dcwb_latch;
    uint32_t                  cp0_status = g_vr4300->regs[VR4300_CP0_REGISTER_STATUS];
    uint32_t                  cp0_cause  = g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE];
    const struct segment     *segment    = exdc_latch->segment;
    bool                      cached;
    dcwb_latch->common = exdc_latch->common;
    dcwb_latch->result = exdc_latch->result;
    dcwb_latch->dest   = exdc_latch->dest;
    if (__builtin_expect((g_vr4300->signals & VR4300_SIGNAL_COLDRESET), 0)) {
        VR4300_RST();
        return 1;
    }
    if (__builtin_expect((cp0_cause & cp0_status & 0xFF00 && (((cp0_status ^ 6) & 0x7) == 0x7)), 0)) {
        VR4300_INTR();
        return 1;
    }
    if (exdc_latch->request.type != VR4300_BUS_REQUEST_NONE) {
        struct vr4300_bus_request *request = &exdc_latch->request;
        uint64_t                   vaddr   = exdc_latch->request.vaddr;
        struct vr4300_dcache_line *line;
        uint32_t                   paddr;
        if ((vaddr - segment->start) >= segment->length) {
            if (__builtin_expect(((segment = get_segment(vaddr, cp0_status)) == __null), 0)) {
                VR4300_DADE();
                return 1;
            }
            exdc_latch->segment = segment;
        }
        paddr  = vaddr - segment->offset;
        cached = segment->cached;
        if (segment->mapped) {
            unsigned asid = g_vr4300->regs[VR4300_CP0_REGISTER_ENTRYHI] & 0xFF;
            unsigned select, tlb_inv, tlb_miss, tlb_mod, index;
            uint32_t page_mask;
            tlb_miss  = tlb_probe(&g_vr4300->cp0.tlb, vaddr, asid, &index);
            page_mask = g_vr4300->cp0.page_mask[index];
            select    = ((page_mask + 1) & vaddr) != 0;
            tlb_inv   = !(g_vr4300->cp0.state[index][select] & 2);
            tlb_mod   = !(g_vr4300->cp0.state[index][select] & 4) && request->type == VR4300_BUS_REQUEST_WRITE;
            if (__builtin_expect((tlb_miss | tlb_inv | tlb_mod), 0)) {
                VR4300_DTLB(tlb_miss, tlb_inv, tlb_mod);
                return 1;
            }
            cached = ((g_vr4300->cp0.state[index][select] & 0x38) != 0x10);
            paddr  = (g_vr4300->cp0.pfn[index][select]) | (vaddr & page_mask);
        }
        if (__builtin_expect(((paddr & ~0x80000007U) == (g_vr4300->regs[VR4300_CP0_REGISTER_WATCHLO] & ~0x80000007U)),
                             0)) {
            if (g_vr4300->regs[VR4300_CP0_REGISTER_WATCHLO] & request->type & 0x3) {
                VR4300_WAT();
                return 1;
            }
        }
        if (__builtin_expect((exdc_latch->request.type < VR4300_BUS_REQUEST_CACHE), !0)) {
            uint64_t dword, rtemp, wtemp, wdqm;
            unsigned shiftamt, rshiftamt, lshiftamt;
            uint32_t s_paddr;
            line = vr4300_dcache_probe(&g_vr4300->dcache, vaddr, paddr);
            if (cached) {
                bool last_cache_was_store           = dcwb_latch->last_op_was_cache_store;
                dcwb_latch->last_op_was_cache_store = (exdc_latch->request.type == VR4300_BUS_REQUEST_WRITE);
                if (!line) {
                    request->paddr     = paddr;
                    exdc_latch->cached = cached;
                    if (g_vr4300->profile_samples) {
                        uint32_t idx = exdc_latch->common.pc - 0x80000000;
                        idx &= (8 * 1024 * 1024) - 1;
                        g_vr4300->profile_samples[idx + (8 * 1024 * 1024)]++;
                    }
                    g_vr4300->pipeline.cycles_to_stall  = 0;
                    g_vr4300->regs[PIPELINE_CYCLE_TYPE] = 6;
                    return 1;
                }
                if (last_cache_was_store) {
                    VR4300_DCB();
                    return 1;
                }
            } else {
                dcwb_latch->last_op_was_cache_store = false;
                request->paddr                      = paddr;
                exdc_latch->cached                  = cached;
                if (g_vr4300->profile_samples) {
                    uint32_t idx = exdc_latch->common.pc - 0x80000000;
                    idx &= (8 * 1024 * 1024) - 1;
                    g_vr4300->profile_samples[idx + (8 * 1024 * 1024)]++;
                }
                VR4300_DCM();
                return 1;
            }
            s_paddr = paddr << 3;
            paddr &= 0x8;
            memcpy(&dword, line->data + paddr, sizeof(dword));
            shiftamt  = (s_paddr ^ (4 << 3)) & request->access_type;
            rshiftamt = (8 - request->size) << 3;
            lshiftamt = (s_paddr & (0x7 << 3));
            wdqm      = request->wdqm << shiftamt;
            wtemp     = (request->data << shiftamt) & wdqm;
            rtemp     = ((int64_t)(dword << lshiftamt) >> rshiftamt) & request->data;
            dword     = (dword & ~wdqm) | wtemp;
            dcwb_latch->result |= rtemp << request->postshift;
            memcpy(line->data + paddr, &dword, sizeof(dword));
            line->metadata |= exdc_latch->request.type;
        } else {
            unsigned delay;
            if ((delay = request->cacheop(vaddr, paddr))) {
                g_vr4300->pipeline.cycles_to_stall  = delay - 1;
                g_vr4300->regs[PIPELINE_CYCLE_TYPE] = 2;
                return 1;
            }
        }
    }
    return 0;
}
int vr4300::vr4300_wb_stage()
{
    const struct vr4300_dcwb_latch *dcwb_latch = &g_vr4300->pipeline.dcwb_latch;
    g_vr4300->regs[dcwb_latch->dest]           = dcwb_latch->result;
    return 0;
}
void vr4300::vr4300_cycle_slow_wb()
{
    struct vr4300_pipeline   *pipeline   = &g_vr4300->pipeline;
    struct vr4300_dcwb_latch *dcwb_latch = &pipeline->dcwb_latch;
    if (pipeline->exception_history++ > 3)
        pipeline->fault_present = false;
    if (dcwb_latch->common.fault == VR4300_FAULT_NONE) {
        if (vr4300_wb_stage())
            return;
    }
    vr4300_cycle_slow_dc();
}
void vr4300::vr4300_cycle_slow_dc()
{
    struct vr4300_pipeline   *pipeline   = &g_vr4300->pipeline;
    struct vr4300_dcwb_latch *dcwb_latch = &pipeline->dcwb_latch;
    struct vr4300_exdc_latch *exdc_latch = &pipeline->exdc_latch;
    if (exdc_latch->common.fault == VR4300_FAULT_NONE) {
        if (vr4300_dc_stage())
            return;
    } else {
        dcwb_latch->common = exdc_latch->common;
        dcwb_latch->dest   = 0;
    }
    vr4300_cycle_slow_ex();
}
void vr4300::vr4300_cycle_slow_ex()
{
    struct vr4300_pipeline   *pipeline   = &g_vr4300->pipeline;
    struct vr4300_exdc_latch *exdc_latch = &pipeline->exdc_latch;
    struct vr4300_rfex_latch *rfex_latch = &pipeline->rfex_latch;
    if (rfex_latch->common.fault == VR4300_FAULT_NONE) {
        if (vr4300_ex_stage())
            return;
    } else {
        exdc_latch->common = rfex_latch->common;
        exdc_latch->dest   = 0;
    }
    vr4300_cycle_slow_rf();
}
void vr4300::vr4300_cycle_slow_rf()
{
    struct vr4300_pipeline   *pipeline   = &g_vr4300->pipeline;
    struct vr4300_rfex_latch *rfex_latch = &pipeline->rfex_latch;
    struct vr4300_icrf_latch *icrf_latch = &pipeline->icrf_latch;
    if (icrf_latch->common.fault == VR4300_FAULT_NONE) {
        if (vr4300_rf_stage())
            return;
    } else
        rfex_latch->common = icrf_latch->common;
    vr4300_cycle_slow_ic();
}
void vr4300::vr4300_cycle_slow_ic()
{
    g_vr4300->pipeline.icrf_latch.common.fault = VR4300_FAULT_NONE;
    g_vr4300->regs[PIPELINE_CYCLE_TYPE]        = 0;
    vr4300_ic_stage();
}
void vr4300::vr4300_cycle_busywait()
{
    uint32_t cp0_status = g_vr4300->regs[VR4300_CP0_REGISTER_STATUS];
    uint32_t cp0_cause  = g_vr4300->regs[VR4300_CP0_REGISTER_CAUSE];
    if (__builtin_expect((cp0_cause & cp0_status & 0xFF00), 0) && (cp0_status & 0x1) && !(cp0_status & 0x6)) {
        VR4300_INTR();
    }
}
void vr4300::vr4300_cycle_()
{
    struct vr4300_pipeline *pipeline = &g_vr4300->pipeline;
    if (pipeline->fault_present + g_vr4300->regs[PIPELINE_CYCLE_TYPE])
        CALL_MEMBER_FN(*this, pipeline_function_lut[regs[PIPELINE_CYCLE_TYPE]])();
    else {
        if (vr4300_wb_stage())
            return;
        if (vr4300_dc_stage())
            return;
        if (vr4300_ex_stage())
            return;
        if (vr4300_rf_stage())
            return;
        vr4300_ic_stage();
    }
}
void vr4300::vr4300_cycle_extra(struct vr4300_stats *stats)
{
    struct vr4300_dcwb_latch *dcwb_latch = &g_vr4300->pipeline.dcwb_latch;
    struct vr4300_rfex_latch *rfex_latch = &g_vr4300->pipeline.rfex_latch;
    stats->executed_instructions += !dcwb_latch->common.fault && !g_vr4300->pipeline.cycles_to_stall;
    stats->total_cycles++;
    stats->opcode_counts[rfex_latch->opcode.id]++;
}
void vr4300::vr4300_pipeline_init(struct vr4300_pipeline *pipeline)
{
    pipeline->icrf_latch.segment = get_default_segment();
    pipeline->exdc_latch.segment = get_default_segment();
}
struct vr4300_icache_line *vr4300::get_line(struct vr4300_icache *icache, uint64_t vaddr)
{
    return icache->lines + (vaddr >> 5 & 0x1FF);
}
const struct vr4300_icache_line *vr4300::get_line_const(const struct vr4300_icache *icache, uint64_t vaddr)
{
    return icache->lines + (vaddr >> 5 & 0x1FF);
}
uint32_t vr4300::get_tag(const struct vr4300_icache_line *line)
{
    return line->metadata & ~0xFFFU;
}
void vr4300::invalidate_line(struct vr4300_icache_line *line)
{
    line->metadata &= ~0x1;
}
bool vr4300::is_valid(const struct vr4300_icache_line *line)
{
    return (line->metadata & 0x1) == 0x1;
}
void vr4300::set_taglo(struct vr4300_icache_line *line, uint32_t taglo)
{
    line->metadata = (taglo << 4 & 0xFFFFF000) | (taglo >> 7 & 0x1);
}
void vr4300::validate_line(struct vr4300_icache_line *line, uint32_t tag)
{
    line->metadata = tag | 0x1;
}
void vr4300::vr4300_icache_fill(struct vr4300_icache *icache, uint64_t vaddr, uint32_t paddr, const void *data)
{
    struct vr4300_icache_line *line = get_line(icache, vaddr);
    memcpy(line->data, data, sizeof(line->data));
    validate_line(line, paddr & ~0xFFFU);
}
uint32_t vr4300::vr4300_icache_get_tag(const struct vr4300_icache *icache, uint64_t vaddr)
{
    const struct vr4300_icache_line *line = get_line_const(icache, vaddr);
    return get_tag(line) | (vaddr & 0xFE0);
}
void vr4300::vr4300_icache_init(struct vr4300_icache *icache)
{
}
void vr4300::vr4300_icache_invalidate(struct vr4300_icache *icache, uint64_t vaddr)
{
    struct vr4300_icache_line *line = get_line(icache, vaddr);
    invalidate_line(line);
}
void vr4300::vr4300_icache_invalidate_hit(struct vr4300_icache *icache, uint64_t vaddr, uint32_t paddr)
{
    struct vr4300_icache_line *line = get_line(icache, vaddr);
    uint32_t                   ptag = get_tag(line);
    if (ptag == (paddr & ~0xFFFU) && is_valid(line))
        invalidate_line(line);
}
const struct vr4300_icache_line *vr4300::vr4300_icache_probe(const struct vr4300_icache *icache, uint64_t vaddr,
                                                             uint32_t paddr)
{
    const struct vr4300_icache_line *line = get_line_const(icache, vaddr);
    uint32_t                         ptag = get_tag(line);
    if (ptag == (paddr & ~0xFFFU) && is_valid(line))
        return line;
    return __null;
}
void vr4300_icache_set_taglo(struct vr4300_icache *icache, uint64_t vaddr, uint32_t taglo)
{
    struct vr4300_icache_line *line = g_vr4300->get_line(icache, vaddr);
    g_vr4300->set_taglo(line, taglo);
}
const struct segment *vr4300::get_default_segment(void)
{
    return &default_segment;
}
const struct segment *vr4300::get_segment(uint64_t address, uint32_t cp0_status)
{
    const struct segment *seg;
    const uint8_t         segment_mode_lut[256] = {
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                sizeof(*seg),
                sizeof(*seg),
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                sizeof(*seg),
                sizeof(*seg),
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                sizeof(*seg),
                sizeof(*seg),
                0,
                0,
                0,
                0,
                0,
                0,
                sizeof(*seg),
                sizeof(*seg),
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                0,
                0,
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                0,
                0,
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                0,
                0,
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                0,
                0,
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                0,
                0,
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                0,
                0,
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                0,
                0,
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                0,
                0,
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
                sizeof(*seg),
    };
    unsigned segment_mode        = segment_mode_lut[cp0_status & 0xFF];
    unsigned mode_and_flags_mask = cp0_status & 0x1E;
    uint64_t sexaddress          = (int64_t)((int32_t)address);
    char     kernel              = (cp0_status & 0x6) || ((cp0_status & 0x18) == 0);
    char     supervisor          = ((cp0_status & 0x6) == 0) && ((cp0_status & 0x18) == 0x8);
    char     user                = ((cp0_status & 0x6) == 0) && ((cp0_status & 0x18) == 0x10);
    char     use_kx              = kernel && (cp0_status & 0x80);
    char     use_sx              = supervisor && (cp0_status & 0x40);
    char     use_ux              = user && (cp0_status & 0x20);
    char     use_64              = use_kx | use_sx | use_ux;


    if (!use_64 && (sexaddress != address))
        return __null;
    seg = (const struct segment *)((uintptr_t)USEGs + segment_mode);
    if (address < seg->length)
        return seg;
    else if (mode_and_flags_mask != 0x10) {
        seg = &KSEGs[2];
        if ((address >> 40) == 0x400000)
            return &XSSEG;
        else if (mode_and_flags_mask != 0x08) {
            if (address >= KSEGs[0].start)
                return KSEGs + (address >> 29 & 0x3);
            else if ((address - XKPHYS0.start) < 0x400000FF80000000ULL)
                seg = kernel_segs_lut[address >> 59 & 0xF];
        }
        if (__builtin_expect(((address - seg->start) < seg->length), !0))
            return seg;
    }
    return __null;
}
void vr4300::tlb_init(struct n64_tlb *tlb)
{
    unsigned i;
    for (i = 0; i < 32; i++) {
        tlb->vpn2.data[i] = ~0;
        tlb->valid[i]     = 0;
    }
}
unsigned vr4300::tlb_probe(const struct n64_tlb *tlb, uint64_t vaddr, uint8_t vasid, unsigned *index)
{
    int      one_hot_idx;
    uint32_t vpn2;
    unsigned i;
    vpn2         = (vaddr >> 35 & 0x18000000U) | (vaddr >> 13 & 0x7FFFFFF);
    __m128i vpn  = _mm_set1_epi32(vpn2);
    __m128i asid = _mm_set1_epi8(vasid);
    for (i = 0; i < 32; i += 8) {
        __m128i check_l, check_h, vpn_check;
        __m128i check_a, check_g, asid_check;
        __m128i check;
        __m128i page_mask_l = _mm_load_si128((__m128i *)(tlb->page_mask.data + i + 0));
        __m128i page_mask_h = _mm_load_si128((__m128i *)(tlb->page_mask.data + i + 4));
        __m128i vpn_l       = _mm_load_si128((__m128i *)(tlb->vpn2.data + i + 0));
        __m128i vpn_h       = _mm_load_si128((__m128i *)(tlb->vpn2.data + i + 4));
        check_l             = _mm_and_si128(vpn, page_mask_l);
        check_l             = _mm_cmpeq_epi32(check_l, vpn_l);
        check_h             = _mm_and_si128(vpn, page_mask_h);
        check_h             = _mm_cmpeq_epi32(check_h, vpn_h);
        vpn_check           = _mm_packs_epi32(check_l, check_h);
        vpn_check           = _mm_packs_epi16(vpn_check, vpn_check);
        check_g             = _mm_loadl_epi64((__m128i *)(tlb->global + i));
        check_a             = _mm_loadl_epi64((__m128i *)(tlb->asid + i));
        asid_check          = _mm_cmpeq_epi8(check_a, asid);
        asid_check          = _mm_or_si128(check_g, asid_check);
        check               = _mm_and_si128(vpn_check, asid_check);
        if ((one_hot_idx = _mm_movemask_epi8(check)) != 0) {
            *index = i + n64_one_hot_lut[one_hot_idx & 0xFF];
            return 0;
        }
    }
    *index = 0;
    return 1;
}
int vr4300::tlb_read(const struct n64_tlb *tlb, unsigned index, uint64_t *entry_hi)
{
    *entry_hi = ((tlb->vpn2.data[index] & 0x18000000LLU) << 35) | ((tlb->vpn2.data[index] & 0x7FFFFFFLLU) << 13) |
                ((tlb->global[index] & 1) << 12) | (tlb->asid[index]);
    return 0;
}
int vr4300::tlb_write(struct n64_tlb *tlb, unsigned index, uint64_t entry_hi, uint64_t entry_lo_0, uint64_t entry_lo_1,
                      uint32_t page_mask)
{
    tlb->page_mask.data[index] = ~(page_mask >> 13);
    tlb->vpn2.data[index]      = (entry_hi >> 35 & 0x18000000U) | (entry_hi >> 13 & 0x7FFFFFF);
    tlb->global[index]         = (entry_lo_0 & 0x1) && (entry_lo_1 & 0x1) ? 0xFF : 0x00;
    tlb->valid[index]          = (entry_lo_0 & 0x2) || (entry_lo_1 & 0x2) ? 0xFF : 0x00;
    tlb->asid[index]           = entry_hi & 0xFF;
    return 0;
}
const struct vr4300_opcode *vr4300::vr4300_decode_instruction(uint32_t iw)
{
    const struct vr4300_opcode_escape *escape = vr4300_escape_table + (iw >> 25);
    unsigned                           index  = iw >> escape->shift & escape->mask;
    const struct vr4300_opcode        *group  = vr4300_opcode_table + escape->offset;
    return group + index;
}
uint64_t vr4300::vr4300_addsub_mask(uint32_t iw)
{
    uint64_t mask;
    __asm__("shr $2,       %k[iwiw];"
            "sbb %q[mask], %q[mask];"
            : [mask] "=r"(mask), [iwiw] "+r"(iw)
            :
            : "cc");
    return mask;
}
uint32_t vr4300::vr4300_branch_mask(uint32_t iw, unsigned index)
{
    iw = (uint32_t)((int32_t)(iw << (31 - index)) >> 31);
    return ~iw;
}
uint64_t vr4300::VR4300_LWR_forceset(unsigned offset)
{
    uint64_t mask;
    __asm__("add $4294967293, %k[offs];"
            "sbb %q[mask],    %q[mask];"
            : [mask] "=r"(mask), [offs] "+r"(offset)
            :
            : "cc");
    return mask;
}
int vr4300::VR4300_ADD_SUB(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint64_t                  mask       = vr4300_addsub_mask(iw);
    unsigned                  dest;
    uint64_t                  rd;
    dest               = ((iw) >> 11 & 0x1F);
    rt                 = (rt ^ mask) - mask;
    rd                 = rs + rt;
    exdc_latch->result = (int32_t)rd;
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_ADDI_SUBI(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint64_t                  mask       = 0;
    unsigned                  dest;
    dest               = ((iw) >> 16 & 0x1F);
    rt                 = (int16_t)iw;
    rt                 = (rt ^ mask) - mask;
    rt                 = rs + rt;
    exdc_latch->result = (int32_t)rt;
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_ADDIU_LUI_SUBIU(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  immshift   = iw >> 24 & 0x10;
    unsigned                  dest;
    dest               = ((iw) >> 16 & 0x1F);
    rt                 = (int16_t)iw;
    rt                 = rs + (rt << immshift);
    exdc_latch->result = (int32_t)rt;
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_ADDU_SUBU(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint64_t                  mask       = vr4300_addsub_mask(iw);
    unsigned                  dest;
    uint64_t                  rd;
    dest               = ((iw) >> 11 & 0x1F);
    rt                 = (rt ^ mask) - mask;
    rd                 = rs + rt;
    exdc_latch->result = (int32_t)rd;
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_AND_OR_XOR(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest;
    uint64_t                  rd, rand, rxor;
    dest = ((iw) >> 11 & 0x1F);
    rand = rs & rt;
    rxor = rs ^ rt;
    rd   = rand + rxor;
    if ((iw & 1) == 0)
        rd = rxor;
    if ((iw & 3) == 0)
        rd = rand;
    exdc_latch->result = rd;
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_ANDI_ORI_XORI(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest;
    uint64_t                  rd, rand, rxor;
    dest = ((iw) >> 16 & 0x1F);
    rt   = (uint16_t)iw;
    rand = rs & rt;
    rxor = rs ^ rt;
    rd   = rand + rxor;
    if ((iw & 67108864) == 0)
        rd = rxor;
    if ((iw & 201326592) == 0)
        rd = rand;
    exdc_latch->result = rd;
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_BEQ_BEQL_BNE_BNEL(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_icrf_latch *icrf_latch = &g_vr4300->pipeline.icrf_latch;
    struct vr4300_rfex_latch *rfex_latch = &g_vr4300->pipeline.rfex_latch;
    uint32_t                  mask       = vr4300_branch_mask(iw, 30);
    uint64_t                  offset     = (uint64_t)((int16_t)iw) << 2;
    bool                      is_ne      = iw >> 26 & 0x1;
    bool                      cmp        = rs == rt;
    if (cmp == is_ne) {
        rfex_latch->iw_mask = mask;
        return 0;
    }
    icrf_latch->pc = rfex_latch->common.pc + (offset + 4);
    return 0;
}
int vr4300::VR4300_BGEZ_BGEZL_BLTZ_BLTZL(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_icrf_latch *icrf_latch = &g_vr4300->pipeline.icrf_latch;
    struct vr4300_rfex_latch *rfex_latch = &g_vr4300->pipeline.rfex_latch;
    uint32_t                  mask       = vr4300_branch_mask(iw, 17);
    uint64_t                  offset     = (uint64_t)((int16_t)iw) << 2;
    bool                      is_ge      = iw >> 16 & 0x1;
    bool                      cmp        = (int64_t)rs < 0;
    if (cmp == is_ge) {
        rfex_latch->iw_mask = mask;
        return 0;
    }
    icrf_latch->pc = rfex_latch->common.pc + (offset + 4);
    return 0;
}
int vr4300::VR4300_BGEZAL_BGEZALL_BLTZAL_BLTZALL(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_icrf_latch *icrf_latch = &g_vr4300->pipeline.icrf_latch;
    struct vr4300_rfex_latch *rfex_latch = &g_vr4300->pipeline.rfex_latch;
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint32_t                  mask       = vr4300_branch_mask(iw, 17);
    uint64_t                  offset     = (uint64_t)((int16_t)iw) << 2;
    bool                      is_ge      = iw >> 16 & 0x1;
    bool                      cmp        = (int64_t)rs < 0;
    exdc_latch->result                   = rfex_latch->common.pc + 8;
    exdc_latch->dest                     = VR4300_REGISTER_RA;
    if (cmp == is_ge) {
        rfex_latch->iw_mask = mask;
        return 0;
    }
    icrf_latch->pc = rfex_latch->common.pc + (offset + 4);
    return 0;
}
int vr4300::VR4300_BGTZ_BGTZL_BLEZ_BLEZL(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_icrf_latch *icrf_latch = &g_vr4300->pipeline.icrf_latch;
    struct vr4300_rfex_latch *rfex_latch = &g_vr4300->pipeline.rfex_latch;
    uint32_t                  mask       = vr4300_branch_mask(iw, 30);
    uint64_t                  offset     = (uint64_t)((int16_t)iw) << 2;
    bool                      is_gt      = iw >> 26 & 0x1;
    bool                      cmp        = (int64_t)rs <= 0;
    if (cmp == is_gt) {
        rfex_latch->iw_mask = mask;
        return 0;
    }
    icrf_latch->pc = rfex_latch->common.pc + (offset + 4);
    return 0;
}
int vr4300::VR4300_BREAK(uint32_t iw, uint64_t rs, uint64_t rt)
{
    VR4300_BRPT();
    return 1;
}
int vr4300::VR4300_TEQ_TNE(uint32_t iw, uint64_t rs, uint64_t rt)
{
    bool is_ne = iw >> 1 & 0x1;
    bool cmp   = rs == rt;
    if (cmp != is_ne) {
        VR4300_TRAP();
    }
    return 0;
}
int vr4300::VR4300_TEQI_TNEI(uint32_t iw, uint64_t rs, uint64_t rt)
{
    int64_t imm   = (int16_t)iw;
    bool    is_ne = iw >> 17 & 0x1;
    bool    cmp   = rs == (uint64_t)imm;
    if (cmp != is_ne) {
        VR4300_TRAP();
    }
    return 0;
}
int vr4300::VR4300_TGE_TLT(uint32_t iw, uint64_t rs, uint64_t rt)
{
    bool is_lt = iw >> 1 & 0x1;
    bool cmp   = (int64_t)rs >= (int64_t)rt;
    if (cmp != is_lt) {
        VR4300_TRAP();
    }
    return 0;
}
int vr4300::VR4300_TGEI_TLTI(uint32_t iw, uint64_t rs, uint64_t rt)
{
    int64_t imm   = (int16_t)iw;
    bool    is_lt = iw >> 17 & 0x1;
    bool    cmp   = (int64_t)rs >= imm;
    if (cmp != is_lt) {
        VR4300_TRAP();
    }
    return 0;
}
int vr4300::VR4300_TGEIU_TLTIU(uint32_t iw, uint64_t rs, uint64_t rt)
{
    int64_t imm   = (int16_t)iw;
    bool    is_lt = iw >> 17 & 0x1;
    bool    cmp   = rs >= (uint64_t)imm;
    if (cmp != is_lt) {
        VR4300_TRAP();
    }
    return 0;
}
int vr4300::VR4300_TGEU_TLTU(uint32_t iw, uint64_t rs, uint64_t rt)
{
    bool is_lt = iw >> 1 & 0x1;
    bool cmp   = rs >= rt;
    if (cmp != is_lt) {
        VR4300_TRAP();
    }
    return 0;
}
int vr4300::VR4300_DADD_DSUB(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint64_t                  mask       = vr4300_addsub_mask(iw);
    unsigned                  dest;
    uint64_t                  rd;
    dest               = ((iw) >> 11 & 0x1F);
    rt                 = (rt ^ mask) - mask;
    rd                 = rs + rt;
    exdc_latch->result = (int64_t)rd;
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_DADDI_DSUBI(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint64_t                  mask       = 0;
    unsigned                  dest;
    dest               = ((iw) >> 16 & 0x1F);
    rt                 = (int16_t)iw;
    rt                 = (rt ^ mask) - mask;
    rt                 = rs + rt;
    exdc_latch->result = (int64_t)rt;
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_DADDIU_DSUBIU(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint64_t                  mask       = 0;
    unsigned                  dest;
    dest               = ((iw) >> 16 & 0x1F);
    rt                 = (int16_t)iw;
    rt                 = (rt ^ mask) - mask;
    rt                 = rs + rt;
    exdc_latch->result = (int64_t)rt;
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_DADDU_DSUBU(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint64_t                  mask       = vr4300_addsub_mask(iw);
    unsigned                  dest;
    uint64_t                  rd;
    dest               = ((iw) >> 11 & 0x1F);
    rt                 = (rt ^ mask) - mask;
    rd                 = rs + rt;
    exdc_latch->result = (int64_t)rd;
    exdc_latch->dest   = dest;
    return 0;
}
int vr4300::VR4300_DIV_DIVU(uint32_t iw, uint64_t rs, uint64_t rt)
{
    bool     is_divu  = iw & 0x1;
    uint64_t sex_mask = vr4300_mult_sex_mask[is_divu];
    int64_t  rs_sex   = (int32_t)rs & sex_mask;
    int64_t  rt_sex   = (int32_t)rt & sex_mask;
    int32_t  div;
    int32_t  mod;
    if (__builtin_expect((rt_sex != 0), !0)) {
        div = rs_sex / rt_sex;
        mod = rs_sex % rt_sex;
    } else {
        div = rs_sex < 0 ? 1 : -1;
        mod = rs_sex;
    }
    g_vr4300->regs[VR4300_REGISTER_LO] = div;
    g_vr4300->regs[VR4300_REGISTER_HI] = mod;
    return vr4300_do_mci(37);
}
int vr4300::VR4300_DDIV(uint32_t iw, uint64_t _rs, uint64_t _rt)
{
    int64_t  rs = _rs;
    int64_t  rt = _rt;
    uint64_t div;
    uint64_t mod;
    if (__builtin_expect((rs == (-9223372036854775807L - 1) && rt == -1), 0)) {
        div = (-9223372036854775807L - 1);
        mod = 0;
    } else if (__builtin_expect((rt != 0), !0)) {
        div = rs / rt;
        mod = rs % rt;
    } else {
        div = rs < 0 ? 1 : -1;
        mod = rs;
    }
    g_vr4300->regs[VR4300_REGISTER_LO] = div;
    g_vr4300->regs[VR4300_REGISTER_HI] = mod;
    return vr4300_do_mci(69);
}
int vr4300::VR4300_DDIVU(uint32_t iw, uint64_t rs, uint64_t rt)
{
    uint64_t div;
    uint64_t mod;
    if (__builtin_expect((rt != 0), !0)) {
        div = rs / rt;
        mod = rs % rt;
    } else {
        div = -1;
        mod = rs;
    }
    g_vr4300->regs[VR4300_REGISTER_LO] = div;
    g_vr4300->regs[VR4300_REGISTER_HI] = mod;
    return vr4300_do_mci(69);
}
int vr4300::VR4300_DMULT(uint32_t iw, uint64_t rs, uint64_t rt)
{
    int64_t    lo, hi;
    __int128_t rsx = (int64_t)rs;
    __int128_t rtx = (int64_t)rt;
    __int128_t result;
    result                             = rsx * rtx;
    lo                                 = result;
    hi                                 = result >> 64;
    g_vr4300->regs[VR4300_REGISTER_LO] = lo;
    g_vr4300->regs[VR4300_REGISTER_HI] = hi;
    return vr4300_do_mci(8);
}
int vr4300::VR4300_DMULTU(uint32_t iw, uint64_t rs, uint64_t rt)
{
    uint64_t    lo, hi;
    __uint128_t rsx = rs;
    __uint128_t rtx = rt;
    __uint128_t result;
    result                             = rsx * rtx;
    lo                                 = result;
    hi                                 = result >> 64;
    g_vr4300->regs[VR4300_REGISTER_LO] = lo;
    g_vr4300->regs[VR4300_REGISTER_HI] = hi;
    return vr4300_do_mci(8);
}
int vr4300::VR4300_DSLL_DSLL32(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    unsigned                  sa         = (iw >> 6 & 0x1F) + ((iw & 0x4) << 3);
    exdc_latch->result                   = rt << sa;
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_DSLLV(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    unsigned                  sa         = rs & 0x3F;
    exdc_latch->result                   = rt << sa;
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_DSRA_DSRA32(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    unsigned                  sa         = (iw >> 6 & 0x1F) + ((iw & 0x4) << 3);
    exdc_latch->result                   = (int64_t)rt >> sa;
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_DSRAV(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    unsigned                  sa         = rs & 0x3F;
    exdc_latch->result                   = (int64_t)rt >> sa;
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_DSRL_DSRL32(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    unsigned                  sa         = (iw >> 6 & 0x1F) + ((iw & 0x4) << 3);
    exdc_latch->result                   = rt >> sa;
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_DSRLV(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    unsigned                  sa         = rs & 0x3F;
    exdc_latch->result                   = rt >> sa;
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_INVALID(uint32_t iw, uint64_t rs, uint64_t rt)
{
    VR4300_RI();
    return 0;
}
int vr4300::VR4300_J_JAL(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_icrf_latch *icrf_latch = &g_vr4300->pipeline.icrf_latch;
    struct vr4300_rfex_latch *rfex_latch = &g_vr4300->pipeline.rfex_latch;
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint32_t                  target     = iw << 2 & 0x0FFFFFFF;
    uint32_t                  mask       = vr4300_branch_mask(iw, 26);
    exdc_latch->result                   = rfex_latch->common.pc + 8;
    exdc_latch->dest                     = VR4300_REGISTER_RA & ~mask;
    icrf_latch->pc                       = (rfex_latch->common.pc & ~0x0FFFFFFFULL) | target;
    return 0;
}
int vr4300::VR4300_JALR_JR(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_icrf_latch *icrf_latch = &g_vr4300->pipeline.icrf_latch;
    struct vr4300_rfex_latch *rfex_latch = &g_vr4300->pipeline.rfex_latch;
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint32_t                  mask       = vr4300_branch_mask(iw, 0);
    uint32_t                  rd         = ((iw) >> 11 & 0x1F);
    exdc_latch->result                   = rfex_latch->common.pc + 8;
    exdc_latch->dest                     = rd & ~mask;
    icrf_latch->pc                       = rs;
    return 0;
}
int vr4300::VR4300_LD_SD(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint64_t                  sel_mask   = (int64_t)(int32_t)(iw << 2) >> 32;
    exdc_latch->request.vaddr            = rs + (int64_t)(int16_t)iw;
    exdc_latch->request.data             = ~sel_mask | (sel_mask & rt);
    exdc_latch->request.wdqm             = sel_mask;
    exdc_latch->request.postshift        = 0;
    exdc_latch->request.access_type      = VR4300_ACCESS_DWORD;
    exdc_latch->request.type             = (vr4300_bus_request_type)(1 - sel_mask);
    exdc_latch->request.size             = 8;
    exdc_latch->dest                     = ~sel_mask & ((iw) >> 16 & 0x1F);
    exdc_latch->result                   = 0;
    return 0;
}
int vr4300::VR4300_LOAD_STORE(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch    = &g_vr4300->pipeline.exdc_latch;
    uint64_t                  sel_mask      = (int64_t)(int32_t)(iw << 2) >> 32;
    uint64_t                  address       = rs + (int64_t)(int16_t)iw;
    unsigned                  request_index = (iw >> 26 & 0x7);
    uint64_t                  dqm           = vr4300_load_sex_mask[request_index] & ~sel_mask;
    unsigned                  request_size  = request_index & 0x3;
    unsigned                  lshiftamt     = (3 - request_size) << 3;
    unsigned                  rshiftamt     = (address & 0x3) << 3;
    exdc_latch->request.vaddr               = address;
    exdc_latch->request.data                = dqm | (sel_mask & ((rt << lshiftamt) >> rshiftamt));
    exdc_latch->request.wdqm                = ((uint32_t)sel_mask << lshiftamt) >> rshiftamt;
    exdc_latch->request.postshift           = 0;
    exdc_latch->request.access_type         = VR4300_ACCESS_WORD;
    exdc_latch->request.type                = (vr4300_bus_request_type)(1 - sel_mask);
    exdc_latch->request.size                = request_size + 1;
    exdc_latch->dest                        = ~sel_mask & ((iw) >> 16 & 0x1F);
    exdc_latch->result                      = 0;
    return 0;
}
int vr4300::VR4300_LDL_LDR(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint64_t                  address    = rs + (int64_t)(int16_t)iw;
    unsigned                  offset     = address & 0x7;
    unsigned                  dest       = ((iw) >> 16 & 0x1F);
    uint64_t                  dqm;
    int                       size;
    if (iw >> 26 & 0x1) {
        address ^= offset;
        size = offset + 1;
        dqm  = ~0ULL >> ((8 - size) << 3);
    } else {
        size = 8;
        dqm  = ~0ULL << (offset << 3);
    }
    exdc_latch->request.vaddr       = address;
    exdc_latch->request.data        = dqm;
    exdc_latch->request.wdqm        = 0ULL;
    exdc_latch->request.postshift   = 0;
    exdc_latch->request.access_type = VR4300_ACCESS_DWORD;
    exdc_latch->request.type        = VR4300_BUS_REQUEST_READ;
    exdc_latch->request.size        = size;
    exdc_latch->dest                = dest;
    exdc_latch->result              = rt & ~dqm;
    return 0;
}
int vr4300::VR4300_LWL_LWR(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint64_t                  address    = rs + (int64_t)(int16_t)iw;
    unsigned                  offset     = address & 0x3;
    unsigned                  dest       = ((iw) >> 16 & 0x1F);
    uint64_t                  dqm;
    int                       size;
    if (iw >> 28 & 0x1) {
        size = offset + 1;
        dqm  = ~0U >> ((4 - size) << 3);
        address ^= offset;
        dqm |= VR4300_LWR_forceset(offset);
    } else {
        dqm  = ~0ULL << (offset << 3);
        size = 4;
    }
    exdc_latch->request.vaddr       = address;
    exdc_latch->request.data        = dqm;
    exdc_latch->request.wdqm        = 0ULL;
    exdc_latch->request.postshift   = 0;
    exdc_latch->request.access_type = VR4300_ACCESS_WORD;
    exdc_latch->request.type        = VR4300_BUS_REQUEST_READ;
    exdc_latch->request.size        = size;
    exdc_latch->dest                = dest;
    exdc_latch->result              = rt & ~dqm;
    return 0;
}
int vr4300::VR4300_MFHI_MFLO(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    bool                      is_mflo    = iw >> 1 & 0x1;
    exdc_latch->result                   = g_vr4300->regs[VR4300_REGISTER_HI + is_mflo];
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_MTHI_MTLO(uint32_t iw, uint64_t rs, uint64_t rt)
{
    bool is_mtlo                                 = iw >> 1 & 0x1;
    g_vr4300->regs[VR4300_REGISTER_HI + is_mtlo] = rs;
    return 0;
}
int vr4300::VR4300_MULT_MULTU(uint32_t iw, uint64_t rs, uint64_t rt)
{
    bool     is_multu                  = iw & 0x1;
    uint64_t sex_mask                  = vr4300_mult_sex_mask[is_multu];
    uint64_t rs_sex                    = (int32_t)rs & sex_mask;
    uint64_t rt_sex                    = (int32_t)rt & sex_mask;
    uint64_t result                    = rs_sex * rt_sex;
    g_vr4300->regs[VR4300_REGISTER_LO] = (int32_t)result;
    g_vr4300->regs[VR4300_REGISTER_HI] = (int32_t)(result >> 32);
    return 0;
}
int vr4300::VR4300_NOR(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    exdc_latch->result                   = ~(rs | rt);
    exdc_latch->dest                     = dest;
    return vr4300_do_mci(57);
}
int vr4300::VR4300_SLL_SLLV(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    unsigned                  sa         = (rs & 0x1F) + (iw >> 6 & 0x1F);
    exdc_latch->result                   = (int32_t)(rt << sa);
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_SLT(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    exdc_latch->result                   = (int64_t)rs < (int64_t)rt;
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_SLTI(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 16 & 0x1F);
    int64_t                   imm        = (int16_t)iw;
    exdc_latch->result                   = (int64_t)rs < imm;
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_SLTIU(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 16 & 0x1F);
    uint64_t                  imm        = (int16_t)iw;
    exdc_latch->result                   = rs < imm;
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_SLTU(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    exdc_latch->result                   = rs < rt;
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_SRA(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    unsigned                  sa         = iw >> 6 & 0x1F;
    exdc_latch->result                   = (int32_t)(rt >> sa);
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_SRAV(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    unsigned                  sa         = rs & 0x1F;
    exdc_latch->result                   = (int32_t)(rt >> sa);
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_SRL(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    unsigned                  sa         = iw >> 6 & 0x1F;
    exdc_latch->result                   = (int32_t)((uint32_t)rt >> sa);
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_SRLV(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    unsigned                  dest       = ((iw) >> 11 & 0x1F);
    unsigned                  sa         = rs & 0x1F;
    exdc_latch->result                   = (int32_t)((uint32_t)rt >> sa);
    exdc_latch->dest                     = dest;
    return 0;
}
int vr4300::VR4300_SDL_SDR(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint64_t                  address    = rs + (int64_t)(int16_t)iw;
    unsigned                  offset     = address & 0x7;
    uint64_t                  mask       = ~0ULL;
    unsigned                  shiftamt;
    uint64_t                  data;
    uint64_t                  dqm;
    if (iw >> 26 & 0x1) {
        shiftamt = (7 - offset) << 3;
        data     = rt << shiftamt;
        dqm      = mask << shiftamt;
    } else {
        shiftamt = offset << 3;
        data     = rt >> shiftamt;
        dqm      = mask >> shiftamt;
    }
    exdc_latch->request.vaddr       = address;
    exdc_latch->request.data        = data;
    exdc_latch->request.wdqm        = dqm;
    exdc_latch->request.access_type = VR4300_ACCESS_DWORD;
    exdc_latch->request.type        = VR4300_BUS_REQUEST_WRITE;
    exdc_latch->request.size        = 8;
    return 0;
}
int vr4300::VR4300_SWL_SWR(uint32_t iw, uint64_t rs, uint64_t rt)
{
    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint64_t                  address    = rs + (int64_t)(int16_t)iw;
    unsigned                  offset     = address & 0x3;
    uint32_t                  mask       = ~0U;
    unsigned                  shiftamt;
    uint64_t                  data;
    uint64_t                  dqm;
    if (iw >> 28 & 0x1) {
        shiftamt = (3 - offset) << 3;
        data     = rt << shiftamt;
        dqm      = mask << shiftamt;
    } else {
        shiftamt = offset << 3;
        data     = rt >> shiftamt;
        dqm      = mask >> shiftamt;
    }
    exdc_latch->request.vaddr       = address;
    exdc_latch->request.data        = data;
    exdc_latch->request.wdqm        = dqm;
    exdc_latch->request.access_type = VR4300_ACCESS_WORD;
    exdc_latch->request.type        = VR4300_BUS_REQUEST_WRITE;
    exdc_latch->request.size        = 4;
    return 0;
}
int vr4300::VR4300_SYSCALL(uint32_t iw, uint64_t rs, uint64_t rt)
{
    VR4300_SYSC();
    return 1;
}
uint32_t vr4300::fpu_get_state(void)
{
    return _mm_getcsr();
}
void vr4300::fpu_set_state(uint32_t state)
{
    _mm_setcsr(state);
}
void vr4300::fpu_abs_32(const uint32_t *fs, uint32_t *fd)
{
    float  fs_float, fd_float;
    __m128 fs_reg, fd_reg;
    memcpy(&fs_float, fs, sizeof(fs_float));
    fs_reg   = _mm_set_ss(fs_float);
    fd_reg   = _mm_andnot_ps(_mm_set_ss(-0.0f), fs_reg);
    fd_float = _mm_cvtss_f32(fd_reg);
    memcpy(fd, &fd_float, sizeof(fd_float));
}
void vr4300::fpu_abs_64(const uint64_t *fs, uint64_t *fd)
{
    double  fs_double, fd_double;
    __m128d fs_reg, fd_reg;
    memcpy(&fs_double, fs, sizeof(fs_double));
    fs_reg    = _mm_set_sd(fs_double);
    fd_reg    = _mm_andnot_pd(_mm_set_sd(-0.0), fs_reg);
    fd_double = _mm_cvtsd_f64(fd_reg);
    memcpy(fd, &fd_double, sizeof(fd_double));
}
void vr4300::fpu_add_32(const uint32_t *fs, const uint32_t *ft, uint32_t *fd)
{
    float  fs_float, ft_float, fd_float;
    __m128 fs_reg, ft_reg, fd_reg;
    memcpy(&fs_float, fs, sizeof(fs_float));
    memcpy(&ft_float, ft, sizeof(ft_float));
    fs_reg   = _mm_set_ss(fs_float);
    ft_reg   = _mm_set_ss(ft_float);
    fd_reg   = _mm_add_ss(fs_reg, ft_reg);
    fd_float = _mm_cvtss_f32(fd_reg);
    memcpy(fd, &fd_float, sizeof(fd_float));
}
void vr4300::fpu_add_64(const uint64_t *fs, const uint64_t *ft, uint64_t *fd)
{
    double  fs_double, ft_double, fd_double;
    __m128d fs_reg, ft_reg, fd_reg;
    memcpy(&fs_double, fs, sizeof(fs_double));
    memcpy(&ft_double, ft, sizeof(ft_double));
    fs_reg    = _mm_set_sd(fs_double);
    ft_reg    = _mm_set_sd(ft_double);
    fd_reg    = _mm_add_sd(fs_reg, ft_reg);
    fd_double = _mm_cvtsd_f64(fd_reg);
    memcpy(fd, &fd_double, sizeof(fd_double));
}
uint8_t vr4300::fpu_cmp_ueq_32(const uint32_t *fs, const uint32_t *ft)
{
    float  fs_float, ft_float;
    __m128 fs_reg, ft_reg;
    memcpy(&fs_float, fs, sizeof(fs_float));
    memcpy(&ft_float, ft, sizeof(ft_float));
    fs_reg = _mm_set_ss(fs_float);
    ft_reg = _mm_set_ss(ft_float);
    return _mm_comieq_ss(fs_reg, ft_reg);
}
uint8_t vr4300::fpu_cmp_ueq_64(const uint64_t *fs, const uint64_t *ft)
{
    double  fs_double, ft_double;
    __m128d fs_reg, ft_reg;
    memcpy(&fs_double, fs, sizeof(fs_double));
    memcpy(&ft_double, ft, sizeof(ft_double));
    fs_reg = _mm_set_sd(fs_double);
    ft_reg = _mm_set_sd(ft_double);
    return _mm_comieq_sd(fs_reg, ft_reg);
}
uint8_t vr4300::fpu_cmp_ule_32(const uint32_t *fs, const uint32_t *ft)
{
    float  fs_float, ft_float;
    __m128 fs_reg, ft_reg;
    memcpy(&fs_float, fs, sizeof(fs_float));
    memcpy(&ft_float, ft, sizeof(ft_float));
    fs_reg = _mm_set_ss(fs_float);
    ft_reg = _mm_set_ss(ft_float);
    return _mm_comile_ss(fs_reg, ft_reg);
}
uint8_t vr4300::fpu_cmp_ule_64(const uint64_t *fs, const uint64_t *ft)
{
    double  fs_double, ft_double;
    __m128d fs_reg, ft_reg;
    memcpy(&fs_double, fs, sizeof(fs_double));
    memcpy(&ft_double, ft, sizeof(ft_double));
    fs_reg = _mm_set_sd(fs_double);
    ft_reg = _mm_set_sd(ft_double);
    return _mm_comile_sd(fs_reg, ft_reg);
}
uint8_t vr4300::fpu_cmp_ult_32(const uint32_t *fs, const uint32_t *ft)
{
    float  fs_float, ft_float;
    __m128 fs_reg, ft_reg;
    memcpy(&fs_float, fs, sizeof(fs_float));
    memcpy(&ft_float, ft, sizeof(ft_float));
    fs_reg = _mm_set_ss(fs_float);
    ft_reg = _mm_set_ss(ft_float);
    return _mm_comilt_ss(fs_reg, ft_reg);
}
uint8_t vr4300::fpu_cmp_ult_64(const uint64_t *fs, const uint64_t *ft)
{
    double  fs_double, ft_double;
    __m128d fs_reg, ft_reg;
    memcpy(&fs_double, fs, sizeof(fs_double));
    memcpy(&ft_double, ft, sizeof(ft_double));
    fs_reg = _mm_set_sd(fs_double);
    ft_reg = _mm_set_sd(ft_double);
    return _mm_comilt_sd(fs_reg, ft_reg);
}
void vr4300::fpu_cvt_f32_f64(const uint64_t *fs, uint32_t *fd)
{
    double  fs_double;
    float   fd_float;
    __m128d fs_reg;
    __m128  fd_reg;
    memcpy(&fs_double, fs, sizeof(fs_double));
    fd_reg   = _mm_setzero_ps();
    fs_reg   = _mm_set_sd(fs_double);
    fd_reg   = _mm_cvtsd_ss(fd_reg, fs_reg);
    fd_float = _mm_cvtss_f32(fd_reg);
    memcpy(fd, &fd_float, sizeof(fd_float));
}
void vr4300::fpu_cvt_f32_i32(const uint32_t *fs, uint32_t *fd)
{
    float  fd_float;
    __m128 fd_reg;
    fd_reg   = _mm_setzero_ps();
    fd_reg   = _mm_cvtsi32_ss(fd_reg, *fs);
    fd_float = _mm_cvtss_f32(fd_reg);
    memcpy(fd, &fd_float, sizeof(fd_float));
}
void vr4300::fpu_cvt_f32_i64(const uint64_t *fs, uint32_t *fd)
{
    float  fd_float;
    __m128 fd_reg;
    fd_reg   = _mm_setzero_ps();
    fd_reg   = _mm_cvtsi64_ss(fd_reg, *fs);
    fd_float = _mm_cvtss_f32(fd_reg);
    memcpy(fd, &fd_float, sizeof(fd_float));
}
void vr4300::fpu_cvt_f64_f32(const uint32_t *fs, uint64_t *fd)
{
    double  fd_double;
    float   fs_float;
    __m128d fd_reg;
    __m128  fs_reg;
    memcpy(&fs_float, fs, sizeof(fs_float));
    fd_reg    = _mm_setzero_pd();
    fs_reg    = _mm_set_ss(fs_float);
    fd_reg    = _mm_cvtss_sd(fd_reg, fs_reg);
    fd_double = _mm_cvtsd_f64(fd_reg);
    memcpy(fd, &fd_double, sizeof(fd_double));
}
void vr4300::fpu_cvt_f64_i32(const uint32_t *fs, uint64_t *fd)
{
    double  fd_double;
    __m128d fd_reg;
    fd_reg    = _mm_setzero_pd();
    fd_reg    = _mm_cvtsi32_sd(fd_reg, *fs);
    fd_double = _mm_cvtsd_f64(fd_reg);
    memcpy(fd, &fd_double, sizeof(fd_double));
}
void vr4300::fpu_cvt_f64_i64(const uint64_t *fs, uint64_t *fd)
{
    double  fd_double;
    __m128d fd_reg;
    fd_reg    = _mm_setzero_pd();
    fd_reg    = _mm_cvtsi64_sd(fd_reg, *fs);
    fd_double = _mm_cvtsd_f64(fd_reg);
    memcpy(fd, &fd_double, sizeof(fd_double));
}
void vr4300::fpu_cvt_i32_f32(const uint32_t *fs, uint32_t *fd)
{
    float  fs_float;
    __m128 fs_reg;
    memcpy(&fs_float, fs, sizeof(fs_float));
    fs_reg = _mm_set_ss(fs_float);
    *fd    = _mm_cvtss_si32(fs_reg);
}
void vr4300::fpu_cvt_i32_f64(const uint64_t *fs, uint32_t *fd)
{
    double  fs_double;
    __m128d fs_reg;
    memcpy(&fs_double, fs, sizeof(fs_double));
    fs_reg = _mm_set_sd(fs_double);
    *fd    = _mm_cvtsd_si32(fs_reg);
}
void vr4300::fpu_cvt_i64_f32(const uint32_t *fs, uint64_t *fd)
{
    float  fs_float;
    __m128 fs_reg;
    memcpy(&fs_float, fs, sizeof(fs_float));
    fs_reg = _mm_set_ss(fs_float);
    *fd    = _mm_cvtss_si64(fs_reg);
}
void vr4300::fpu_cvt_i64_f64(const uint64_t *fs, uint64_t *fd)
{
    double  fs_double;
    __m128d fs_reg;
    memcpy(&fs_double, fs, sizeof(fs_double));
    fs_reg = _mm_set_sd(fs_double);
    *fd    = _mm_cvtsd_si64(fs_reg);
}
void vr4300::fpu_div_32(const uint32_t *fs, const uint32_t *ft, uint32_t *fd)
{
    float  fs_float, ft_float, fd_float;
    __m128 fs_reg, ft_reg, fd_reg;
    memcpy(&fs_float, fs, sizeof(fs_float));
    memcpy(&ft_float, ft, sizeof(ft_float));
    fs_reg   = _mm_set_ss(fs_float);
    ft_reg   = _mm_set_ss(ft_float);
    fd_reg   = _mm_div_ss(fs_reg, ft_reg);
    fd_float = _mm_cvtss_f32(fd_reg);
    memcpy(fd, &fd_float, sizeof(fd_float));
}
void vr4300::fpu_div_64(const uint64_t *fs, const uint64_t *ft, uint64_t *fd)
{
    double  fs_double, ft_double, fd_double;
    __m128d fs_reg, ft_reg, fd_reg;
    memcpy(&fs_double, fs, sizeof(fs_double));
    memcpy(&ft_double, ft, sizeof(ft_double));
    fs_reg    = _mm_set_sd(fs_double);
    ft_reg    = _mm_set_sd(ft_double);
    fd_reg    = _mm_div_sd(fs_reg, ft_reg);
    fd_double = _mm_cvtsd_f64(fd_reg);
    memcpy(fd, &fd_double, sizeof(fd_double));
}
uint8_t vr4300::fpu_cmp_eq_32(const uint32_t *fs, const uint32_t *ft)
{
    float   fs_float, ft_float;
    __m128  fs_reg, ft_reg;
    uint8_t condition;
    memcpy(&fs_float, fs, sizeof(fs_float));
    memcpy(&ft_float, ft, sizeof(ft_float));
    fs_reg = _mm_set_ss(fs_float);
    ft_reg = _mm_set_ss(ft_float);
    __asm__ __volatile__("comiss %1, %2\n\t"
                         "sete %%dl\n\t"
                         "setnp %%al\n\t"
                         "and %%dl, %%al\n\t"
                         : "=a"(condition)
                         : "x"(fs_reg), "x"(ft_reg)
                         : "dl", "cc");
    return condition;
}
uint8_t vr4300::fpu_cmp_eq_64(const uint64_t *fs, const uint64_t *ft)
{
    double  fs_double, ft_double;
    __m128d fs_reg, ft_reg;
    uint8_t condition;
    memcpy(&fs_double, fs, sizeof(fs_double));
    memcpy(&ft_double, ft, sizeof(ft_double));
    fs_reg = _mm_set_sd(fs_double);
    ft_reg = _mm_set_sd(ft_double);
    __asm__ __volatile__("comisd %1, %2\n\t"
                         "sete %%dl\n\t"
                         "setnp %%al\n\t"
                         "and %%dl, %%al\n\t"
                         : "=a"(condition)
                         : "x"(fs_reg), "x"(ft_reg)
                         : "dl", "cc");
    return condition;
}
uint8_t vr4300::fpu_cmp_f_32(const uint32_t *fs, const uint32_t *ft)
{
    float  fs_float, ft_float;
    __m128 fs_reg, ft_reg;
    memcpy(&fs_float, fs, sizeof(fs_float));
    memcpy(&ft_float, ft, sizeof(ft_float));
    fs_reg = _mm_set_ss(fs_float);
    ft_reg = _mm_set_ss(ft_float);
    __asm__ __volatile__("comiss %0, %1\n\t" ::"x"(fs_reg), "x"(ft_reg) : "cc");
    return 0;
}
uint8_t vr4300::fpu_cmp_f_64(const uint64_t *fs, const uint64_t *ft)
{
    double  fs_double, ft_double;
    __m128d fs_reg, ft_reg;
    memcpy(&fs_double, fs, sizeof(fs_double));
    memcpy(&ft_double, ft, sizeof(ft_double));
    fs_reg = _mm_set_sd(fs_double);
    ft_reg = _mm_set_sd(ft_double);
    __asm__ __volatile__("comisd %0, %1\n\t" ::"x"(fs_reg), "x"(ft_reg) : "cc");
    return 0;
}
uint8_t vr4300::fpu_cmp_ole_32(const uint32_t *fs, const uint32_t *ft)
{
    float   fs_float, ft_float;
    __m128  fs_reg, ft_reg;
    uint8_t condition;
    memcpy(&fs_float, fs, sizeof(fs_float));
    memcpy(&ft_float, ft, sizeof(ft_float));
    fs_reg = _mm_set_ss(fs_float);
    ft_reg = _mm_set_ss(ft_float);
    __asm__ __volatile__("comiss %1, %2\n\t"
                         "setae %%dl\n\t"
                         "setnp %%al\n\t"
                         "and %%dl, %%al\n\t"
                         : "=a"(condition)
                         : "x"(fs_reg), "x"(ft_reg)
                         : "dl", "cc");
    return condition;
}
uint8_t vr4300::fpu_cmp_ole_64(const uint64_t *fs, const uint64_t *ft)
{
    double  fs_double, ft_double;
    __m128d fs_reg, ft_reg;
    uint8_t condition;
    memcpy(&fs_double, fs, sizeof(fs_double));
    memcpy(&ft_double, ft, sizeof(ft_double));
    fs_reg = _mm_set_sd(fs_double);
    ft_reg = _mm_set_sd(ft_double);
    __asm__ __volatile__("comisd %1, %2\n\t"
                         "setae %%dl\n\t"
                         "setnp %%al\n\t"
                         "and %%dl, %%al\n\t"
                         : "=a"(condition)
                         : "x"(fs_reg), "x"(ft_reg)
                         : "dl", "cc");
    return condition;
}
uint8_t vr4300::fpu_cmp_olt_32(const uint32_t *fs, const uint32_t *ft)
{
    float   fs_float, ft_float;
    __m128  fs_reg, ft_reg;
    uint8_t condition;
    memcpy(&fs_float, fs, sizeof(fs_float));
    memcpy(&ft_float, ft, sizeof(ft_float));
    fs_reg = _mm_set_ss(fs_float);
    ft_reg = _mm_set_ss(ft_float);
    __asm__ __volatile__("comiss %1, %2\n\t"
                         "seta %%dl\n\t"
                         "setnp %%al\n\t"
                         "and %%dl, %%al\n\t"
                         : "=a"(condition)
                         : "x"(fs_reg), "x"(ft_reg)
                         : "dl", "cc");
    return condition;
}
uint8_t vr4300::fpu_cmp_olt_64(const uint64_t *fs, const uint64_t *ft)
{
    double  fs_double, ft_double;
    __m128d fs_reg, ft_reg;
    uint8_t condition;
    memcpy(&fs_double, fs, sizeof(fs_double));
    memcpy(&ft_double, ft, sizeof(ft_double));
    fs_reg = _mm_set_sd(fs_double);
    ft_reg = _mm_set_sd(ft_double);
    __asm__ __volatile__("comisd %1, %2\n\t"
                         "seta %%dl\n\t"
                         "setnp %%al\n\t"
                         "and %%dl, %%al\n\t"
                         : "=a"(condition)
                         : "x"(fs_reg), "x"(ft_reg)
                         : "dl", "cc");
    return condition;
}
uint8_t vr4300::fpu_cmp_un_32(const uint32_t *fs, const uint32_t *ft)
{
    float   fs_float, ft_float;
    __m128  fs_reg, ft_reg;
    uint8_t condition;
    memcpy(&fs_float, fs, sizeof(fs_float));
    memcpy(&ft_float, ft, sizeof(ft_float));
    fs_reg = _mm_set_ss(fs_float);
    ft_reg = _mm_set_ss(ft_float);
    __asm__ __volatile__("comiss %1, %2\n\t"
                         "setp %%al\n\t"
                         : "=a"(condition)
                         : "x"(fs_reg), "x"(ft_reg)
                         : "cc");
    return condition;
}
uint8_t vr4300::fpu_cmp_un_64(const uint64_t *fs, const uint64_t *ft)
{
    double  fs_double, ft_double;
    __m128d fs_reg, ft_reg;
    uint8_t condition;
    memcpy(&fs_double, fs, sizeof(fs_double));
    memcpy(&ft_double, ft, sizeof(ft_double));
    fs_reg = _mm_set_sd(fs_double);
    ft_reg = _mm_set_sd(ft_double);
    __asm__ __volatile__("comisd %1, %2\n\t"
                         "setp %%al\n\t"
                         : "=a"(condition)
                         : "x"(fs_reg), "x"(ft_reg)
                         : "cc");
    return condition;
}
void vr4300::fpu_mul_32(const uint32_t *fs, const uint32_t *ft, uint32_t *fd)
{
    float  fs_float, ft_float, fd_float;
    __m128 fs_reg, ft_reg, fd_reg;
    memcpy(&fs_float, fs, sizeof(fs_float));
    memcpy(&ft_float, ft, sizeof(ft_float));
    fs_reg   = _mm_set_ss(fs_float);
    ft_reg   = _mm_set_ss(ft_float);
    fd_reg   = _mm_mul_ss(fs_reg, ft_reg);
    fd_float = _mm_cvtss_f32(fd_reg);
    memcpy(fd, &fd_float, sizeof(fd_float));
}
void vr4300::fpu_mul_64(const uint64_t *fs, const uint64_t *ft, uint64_t *fd)
{
    double  fs_double, ft_double, fd_double;
    __m128d fs_reg, ft_reg, fd_reg;
    memcpy(&fs_double, fs, sizeof(fs_double));
    memcpy(&ft_double, ft, sizeof(ft_double));
    fs_reg    = _mm_set_sd(fs_double);
    ft_reg    = _mm_set_sd(ft_double);
    fd_reg    = _mm_mul_sd(fs_reg, ft_reg);
    fd_double = _mm_cvtsd_f64(fd_reg);
    memcpy(fd, &fd_double, sizeof(fd_double));
}
void vr4300::fpu_neg_32(const uint32_t *fs, uint32_t *fd)
{
    float  fs_float, fd_float;
    __m128 fs_reg, fd_reg;
    memcpy(&fs_float, fs, sizeof(fs_float));
    fs_reg   = _mm_set_ss(fs_float);
    fd_reg   = _mm_xor_ps(_mm_set_ss(-0.0f), fs_reg);
    fd_float = _mm_cvtss_f32(fd_reg);
    memcpy(fd, &fd_float, sizeof(fd_float));
}
void vr4300::fpu_neg_64(const uint64_t *fs, uint64_t *fd)
{
    double  fs_double, fd_double;
    __m128d fs_reg, fd_reg;
    memcpy(&fs_double, fs, sizeof(fs_double));
    fs_reg    = _mm_set_sd(fs_double);
    fd_reg    = _mm_xor_pd(_mm_set_sd(-0.0), fs_reg);
    fd_double = _mm_cvtsd_f64(fd_reg);
    memcpy(fd, &fd_double, sizeof(fd_double));
}
void vr4300::fpu_sqrt_32(const uint32_t *fs, uint32_t *fd)
{
    float  fs_float, fd_float;
    __m128 fs_reg, fd_reg;
    memcpy(&fs_float, fs, sizeof(fs_float));
    fs_reg   = _mm_set_ss(fs_float);
    fd_reg   = _mm_sqrt_ss(fs_reg);
    fd_float = _mm_cvtss_f32(fd_reg);
    memcpy(fd, &fd_float, sizeof(fd_float));
}
void vr4300::fpu_sqrt_64(const uint64_t *fs, uint64_t *fd)
{
    double  fs_double, fd_double;
    __m128d fs_reg, fd_reg;
    memcpy(&fs_double, fs, sizeof(fs_double));
    fs_reg    = _mm_set_sd(fs_double);
    fd_reg    = _mm_sqrt_sd(fs_reg, fs_reg);
    fd_double = _mm_cvtsd_f64(fd_reg);
    memcpy(fd, &fd_double, sizeof(fd_double));
}
void vr4300::fpu_sub_32(const uint32_t *fs, const uint32_t *ft, uint32_t *fd)
{
    float  fs_float, ft_float, fd_float;
    __m128 fs_reg, ft_reg, fd_reg;
    memcpy(&fs_float, fs, sizeof(fs_float));
    memcpy(&ft_float, ft, sizeof(ft_float));
    fs_reg   = _mm_set_ss(fs_float);
    ft_reg   = _mm_set_ss(ft_float);
    fd_reg   = _mm_sub_ss(fs_reg, ft_reg);
    fd_float = _mm_cvtss_f32(fd_reg);
    memcpy(fd, &fd_float, sizeof(fd_float));
}
void vr4300::fpu_sub_64(const uint64_t *fs, const uint64_t *ft, uint64_t *fd)
{
    double  fs_double, ft_double, fd_double;
    __m128d fs_reg, ft_reg, fd_reg;
    memcpy(&fs_double, fs, sizeof(fs_double));
    memcpy(&ft_double, ft, sizeof(ft_double));
    fs_reg    = _mm_set_sd(fs_double);
    ft_reg    = _mm_set_sd(ft_double);
    fd_reg    = _mm_sub_sd(fs_reg, ft_reg);
    fd_double = _mm_cvtsd_f64(fd_reg);
    memcpy(fd, &fd_double, sizeof(fd_double));
}
void vr4300::fpu_trunc_i32_f32(const uint32_t *fs, uint32_t *fd)
{
    float  fs_float;
    __m128 fs_reg;
    memcpy(&fs_float, fs, sizeof(fs_float));
    fs_reg = _mm_set_ss(fs_float);
    *fd    = _mm_cvttss_si32(fs_reg);
}
void vr4300::fpu_trunc_i32_f64(const uint64_t *fs, uint32_t *fd)
{
    double  fs_double;
    __m128d fs_reg;
    memcpy(&fs_double, fs, sizeof(fs_double));
    fs_reg = _mm_set_sd(fs_double);
    *fd    = _mm_cvttsd_si32(fs_reg);
}
void vr4300::fpu_trunc_i64_f32(const uint32_t *fs, uint64_t *fd)
{
    float  fs_float;
    __m128 fs_reg;
    memcpy(&fs_float, fs, sizeof(fs_float));
    fs_reg = _mm_set_ss(fs_float);
    *fd    = _mm_cvttss_si64(fs_reg);
}
void vr4300::fpu_trunc_i64_f64(const uint64_t *fs, uint64_t *fd)
{
    double  fs_double;
    __m128d fs_reg;
    memcpy(&fs_double, fs, sizeof(fs_double));
    fs_reg = _mm_set_sd(fs_double);
    *fd    = _mm_cvttsd_si64(fs_reg);
}

// cacheop

int vr4300_cacheop_unimplemented(uint64_t vaddr, uint32_t paddr)
{
    abort();
    return 0;
}
int vr4300_cacheop_ic_invalidate(uint64_t vaddr, uint32_t paddr)
{
    g_vr4300->vr4300_icache_invalidate(&g_vr4300->icache, vaddr);
    return 0;
}
int vr4300_cacheop_ic_set_taglo(uint64_t vaddr, uint32_t paddr)
{
    vr4300_icache_set_taglo(&g_vr4300->icache, vaddr, g_vr4300->regs[VR4300_CP0_REGISTER_TAGLO]);
    return 0;
}
int vr4300_cacheop_ic_invalidate_hit(uint64_t vaddr, uint32_t paddr)
{
    g_vr4300->vr4300_icache_invalidate_hit(&g_vr4300->icache, vaddr, paddr);
    return 0;
}
int vr4300_cacheop_dc_get_taglo(uint64_t vaddr, uint32_t paddr)
{
    g_vr4300->regs[VR4300_CP0_REGISTER_TAGLO] = (int32_t)g_vr4300->vr4300_dcache_get_taglo(&g_vr4300->dcache, vaddr);
    return 0;
}
int vr4300_cacheop_dc_set_taglo(uint64_t vaddr, uint32_t paddr)
{
    g_vr4300->vr4300_dcache_set_taglo(&g_vr4300->dcache, vaddr, g_vr4300->regs[VR4300_CP0_REGISTER_TAGLO]);
    return 0;
}
int vr4300_cacheop_dc_wb_invalidate(uint64_t vaddr, uint32_t paddr)
{
    struct vr4300_dcache_line *line;
    uint32_t                   bus_address;
    uint32_t                   data[4];
    unsigned                   i;
    if (!(line = g_vr4300->vr4300_dcache_wb_invalidate(&g_vr4300->dcache, vaddr)))
        return 0;
    if (line->metadata & 0x2) {
        bus_address = g_vr4300->vr4300_dcache_get_tag(line, vaddr);
        memcpy(data, line->data, sizeof(data));
        for (i = 0; i < 4; i++)
            bus_write_word(g_vr4300->bus, bus_address + i * 4, data[i ^ (4 >> 2)], ~0);
        line->metadata &= ~0x2;
        return (46 - 2);
    }
    return 0;
}
int vr4300_cacheop_dc_create_dirty_ex(uint64_t vaddr, uint32_t paddr)
{
    struct vr4300_dcache_line *line;
    uint32_t                   bus_address;
    uint32_t                   data[4];
    unsigned                   i;
    int                        delay = 0;
    if ((line = g_vr4300->vr4300_dcache_should_flush_line(&g_vr4300->dcache, vaddr))) {
        bus_address = g_vr4300->vr4300_dcache_get_tag(line, vaddr);
        memcpy(data, line->data, sizeof(data));
        for (i = 0; i < 4; i++)
            bus_write_word(g_vr4300->bus, bus_address + i * 4, data[i ^ (4 >> 2)], ~0);
        delay = (46 - 2);
    }
    g_vr4300->vr4300_dcache_create_dirty_exclusive(&g_vr4300->dcache, vaddr, paddr);
    return delay;
}
int vr4300_cacheop_dc_hit_invalidate(uint64_t vaddr, uint32_t paddr)
{
    struct vr4300_dcache_line *line;
    if ((line = g_vr4300->vr4300_dcache_probe(&g_vr4300->dcache, vaddr, paddr)))
        g_vr4300->vr4300_dcache_invalidate(line);
    return 0;
}
int vr4300_cacheop_dc_hit_wb_invalidate(uint64_t vaddr, uint32_t paddr)
{
    struct vr4300_dcache_line *line;
    uint32_t                   bus_address;
    uint32_t                   data[4];
    unsigned                   i;
    if (!(line = g_vr4300->vr4300_dcache_probe(&g_vr4300->dcache, vaddr, paddr)))
        return 0;
    if (line->metadata & 0x2) {
        bus_address = g_vr4300->vr4300_dcache_get_tag(line, vaddr);
        memcpy(data, line->data, sizeof(data));
        for (i = 0; i < 4; i++)
            bus_write_word(g_vr4300->bus, bus_address + i * 4, data[i ^ (4 >> 2)], ~0);
        line->metadata &= ~0x1;
        return (46 - 2);
    }
    line->metadata &= ~0x1;
    return 0;
}
int vr4300_cacheop_dc_hit_wb(uint64_t vaddr, uint32_t paddr)
{
    struct vr4300_dcache_line *line;
    uint32_t                   bus_address;
    uint32_t                   data[4];
    unsigned                   i;
    if (!(line = g_vr4300->vr4300_dcache_probe(&g_vr4300->dcache, vaddr, paddr)))
        return 0;
    if (line->metadata & 0x2) {
        bus_address = g_vr4300->vr4300_dcache_get_tag(line, vaddr);
        memcpy(data, line->data, sizeof(data));
        for (i = 0; i < 4; i++)
            bus_write_word(g_vr4300->bus, bus_address + i * 4, data[i ^ (4 >> 2)], ~0);
        line->metadata &= ~0x2;
        return (46 - 2);
    }
    return 0;
}
int vr4300::VR4300_CACHE(uint32_t iw, uint64_t rs, uint64_t rt)
{
    vr4300_cacheop_func_t cacheop_lut[32] = {
        vr4300_cacheop_ic_invalidate,     vr4300_cacheop_unimplemented,        vr4300_cacheop_ic_set_taglo,
        vr4300_cacheop_unimplemented,     vr4300_cacheop_ic_invalidate_hit,    vr4300_cacheop_unimplemented,
        vr4300_cacheop_unimplemented,     vr4300_cacheop_unimplemented,        vr4300_cacheop_dc_wb_invalidate,
        vr4300_cacheop_dc_get_taglo,      vr4300_cacheop_dc_set_taglo,         vr4300_cacheop_dc_create_dirty_ex,
        vr4300_cacheop_dc_hit_invalidate, vr4300_cacheop_dc_hit_wb_invalidate, vr4300_cacheop_dc_hit_wb,
        vr4300_cacheop_unimplemented,     vr4300_cacheop_unimplemented,        vr4300_cacheop_unimplemented,
        vr4300_cacheop_unimplemented,     vr4300_cacheop_unimplemented,        vr4300_cacheop_unimplemented,
        vr4300_cacheop_unimplemented,     vr4300_cacheop_unimplemented,        vr4300_cacheop_unimplemented,
        vr4300_cacheop_unimplemented,     vr4300_cacheop_unimplemented,        vr4300_cacheop_unimplemented,
        vr4300_cacheop_unimplemented,     vr4300_cacheop_unimplemented,        vr4300_cacheop_unimplemented,
        vr4300_cacheop_unimplemented,     vr4300_cacheop_unimplemented};

    struct vr4300_exdc_latch *exdc_latch = &g_vr4300->pipeline.exdc_latch;
    uint64_t                  vaddr      = rs + (int16_t)iw;
    uint32_t                  op_type    = (iw >> 18 & 0x7);
    unsigned                  op         = (iw >> 13 & 0x18) | op_type;
    exdc_latch->request.vaddr            = vaddr;
    exdc_latch->request.cacheop          = cacheop_lut[op];
    exdc_latch->request.type             = op_type > 2 ? VR4300_BUS_REQUEST_CACHE_WRITE : VR4300_BUS_REQUEST_CACHE_IDX;
    return 0;
}
