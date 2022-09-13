#ifndef __vr4300_cpu_h__
#define __vr4300_cpu_h__

#include <emmintrin.h>
#include "../utils/common.h"
#include "bus.h"

typedef int (*vr4300_cacheop_func_t)(uint64_t vaddr, uint32_t paddr);


struct vr4300;
struct vr4300_stats;
struct bus_controller;


enum rcp_interrupt_mask
{
    MI_INTR_SP = 0x01,
    MI_INTR_SI = 0x02,
    MI_INTR_AI = 0x04,
    MI_INTR_VI = 0x08,
    MI_INTR_PI = 0x10,
    MI_INTR_DP = 0x20
};
enum vr4300_debug_break_reason
{
    VR4300_DEBUG_BREAK_REASON_NONE,
    VR4300_DEBUG_BREAK_REASON_BREAKPOINT,
    VR4300_DEBUG_BREAK_REASON_EXCEPTION,
    VR4300_DEBUG_BREAK_REASON_PAUSE,
};
enum vr4300_opcode_id
{
    VR4300_OPCODE_INVALID,
    VR4300_OPCODE_ADD,
    VR4300_OPCODE_ADDI,
    VR4300_OPCODE_ADDIU,
    VR4300_OPCODE_ADDU,
    VR4300_OPCODE_AND,
    VR4300_OPCODE_ANDI,
    VR4300_OPCODE_BC0,
    VR4300_OPCODE_BC1,
    VR4300_OPCODE_BC2,
    VR4300_OPCODE_BEQ,
    VR4300_OPCODE_BEQL,
    VR4300_OPCODE_BGEZ,
    VR4300_OPCODE_BGEZAL,
    VR4300_OPCODE_BGEZALL,
    VR4300_OPCODE_BGEZL,
    VR4300_OPCODE_BGTZ,
    VR4300_OPCODE_BGTZL,
    VR4300_OPCODE_BLEZ,
    VR4300_OPCODE_BLEZL,
    VR4300_OPCODE_BLTZ,
    VR4300_OPCODE_BLTZAL,
    VR4300_OPCODE_BLTZALL,
    VR4300_OPCODE_BLTZL,
    VR4300_OPCODE_BNE,
    VR4300_OPCODE_BNEL,
    VR4300_OPCODE_BREAK,
    VR4300_OPCODE_CACHE,
    VR4300_OPCODE_CFC0,
    VR4300_OPCODE_CFC1,
    VR4300_OPCODE_CFC2,
    VR4300_OPCODE_CP1_ABS,
    VR4300_OPCODE_CP1_ADD,
    VR4300_OPCODE_CP1_C_EQ,
    VR4300_OPCODE_CP1_C_F,
    VR4300_OPCODE_CP1_C_LE,
    VR4300_OPCODE_CP1_C_LT,
    VR4300_OPCODE_CP1_C_NGE,
    VR4300_OPCODE_CP1_C_NGL,
    VR4300_OPCODE_CP1_C_NGLE,
    VR4300_OPCODE_CP1_C_NGT,
    VR4300_OPCODE_CP1_C_OLE,
    VR4300_OPCODE_CP1_C_OLT,
    VR4300_OPCODE_CP1_C_UEQ,
    VR4300_OPCODE_CP1_C_ULE,
    VR4300_OPCODE_CP1_C_ULT,
    VR4300_OPCODE_CP1_C_UN,
    VR4300_OPCODE_CP1_C_SEQ,
    VR4300_OPCODE_CP1_C_SF,
    VR4300_OPCODE_CP1_CEIL_L,
    VR4300_OPCODE_CP1_CEIL_W,
    VR4300_OPCODE_CP1_CVT_D,
    VR4300_OPCODE_CP1_CVT_L,
    VR4300_OPCODE_CP1_CVT_S,
    VR4300_OPCODE_CP1_CVT_W,
    VR4300_OPCODE_CP1_DIV,
    VR4300_OPCODE_CP1_FLOOR_L,
    VR4300_OPCODE_CP1_FLOOR_W,
    VR4300_OPCODE_CP1_MOV,
    VR4300_OPCODE_CP1_MUL,
    VR4300_OPCODE_CP1_NEG,
    VR4300_OPCODE_CP1_ROUND_L,
    VR4300_OPCODE_CP1_ROUND_W,
    VR4300_OPCODE_CP1_SQRT,
    VR4300_OPCODE_CP1_SUB,
    VR4300_OPCODE_CP1_TRUNC_L,
    VR4300_OPCODE_CP1_TRUNC_W,
    VR4300_OPCODE_CTC0,
    VR4300_OPCODE_CTC1,
    VR4300_OPCODE_CTC2,
    VR4300_OPCODE_DADD,
    VR4300_OPCODE_DADDI,
    VR4300_OPCODE_DADDIU,
    VR4300_OPCODE_DADDU,
    VR4300_OPCODE_DDIV,
    VR4300_OPCODE_DDIVU,
    VR4300_OPCODE_DIV,
    VR4300_OPCODE_DIVU,
    VR4300_OPCODE_DMFC0,
    VR4300_OPCODE_DMFC1,
    VR4300_OPCODE_DMFC2,
    VR4300_OPCODE_DMTC0,
    VR4300_OPCODE_DMTC1,
    VR4300_OPCODE_DMTC2,
    VR4300_OPCODE_DMULT,
    VR4300_OPCODE_DMULTU,
    VR4300_OPCODE_DSLL,
    VR4300_OPCODE_DSLLV,
    VR4300_OPCODE_DSLL32,
    VR4300_OPCODE_DSRA,
    VR4300_OPCODE_DSRAV,
    VR4300_OPCODE_DSRA32,
    VR4300_OPCODE_DSRL,
    VR4300_OPCODE_DSRLV,
    VR4300_OPCODE_DSRL32,
    VR4300_OPCODE_DSUB,
    VR4300_OPCODE_DSUBU,
    VR4300_OPCODE_ERET,
    VR4300_OPCODE_J,
    VR4300_OPCODE_JAL,
    VR4300_OPCODE_JALR,
    VR4300_OPCODE_JR,
    VR4300_OPCODE_LB,
    VR4300_OPCODE_LBU,
    VR4300_OPCODE_LD,
    VR4300_OPCODE_LDC0,
    VR4300_OPCODE_LDC1,
    VR4300_OPCODE_LDC2,
    VR4300_OPCODE_LDL,
    VR4300_OPCODE_LDR,
    VR4300_OPCODE_LH,
    VR4300_OPCODE_LHU,
    VR4300_OPCODE_LL,
    VR4300_OPCODE_LLD,
    VR4300_OPCODE_LUI,
    VR4300_OPCODE_LW,
    VR4300_OPCODE_LWC0,
    VR4300_OPCODE_LWC1,
    VR4300_OPCODE_LWC2,
    VR4300_OPCODE_LWL,
    VR4300_OPCODE_LWR,
    VR4300_OPCODE_LWU,
    VR4300_OPCODE_MFC0,
    VR4300_OPCODE_MFC1,
    VR4300_OPCODE_MFC2,
    VR4300_OPCODE_MFHI,
    VR4300_OPCODE_MFLO,
    VR4300_OPCODE_MTC0,
    VR4300_OPCODE_MTC1,
    VR4300_OPCODE_MTC2,
    VR4300_OPCODE_MTHI,
    VR4300_OPCODE_MTLO,
    VR4300_OPCODE_MULT,
    VR4300_OPCODE_MULTU,
    VR4300_OPCODE_NOR,
    VR4300_OPCODE_OR,
    VR4300_OPCODE_ORI,
    VR4300_OPCODE_SB,
    VR4300_OPCODE_SC,
    VR4300_OPCODE_SCD,
    VR4300_OPCODE_SD,
    VR4300_OPCODE_SDC0,
    VR4300_OPCODE_SDC1,
    VR4300_OPCODE_SDC2,
    VR4300_OPCODE_SDL,
    VR4300_OPCODE_SDR,
    VR4300_OPCODE_SH,
    VR4300_OPCODE_SLL,
    VR4300_OPCODE_SLLV,
    VR4300_OPCODE_SLT,
    VR4300_OPCODE_SLTI,
    VR4300_OPCODE_SLTIU,
    VR4300_OPCODE_SLTU,
    VR4300_OPCODE_SRA,
    VR4300_OPCODE_SRAV,
    VR4300_OPCODE_SRL,
    VR4300_OPCODE_SRLV,
    VR4300_OPCODE_SUB,
    VR4300_OPCODE_SUBU,
    VR4300_OPCODE_SW,
    VR4300_OPCODE_SWC0,
    VR4300_OPCODE_SWC1,
    VR4300_OPCODE_SWC2,
    VR4300_OPCODE_SWL,
    VR4300_OPCODE_SWR,
    VR4300_OPCODE_SYNC,
    VR4300_OPCODE_SYSCALL,
    VR4300_OPCODE_TEQ,
    VR4300_OPCODE_TEQI,
    VR4300_OPCODE_TGE,
    VR4300_OPCODE_TGEI,
    VR4300_OPCODE_TGEIU,
    VR4300_OPCODE_TGEU,
    VR4300_OPCODE_TLBP,
    VR4300_OPCODE_TLBR,
    VR4300_OPCODE_TLBWI,
    VR4300_OPCODE_TLBWR,
    VR4300_OPCODE_TLT,
    VR4300_OPCODE_TLTI,
    VR4300_OPCODE_TLTIU,
    VR4300_OPCODE_TLTU,
    VR4300_OPCODE_TNE,
    VR4300_OPCODE_TNEI,
    VR4300_OPCODE_XOR,
    VR4300_OPCODE_XORI,
    NUM_VR4300_OPCODES
};
enum vr4300_cp0_register
{
    VR4300_CP0_REGISTER_INDEX       = 32,
    VR4300_CP0_REGISTER_RANDOM      = 33,
    VR4300_CP0_REGISTER_ENTRYLO0    = 34,
    VR4300_CP0_REGISTER_ENTRYLO1    = 35,
    VR4300_CP0_REGISTER_CONTEXT     = 36,
    VR4300_CP0_REGISTER_PAGEMASK    = 37,
    VR4300_CP0_REGISTER_WIRED       = 38,
    VR4300_CP0_REGISTER_BADVADDR    = 40,
    VR4300_CP0_REGISTER_COUNT       = 41,
    VR4300_CP0_REGISTER_ENTRYHI     = 42,
    VR4300_CP0_REGISTER_COMPARE     = 43,
    VR4300_CP0_REGISTER_STATUS      = 44,
    VR4300_CP0_REGISTER_CAUSE       = 45,
    VR4300_CP0_REGISTER_EPC         = 46,
    VR4300_CP0_REGISTER_PRID        = 47,
    VR4300_CP0_REGISTER_CONFIG      = 48,
    VR4300_CP0_REGISTER_LLADDR      = 49,
    VR4300_CP0_REGISTER_WATCHLO     = 50,
    VR4300_CP0_REGISTER_WATCHHI     = 51,
    VR4300_CP0_REGISTER_XCONTEXT    = 52,
    VR4300_CP0_REGISTER_PARITYERROR = 58,
    VR4300_CP0_REGISTER_CACHEERR    = 59,
    VR4300_CP0_REGISTER_TAGLO       = 60,
    VR4300_CP0_REGISTER_TAGHI       = 61,
    VR4300_CP0_REGISTER_ERROREPC    = 62,
    NUM_VR4300_CP0_REGISTERS        = 32,
};
enum vr4300_fault_id
{
    VR4300_FAULT_NONE,
    VR4300_FAULT_CP0I,
    VR4300_FAULT_RST,
    VR4300_FAULT_NMI,
    VR4300_FAULT_OVFL,
    VR4300_FAULT_TRAP,
    VR4300_FAULT_FPE,
    VR4300_FAULT_DADE,
    VR4300_FAULT_DTLB,
    VR4300_FAULT_WAT,
    VR4300_FAULT_INTR,
    VR4300_FAULT_DCM,
    VR4300_FAULT_DCB,
    VR4300_FAULT_COP,
    VR4300_FAULT_DBE,
    VR4300_FAULT_SYSC,
    VR4300_FAULT_BRPT,
    VR4300_FAULT_CPU,
    VR4300_FAULT_RSVD,
    VR4300_FAULT_LDI,
    VR4300_FAULT_MCI,
    VR4300_FAULT_IADE,
    VR4300_FAULT_ITM,
    VR4300_FAULT_ICB,
    VR4300_FAULT_UNC,
    VR4300_FAULT_ITLB,
    VR4300_FAULT_IBE,
    VR4300_FAULT_RI,
    NUM_VR4300_FAULTS
};
enum vr4300_signals
{
    VR4300_SIGNAL_FORCEEXIT = 0x000000001,
    VR4300_SIGNAL_COLDRESET = 0x000000002,
    VR4300_SIGNAL_BREAK     = 0x000000004,
};
enum vr4300_register
{
    VR4300_REGISTER_R0,
    VR4300_REGISTER_AT,
    VR4300_REGISTER_V0,
    VR4300_REGISTER_V1,
    VR4300_REGISTER_A0,
    VR4300_REGISTER_A1,
    VR4300_REGISTER_A2,
    VR4300_REGISTER_A3,
    VR4300_REGISTER_T0,
    VR4300_REGISTER_T1,
    VR4300_REGISTER_T2,
    VR4300_REGISTER_T3,
    VR4300_REGISTER_T4,
    VR4300_REGISTER_T5,
    VR4300_REGISTER_T6,
    VR4300_REGISTER_T7,
    VR4300_REGISTER_S0,
    VR4300_REGISTER_S1,
    VR4300_REGISTER_S2,
    VR4300_REGISTER_S3,
    VR4300_REGISTER_S4,
    VR4300_REGISTER_S5,
    VR4300_REGISTER_S6,
    VR4300_REGISTER_S7,
    VR4300_REGISTER_T8,
    VR4300_REGISTER_T9,
    VR4300_REGISTER_K0,
    VR4300_REGISTER_K1,
    VR4300_REGISTER_GP,
    VR4300_REGISTER_SP,
    VR4300_REGISTER_FP,
    VR4300_REGISTER_RA,

    VR4300_REGISTER_CP0_0,
    VR4300_REGISTER_CP0_1,
    VR4300_REGISTER_CP0_2,
    VR4300_REGISTER_CP0_3,
    VR4300_REGISTER_CP0_4,
    VR4300_REGISTER_CP0_5,
    VR4300_REGISTER_CP0_6,
    VR4300_REGISTER_CP0_7,
    VR4300_REGISTER_CP0_8,
    VR4300_REGISTER_CP0_9,
    VR4300_REGISTER_CP0_10,
    VR4300_REGISTER_CP0_11,
    VR4300_REGISTER_CP0_12,
    VR4300_REGISTER_CP0_13,
    VR4300_REGISTER_CP0_14,
    VR4300_REGISTER_CP0_15,
    VR4300_REGISTER_CP0_16,
    VR4300_REGISTER_CP0_17,
    VR4300_REGISTER_CP0_18,
    VR4300_REGISTER_CP0_19,
    VR4300_REGISTER_CP0_20,
    VR4300_REGISTER_CP0_21,
    VR4300_REGISTER_CP0_22,
    VR4300_REGISTER_CP0_23,
    VR4300_REGISTER_CP0_24,
    VR4300_REGISTER_CP0_25,
    VR4300_REGISTER_CP0_26,
    VR4300_REGISTER_CP0_27,
    VR4300_REGISTER_CP0_28,
    VR4300_REGISTER_CP0_29,
    VR4300_REGISTER_CP0_30,
    VR4300_REGISTER_CP0_31,

    VR4300_REGISTER_CP1_0,
    VR4300_REGISTER_CP1_1,
    VR4300_REGISTER_CP1_2,
    VR4300_REGISTER_CP1_3,
    VR4300_REGISTER_CP1_4,
    VR4300_REGISTER_CP1_5,
    VR4300_REGISTER_CP1_6,
    VR4300_REGISTER_CP1_7,
    VR4300_REGISTER_CP1_8,
    VR4300_REGISTER_CP1_9,
    VR4300_REGISTER_CP1_10,
    VR4300_REGISTER_CP1_11,
    VR4300_REGISTER_CP1_12,
    VR4300_REGISTER_CP1_13,
    VR4300_REGISTER_CP1_14,
    VR4300_REGISTER_CP1_15,
    VR4300_REGISTER_CP1_16,
    VR4300_REGISTER_CP1_17,
    VR4300_REGISTER_CP1_18,
    VR4300_REGISTER_CP1_19,
    VR4300_REGISTER_CP1_20,
    VR4300_REGISTER_CP1_21,
    VR4300_REGISTER_CP1_22,
    VR4300_REGISTER_CP1_23,
    VR4300_REGISTER_CP1_24,
    VR4300_REGISTER_CP1_25,
    VR4300_REGISTER_CP1_26,
    VR4300_REGISTER_CP1_27,
    VR4300_REGISTER_CP1_28,
    VR4300_REGISTER_CP1_29,
    VR4300_REGISTER_CP1_30,
    VR4300_REGISTER_CP1_31,

    VR4300_REGISTER_HI,
    VR4300_REGISTER_LO,
    VR4300_CP1_FCR0,
    VR4300_CP1_FCR31,



    PIPELINE_CYCLE_TYPE,
    NUM_VR4300_REGISTERS
};
enum vr4300_fmt
{
    VR4300_FMT_S = 16,
    VR4300_FMT_D = 17,
    VR4300_FMT_W = 20,
    VR4300_FMT_L = 21,
};
enum vr4300_bus_request_type
{
    VR4300_BUS_REQUEST_NONE,
    VR4300_BUS_REQUEST_READ,
    VR4300_BUS_REQUEST_WRITE,
    VR4300_BUS_REQUEST_CACHE       = 4,
    VR4300_BUS_REQUEST_CACHE_IDX   = 4,
    VR4300_BUS_REQUEST_CACHE_WRITE = 5,
};
enum vr4300_access_type
{
    VR4300_ACCESS_WORD  = 1 << 5,
    VR4300_ACCESS_DWORD = 0
};
enum mi_register
{
    MI_INIT_MODE_REG,
    MI_VERSION_REG,
    MI_INTR_REG,
    MI_INTR_MASK_REG,
    NUM_MI_REGISTERS,
};
union aligned_tlb_data
{
    __m128i  __align[8];
    uint32_t data[32];
};
struct n64_tlb
{
    union aligned_tlb_data page_mask;
    union aligned_tlb_data vpn2;
    uint8_t                global[32];
    uint8_t                asid[32];
    uint8_t                valid[32];
};
struct vr4300_cp0
{
    struct n64_tlb tlb;
    uint32_t       page_mask[32];
    uint32_t       pfn[32][2];
    uint8_t        state[32][2];
};
struct segment
{
    uint64_t start;
    uint64_t length;
    uint64_t offset;
    uint8_t  xmode_mask;
    bool     mapped;
    bool     cached;
};
struct vr4300_dcache_line
{
    uint8_t  data[4 * 4];
    uint32_t metadata;
};
struct vr4300_dcache
{
    struct vr4300_dcache_line lines[512];
};
struct vr4300_opcode
{
    uint32_t id;
    uint32_t flags;
};
struct vr4300_bus_request
{
    uint64_t                     vaddr;
    uint64_t                     data;
    uint64_t                     wdqm;
    vr4300_cacheop_func_t        cacheop;
    uint32_t                     paddr;
    enum vr4300_access_type      access_type;
    enum vr4300_bus_request_type type;
    unsigned                     size, postshift;
};
struct vr4300_latch
{
    uint64_t             pc;
    enum vr4300_fault_id fault;
    uint32_t             cause_data;
};
struct vr4300_icrf_latch
{
    struct vr4300_latch   common;
    const struct segment *segment;
    uint64_t              pc;
};
struct vr4300_rfex_latch
{
    struct vr4300_latch  common;
    struct vr4300_opcode opcode;
    uint32_t             iw, iw_mask, paddr;
    bool                 cached;
};
struct vr4300_exdc_latch
{
    struct vr4300_latch       common;
    const struct segment     *segment;
    int64_t                   result;
    uint32_t                  dest;
    struct vr4300_bus_request request;
    bool                      cached;
};
struct vr4300_dcwb_latch
{
    struct vr4300_latch common;
    int64_t             result;
    uint32_t            dest;
    bool                last_op_was_cache_store;
};
struct vr4300_pipeline
{
    struct vr4300_dcwb_latch dcwb_latch;
    struct vr4300_exdc_latch exdc_latch;
    struct vr4300_rfex_latch rfex_latch;
    struct vr4300_icrf_latch icrf_latch;
    unsigned                 exception_history;
    unsigned                 cycles_to_stall;
    bool                     fault_present;
};
struct vr4300_icache_line
{
    uint8_t  data[8 * 4];
    uint32_t metadata;
};
struct vr4300_icache
{
    struct vr4300_icache_line lines[512];
};
struct vr4300_stats
{
    unsigned long executed_instructions;
    unsigned long total_cycles;
    unsigned long opcode_counts[NUM_VR4300_OPCODES];
};
struct vr4300_opcode_escape
{
    uint16_t offset;
    uint8_t  shift, mask;
};
class vr4300 {
    typedef int (vr4300::*vr4300_function)(uint32_t, uint64_t, uint64_t);
    typedef void (vr4300::*vr4300_pipeline_function)();

  public:
    struct bus_controller *bus;
    struct vr4300_pipeline pipeline;

    uint64_t regs[NUM_VR4300_REGISTERS];
    uint32_t mi_regs[NUM_MI_REGISTERS];
    unsigned signals;

    struct vr4300_cp0    cp0;
    struct vr4300_dcache dcache;
    struct vr4300_icache icache;
    uint64_t            *profile_samples;

  public:
    void check_for_interrupts(uint32_t flags);
    void clear_rcp_interrupt(enum rcp_interrupt_mask mask);
    void lower_rcp_interrupt();
    void raise_rcp_interrupt();
    void clear_dd_interrupt();
    void signal_dd_interrupt();
    void signal_rcp_interrupt(enum rcp_interrupt_mask mask);

    int      read_mi_regs(uint32_t address, uint32_t *word);
    int      write_mi_regs(uint32_t address, uint32_t word, uint32_t dqm);
    int      has_profile_samples();
    uint64_t get_profile_sample(size_t i);
    bool     vr4300_read_word_vaddr(uint64_t vaddr, uint32_t *result);
    void     vr4300_cycle();
    void     vr4300_connect_bus(struct bus_controller *bus);
    int      vr4300_init(struct bus_controller *bus, bool profiling);
    void     vr4300_print_summary(struct vr4300_stats *stats);
    uint64_t vr4300_get_register(size_t i);
    uint64_t vr4300_get_pc();
    void     vr4300_signal_break();
    void     vr4300_set_breakpoint(uint64_t at);
    void     vr4300_remove_breakpoint(uint64_t at);

    uint64_t mask_reg(unsigned reg, uint64_t data);

    int VR4300_DMFC0(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_DMTC0(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_ERET(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_MFC0(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_MTC0(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_TLBP(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_TLBR(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_TLBWI(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_TLBWR(uint32_t iw, uint64_t rs, uint64_t rt);

    void vr4300_cp0_init();
    int  vr4300_do_mci(unsigned cycles);
    int  VR4300_CP1_ABS(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_ADD(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_BC1(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_C_EQ_C_SEQ(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_C_F_C_SF(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_C_OLE_C_LE(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_C_OLT_C_LT(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_C_UEQ_C_NGL(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_C_ULE_C_NGT(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_C_ULT_C_NGE(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_C_UN_C_NGLE(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_CEIL_L(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_CEIL_W(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CFC1(uint32_t iw, uint64_t rs, uint64_t rt);
    int  VR4300_CTC1(uint32_t iw, uint64_t rs, uint64_t rt);
    int  VR4300_CP1_CVT_D(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_CVT_L(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_CVT_S(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_CVT_W(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_DIV(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_DMFC1(uint32_t iw, uint64_t fs, uint64_t rt);
    int  VR4300_DMTC1(uint32_t iw, uint64_t fs, uint64_t rt);
    int  VR4300_CP1_FLOOR_L(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_FLOOR_W(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_LDC1(uint32_t iw, uint64_t rs, uint64_t rt);
    int  VR4300_LWC1(uint32_t iw, uint64_t rs, uint64_t ft);
    int  VR4300_CP1_MUL(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_MFC1(uint32_t iw, uint64_t fs, uint64_t rt);
    int  VR4300_CP1_MOV(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_MTC1(uint32_t iw, uint64_t fs, uint64_t rt);
    int  VR4300_CP1_NEG(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_ROUND_L(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_ROUND_W(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_SDC1(uint32_t iw, uint64_t rs, uint64_t ft);
    int  VR4300_CP1_SQRT(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_SUB(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_SWC1(uint32_t iw, uint64_t rs, uint64_t ft);
    int  VR4300_CP1_TRUNC_L(uint32_t iw, uint64_t fs, uint64_t ft);
    int  VR4300_CP1_TRUNC_W(uint32_t iw, uint64_t fs, uint64_t ft);
    void vr4300_cp1_init();

    struct vr4300_dcache_line *get_line(struct vr4300_dcache *dcache, uint64_t vaddr);

    uint32_t get_tag(const struct vr4300_dcache_line *line);
    void     invalidate_line(struct vr4300_dcache_line *line);
    bool     is_dirty(const struct vr4300_dcache_line *line);
    bool     is_valid(const struct vr4300_dcache_line *line);
    void     set_dirty(struct vr4300_dcache_line *line);
    void     set_tag(struct vr4300_dcache_line *line, uint32_t tag);
    void     set_taglo(struct vr4300_dcache_line *line, uint32_t taglo);
    void     validate_line(struct vr4300_dcache_line *line, uint32_t tag);

    void     vr4300_dcache_create_dirty_exclusive(struct vr4300_dcache *dcache, uint64_t vaddr, uint32_t paddr);
    void     vr4300_dcache_fill(struct vr4300_dcache *dcache, uint64_t vaddr, uint32_t paddr, const void *data);
    uint32_t vr4300_dcache_get_tag(const struct vr4300_dcache_line *line, uint64_t vaddr);
    uint32_t vr4300_dcache_get_taglo(struct vr4300_dcache *dcache, uint64_t vaddr);
    void     vr4300_dcache_init(struct vr4300_dcache *dcache);
    void     vr4300_dcache_invalidate(struct vr4300_dcache_line *line);
    void     vr4300_dcache_invalidate_hit(struct vr4300_dcache *dcache, uint64_t vaddr, uint32_t paddr);

    struct vr4300_dcache_line *vr4300_dcache_probe(struct vr4300_dcache *dcache, uint64_t vaddr, uint32_t paddr);
    struct vr4300_dcache_line *vr4300_dcache_should_flush_line(struct vr4300_dcache *dcache, uint64_t vaddr);
    struct vr4300_dcache_line *vr4300_dcache_wb_invalidate(struct vr4300_dcache *dcache, uint64_t vaddr);

    void vr4300_dcache_set_dirty(struct vr4300_dcache_line *line);
    void vr4300_dcache_set_taglo(struct vr4300_dcache *dcache, uint64_t vaddr, uint32_t taglo);

    void vr4300_common_interlocks(unsigned cycles_to_stall, unsigned skip_stages);

    void vr4300_dc_fault(enum vr4300_fault_id fault);
    void vr4300_ex_fault(enum vr4300_fault_id fault);
    void vr4300_rf_fault(enum vr4300_fault_id fault);

    void vr4300_exception_prolog(const struct vr4300_latch *l, uint32_t *cause, uint32_t *status, uint64_t *epc);
    void vr4300_exception_epilogue(uint32_t cause, uint32_t status, uint64_t epc, uint64_t offs);
    void vr4300_tlb_exception_prolog(const struct vr4300_latch *l, uint32_t *cause, uint32_t *status, uint64_t *epc,
                                     const struct segment *segment, uint64_t vaddr, unsigned *offs);

    void VR4300_CPU();
    void VR4300_DADE();
    void VR4300_DCB();
    void VR4300_DCM();
    void VR4300_DTLB(unsigned miss, unsigned inv, unsigned mod);
    void VR4300_IADE();
    void VR4300_ICB();
    void VR4300_INTR();
    void VR4300_INV();
    void VR4300_ITLB(unsigned miss);
    void VR4300_LDI();
    void VR4300_RST();
    void VR4300_SYSC();
    void VR4300_BRPT();
    void VR4300_TRAP();
    void VR4300_RI();
    void VR4300_WAT();

    void vr4300_ic_stage();
    int  vr4300_rf_stage();
    int  vr4300_ex_stage();
    int  vr4300_dc_stage();
    int  vr4300_wb_stage();

    void vr4300_cycle_slow_wb();
    void vr4300_cycle_slow_dc();
    void vr4300_cycle_slow_ex();
    void vr4300_cycle_slow_rf();
    void vr4300_cycle_slow_ic();
    void vr4300_cycle_busywait();
    void vr4300_cycle_();
    void vr4300_cycle_extra(struct vr4300_stats *stats);
    void vr4300_pipeline_init(struct vr4300_pipeline *pipeline);

    struct vr4300_icache_line *get_line(struct vr4300_icache *icache, uint64_t vaddr);

    const struct vr4300_icache_line *get_line_const(const struct vr4300_icache *icache, uint64_t vaddr);

    uint32_t get_tag(const struct vr4300_icache_line *line);
    void     invalidate_line(struct vr4300_icache_line *line);
    bool     is_valid(const struct vr4300_icache_line *line);
    void     set_taglo(struct vr4300_icache_line *line, uint32_t taglo);
    void     validate_line(struct vr4300_icache_line *line, uint32_t tag);

    void     vr4300_icache_fill(struct vr4300_icache *icache, uint64_t vaddr, uint32_t paddr, const void *data);
    uint32_t vr4300_icache_get_tag(const struct vr4300_icache *icache, uint64_t vaddr);
    void     vr4300_icache_init(struct vr4300_icache *icache);
    void     vr4300_icache_invalidate(struct vr4300_icache *icache, uint64_t vaddr);
    void     vr4300_icache_invalidate_hit(struct vr4300_icache *icache, uint64_t vaddr, uint32_t paddr);

    const struct vr4300_icache_line *vr4300_icache_probe(const struct vr4300_icache *icache, uint64_t vaddr,
                                                         uint32_t paddr);

    void vr4300_icache_set_taglo(struct vr4300_icache *icache, uint64_t vaddr, uint32_t taglo);

    const struct segment *get_default_segment(void);
    const struct segment *get_segment(uint64_t address, uint32_t cp0_status);

    void     tlb_init(struct n64_tlb *tlb);
    unsigned tlb_probe(const struct n64_tlb *tlb, uint64_t vaddr, uint8_t vasid, unsigned *index);
    int      tlb_read(const struct n64_tlb *tlb, unsigned index, uint64_t *entry_hi);
    int      tlb_write(struct n64_tlb *tlb, unsigned index, uint64_t entry_hi, uint64_t entry_lo_0, uint64_t entry_lo_1,
                       uint32_t page_mask);

    const struct vr4300_opcode *vr4300_decode_instruction(uint32_t iw);

    uint64_t vr4300_addsub_mask(uint32_t iw);
    uint32_t vr4300_branch_mask(uint32_t iw, unsigned index);
    uint64_t VR4300_LWR_forceset(unsigned offset);

    int VR4300_ADD_SUB(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_ADDI_SUBI(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_ADDIU_LUI_SUBIU(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_ADDU_SUBU(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_AND_OR_XOR(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_ANDI_ORI_XORI(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_BEQ_BEQL_BNE_BNEL(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_BGEZ_BGEZL_BLTZ_BLTZL(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_BGEZAL_BGEZALL_BLTZAL_BLTZALL(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_BGTZ_BGTZL_BLEZ_BLEZL(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_BREAK(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_TEQ_TNE(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_TEQI_TNEI(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_TGE_TLT(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_TGEI_TLTI(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_TGEIU_TLTIU(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_TGEU_TLTU(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_CACHE(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_DADD_DSUB(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_DADDI_DSUBI(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_DADDIU_DSUBIU(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_DADDU_DSUBU(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_DIV_DIVU(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_DDIV(uint32_t iw, uint64_t _rs, uint64_t _rt);
    int VR4300_DDIVU(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_DMULT(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_DMULTU(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_DSLL_DSLL32(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_DSLLV(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_DSRA_DSRA32(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_DSRAV(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_DSRL_DSRL32(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_DSRLV(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_INVALID(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_J_JAL(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_JALR_JR(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_LD_SD(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_LOAD_STORE(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_LDL_LDR(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_LWL_LWR(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_MFHI_MFLO(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_MTHI_MTLO(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_MULT_MULTU(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_NOR(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_SLL_SLLV(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_SLT(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_SLTI(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_SLTIU(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_SLTU(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_SRA(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_SRAV(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_SRL(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_SRLV(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_SDL_SDR(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_SWL_SWR(uint32_t iw, uint64_t rs, uint64_t rt);
    int VR4300_SYSCALL(uint32_t iw, uint64_t rs, uint64_t rt);

    uint32_t fpu_get_state(void);
    void     fpu_set_state(uint32_t state);
    void     fpu_abs_32(const uint32_t *fs, uint32_t *fd);
    void     fpu_abs_64(const uint64_t *fs, uint64_t *fd);
    void     fpu_add_32(const uint32_t *fs, const uint32_t *ft, uint32_t *fd);
    void     fpu_add_64(const uint64_t *fs, const uint64_t *ft, uint64_t *fd);

    uint8_t fpu_cmp_ueq_32(const uint32_t *fs, const uint32_t *ft);
    uint8_t fpu_cmp_ueq_64(const uint64_t *fs, const uint64_t *ft);
    uint8_t fpu_cmp_ule_32(const uint32_t *fs, const uint32_t *ft);
    uint8_t fpu_cmp_ule_64(const uint64_t *fs, const uint64_t *ft);
    uint8_t fpu_cmp_ult_32(const uint32_t *fs, const uint32_t *ft);
    uint8_t fpu_cmp_ult_64(const uint64_t *fs, const uint64_t *ft);

    void fpu_cvt_f32_f64(const uint64_t *fs, uint32_t *fd);
    void fpu_cvt_f32_i32(const uint32_t *fs, uint32_t *fd);
    void fpu_cvt_f32_i64(const uint64_t *fs, uint32_t *fd);
    void fpu_cvt_f64_f32(const uint32_t *fs, uint64_t *fd);
    void fpu_cvt_f64_i32(const uint32_t *fs, uint64_t *fd);
    void fpu_cvt_f64_i64(const uint64_t *fs, uint64_t *fd);
    void fpu_cvt_i32_f32(const uint32_t *fs, uint32_t *fd);
    void fpu_cvt_i32_f64(const uint64_t *fs, uint32_t *fd);
    void fpu_cvt_i64_f32(const uint32_t *fs, uint64_t *fd);
    void fpu_cvt_i64_f64(const uint64_t *fs, uint64_t *fd);
    void fpu_div_32(const uint32_t *fs, const uint32_t *ft, uint32_t *fd);
    void fpu_div_64(const uint64_t *fs, const uint64_t *ft, uint64_t *fd);

    uint8_t fpu_cmp_eq_32(const uint32_t *fs, const uint32_t *ft);
    uint8_t fpu_cmp_eq_64(const uint64_t *fs, const uint64_t *ft);
    uint8_t fpu_cmp_f_32(const uint32_t *fs, const uint32_t *ft);
    uint8_t fpu_cmp_f_64(const uint64_t *fs, const uint64_t *ft);
    uint8_t fpu_cmp_ole_32(const uint32_t *fs, const uint32_t *ft);
    uint8_t fpu_cmp_ole_64(const uint64_t *fs, const uint64_t *ft);
    uint8_t fpu_cmp_olt_32(const uint32_t *fs, const uint32_t *ft);
    uint8_t fpu_cmp_olt_64(const uint64_t *fs, const uint64_t *ft);
    uint8_t fpu_cmp_un_32(const uint32_t *fs, const uint32_t *ft);
    uint8_t fpu_cmp_un_64(const uint64_t *fs, const uint64_t *ft);

    void fpu_mul_32(const uint32_t *fs, const uint32_t *ft, uint32_t *fd);
    void fpu_mul_64(const uint64_t *fs, const uint64_t *ft, uint64_t *fd);
    void fpu_neg_32(const uint32_t *fs, uint32_t *fd);
    void fpu_neg_64(const uint64_t *fs, uint64_t *fd);
    void fpu_sqrt_32(const uint32_t *fs, uint32_t *fd);
    void fpu_sqrt_64(const uint64_t *fs, uint64_t *fd);
    void fpu_sub_32(const uint32_t *fs, const uint32_t *ft, uint32_t *fd);
    void fpu_sub_64(const uint64_t *fs, const uint64_t *ft, uint64_t *fd);
    void fpu_trunc_i32_f32(const uint32_t *fs, uint32_t *fd);
    void fpu_trunc_i32_f64(const uint64_t *fs, uint32_t *fd);
    void fpu_trunc_i64_f32(const uint32_t *fs, uint64_t *fd);
    void fpu_trunc_i64_f64(const uint64_t *fs, uint64_t *fd);


  public:
    const struct segment USEGs[2] = {{
                                         0x0000000000000000ULL,
                                         0x0000000080000000ULL,
                                         0x0000000000000000ULL,
                                         0x20,
                                         true,
                                         true,
                                     },
                                     {
                                         0x0000000000000000ULL,
                                         0x0000010000000000ULL,
                                         0x0000000000000000ULL,
                                         0x20,
                                         true,
                                         true,
                                     }};

    const struct segment KSEGs[4] = {{
                                         0xFFFFFFFF80000000ULL,
                                         0x0000000020000000ULL,
                                         0xFFFFFFFF80000000ULL,
                                         0x80,
                                         false,
                                         true,
                                     },
                                     {
                                         0xFFFFFFFFA0000000ULL,
                                         0x0000000020000000ULL,
                                         0xFFFFFFFFA0000000ULL,
                                         0x80,
                                         false,
                                         false,
                                     },
                                     {
                                         0xFFFFFFFFC0000000ULL,
                                         0x0000000020000000ULL,
                                         0x0000000000000000ULL,
                                         0x80,
                                         true,
                                         true,
                                     },
                                     {
                                         0xFFFFFFFFE0000000ULL,
                                         0x0000000020000000ULL,
                                         0x0000000000000000ULL,
                                         0x80,
                                         true,
                                         true,
                                     }};

    const struct segment XSSEG = {
        0x4000000000000000ULL, 0x0000010000000000ULL, 0x0000000000000000ULL, 0x40, true, true,
    };
    const struct segment XKSEG = {
        0xC000000000000000ULL, 0x0000010000000000ULL, 0x0000000000000000ULL, 0x80, true, true,
    };
    const struct segment XKPHYS0 = {
        0x8000000000000000ULL, 0x0000000100000000ULL, 0x8000000000000000ULL, 0x80, false, true,
    };
    const struct segment XKPHYS1 = {
        0x8800000000000000ULL, 0x0000000100000000ULL, 0x8800000000000000ULL, 0x80, false, true,
    };
    const struct segment XKPHYS2 = {
        0x9000000000000000ULL, 0x0000000100000000ULL, 0x9000000000000000ULL, 0x80, false, false,
    };
    const struct segment XKPHYS3 = {
        0x9800000000000000ULL, 0x0000000100000000ULL, 0x9800000000000000ULL, 0x80, false, true,
    };
    const struct segment XKPHYS4 = {
        0xA000000000000000ULL, 0x0000000100000000ULL, 0xA000000000000000ULL, 0x80, false, true,
    };
    const struct segment XKPHYS5 = {
        0xA800000000000000ULL, 0x0000000100000000ULL, 0xA800000000000000ULL, 0x80, false, true,
    };
    const struct segment XKPHYS6 = {
        0xB000000000000000ULL, 0x0000000100000000ULL, 0xB000000000000000ULL, 0x80, false, true,
    };
    const struct segment XKPHYS7 = {
        0xB800000000000000ULL, 0x0000000100000000ULL, 0xB800000000000000ULL, 0x80, false, true,
    };
    const struct segment default_segment = {
        1ULL, 0ULL, 0ULL, 0x0, false, false,
    };
    const struct segment *kernel_segs_lut[16] = {
        &XKPHYS0, &XKPHYS1, &XKPHYS2, &XKPHYS3, &XKPHYS4, &XKPHYS5, &XKPHYS6, &XKPHYS7,
        &XKSEG,   &XKSEG,   &XKSEG,   &XKSEG,   &XKSEG,   &XKSEG,   &XKSEG,   &XKSEG,
    };

  public:
    const uint64_t                 vr4300_mult_sex_mask[2]  = {~0ULL, ~0ULL >> 32};
    const vr4300_pipeline_function pipeline_function_lut[7] = {
        &vr4300::vr4300_cycle_slow_wb, &vr4300::vr4300_cycle_slow_dc, &vr4300::vr4300_cycle_slow_ex,
        &vr4300::vr4300_cycle_slow_rf, &vr4300::vr4300_cycle_slow_ic, &vr4300::vr4300_cycle_busywait,
        &vr4300::VR4300_DCM,
    };
    const uint64_t vr4300_cp0_reg_masks[32] = {
        0x000000008000003FULL, 0x000000000000003FULL, 0x000000007FFFFFFFULL, 0x000000007FFFFFFFULL,
        0xFFFFFFFFFFFFFFF0ULL, 0x0000000001FFE000ULL, 0xFFFFFFFFFFFFFFFFULL, 0x0000000000000BADULL,
        0xFFFFFFFFFFFFFFFFULL, 0x00000000FFFFFFFFULL, 0xC00000FFFFFFE0FFULL, 0x00000000FFFFFFFFULL,
        0x00000000FFFFFFFFULL, 0x00000000B000FFFFULL, 0xFFFFFFFFFFFFFFFFULL, 0x000000000000FFFFULL,
        0x000000007FFFFFFFULL, 0x00000000FFFFFFFFULL, 0x00000000FFFFFFFBULL, 0x000000000000000FULL,
        0xFFFFFFFFFFFFFFFFULL, 0x0000000000000BADULL, 0x0000000000000BADULL, 0x0000000000000BADULL,
        0x0000000000000BADULL, 0x0000000000000BADULL, 0x0000000000000000ULL, 0x0000000000000000ULL,
        0x000000000FFFFFC0ULL, 0x0000000000000000ULL, 0xFFFFFFFFFFFFFFFFULL, 0x0000000000000BADULL,
    };
    const int8_t n64_one_hot_lut[256] = {
        -1, 0,  1,  -1, 2,  -1, -1, -1, 3,  -1, -1, -1, -1, -1, -1, -1, 4,  -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, 5,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 7,  -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    const uint64_t vr4300_load_sex_mask[8] = {
        ~0ULL, ~0ULL, 0ULL, ~0ULL, 0xFFULL, 0xFFFFULL, 0ULL, 0xFFFFFFFFULL,
    };
    const struct vr4300_opcode vr4300_opcode_table[352] = {
        {(VR4300_OPCODE_SLL), (((1U << 4)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_SRL), (((1U << 4)))},
        {(VR4300_OPCODE_SRA), (((1U << 4)))},
        {(VR4300_OPCODE_SLLV), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_SRLV), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_SRAV), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_JR), ((((1U << 31)) | (1U << 3)))},
        {(VR4300_OPCODE_JALR), ((((1U << 31)) | (1U << 3)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_SYSCALL), (((0)))},
        {(VR4300_OPCODE_BREAK), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_SYNC), (((0)))},
        {(VR4300_OPCODE_MFHI), (((0)))},
        {(VR4300_OPCODE_MTHI), (((1U << 3)))},
        {(VR4300_OPCODE_MFLO), (((0)))},
        {(VR4300_OPCODE_MTLO), (((1U << 3)))},
        {(VR4300_OPCODE_DSLLV), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_DSRLV), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_DSRAV), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_MULT), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_MULTU), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_DIV), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_DIVU), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_DMULT), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_DMULTU), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_DDIV), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_DDIVU), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_ADD), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_ADDU), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_SUB), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_SUBU), (((1U << 3)))},
        {(VR4300_OPCODE_AND), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_OR), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_XOR), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_NOR), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_SLT), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_SLTU), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_DADD), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_DADDU), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_DSUB), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_DSUBU), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_TGE), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_TGEU), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_TLT), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_TLTU), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_TEQ), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_TNE), ((((1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_DSLL), (((1U << 4)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_DSRL), (((1U << 4)))},
        {(VR4300_OPCODE_DSRA), (((1U << 4)))},
        {(VR4300_OPCODE_DSLL32), (((1U << 4)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_DSRL32), (((1U << 4)))},
        {(VR4300_OPCODE_DSRA32), (((1U << 4)))},
        {(VR4300_OPCODE_BLTZ), ((((1U << 31)) | (1U << 3)))},
        {(VR4300_OPCODE_BGEZ), ((((1U << 31)) | (1U << 3)))},
        {(VR4300_OPCODE_BLTZL), ((((1U << 31)) | (1U << 3)))},
        {(VR4300_OPCODE_BGEZL), ((((1U << 31)) | (1U << 3)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_TGEI), (((1U << 3)))},
        {(VR4300_OPCODE_TGEIU), (((1U << 3)))},
        {(VR4300_OPCODE_TLTI), (((1U << 3)))},
        {(VR4300_OPCODE_TLTIU), (((1U << 3)))},
        {(VR4300_OPCODE_TEQI), (((1U << 3)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_TNEI), (((1U << 3)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_BLTZAL), ((((1U << 31)) | (1U << 3)))},
        {(VR4300_OPCODE_BGEZAL), ((((1U << 31)) | (1U << 3)))},
        {(VR4300_OPCODE_BLTZALL), ((((1U << 31)) | (1U << 3)))},
        {(VR4300_OPCODE_BGEZALL), ((((1U << 31)) | (1U << 3)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_MFC0), (((0)))},
        {(VR4300_OPCODE_DMFC0), (((0)))},
        {(VR4300_OPCODE_CFC0), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_MTC0), (((1U << 4)))},
        {(VR4300_OPCODE_DMTC0), (((1U << 4)))},
        {(VR4300_OPCODE_CTC0), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_BC0), (((1U << 31)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_TLBR), (((0)))},
        {(VR4300_OPCODE_TLBWI), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_TLBWR), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_TLBP), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_ERET), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_MFC1), ((((1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_DMFC1), ((((1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_CFC1), ((((1U << 2)) | (0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_MTC1), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | (1U << 4)))},
        {(VR4300_OPCODE_DMTC1), ((((1U << 2)) | (1U << 4)))},
        {(VR4300_OPCODE_CTC1), ((((1U << 2)) | (0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_BC1), ((((1U << 2)) | (1U << 31)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_CP1_ADD), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_SUB), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_MUL), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_DIV), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_SQRT), (((((1U << 2)) | (1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_CP1_ABS), (((((1U << 2)) | (1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_CP1_MOV), (((((1U << 2)) | (1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_CP1_NEG), (((((1U << 2)) | (1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_CP1_ROUND_L), (((((1U << 2)) | (1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_CP1_TRUNC_L), (((((1U << 2)) | (1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_CP1_CEIL_L), (((((1U << 2)) | (1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_CP1_FLOOR_L), (((((1U << 2)) | (1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_CP1_ROUND_W), (((((1U << 2)) | (1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_CP1_TRUNC_W), (((((1U << 2)) | (1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_CP1_CEIL_W), (((((1U << 2)) | (1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_CP1_FLOOR_W), (((((1U << 2)) | (1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_CP1_CVT_S), (((((1U << 2)) | (1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_CP1_CVT_D), (((((1U << 2)) | (1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_CP1_CVT_W), (((((1U << 2)) | (1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_CP1_CVT_L), (((((1U << 2)) | (1U << 2)) | ((1U << 3) | (1U << 0))))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_CP1_C_F), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_C_UN), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_C_EQ), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_C_UEQ), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_C_OLT), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_C_ULT), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_C_OLE), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_C_ULE), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_C_SF), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_C_NGLE), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_C_SEQ), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_C_NGL), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_C_LT), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_C_NGE), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_C_LE), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_CP1_C_NGT), (((((1U << 2)) | ((1U << 3) | (1U << 0))) | ((1U << 4) | (1U << 1))))},
        {(VR4300_OPCODE_MFC2), (((0)))},
        {(VR4300_OPCODE_DMFC2), (((0)))},
        {(VR4300_OPCODE_CFC2), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_MTC2), (((0)))},
        {(VR4300_OPCODE_DMTC2), (((0)))},
        {(VR4300_OPCODE_CTC2), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_BC1), (((1U << 31)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_J), (((1U << 31)))},
        {(VR4300_OPCODE_JAL), (((1U << 31)))},
        {(VR4300_OPCODE_BEQ), (((((1U << 31)) | (1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_BNE), (((((1U << 31)) | (1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_BLEZ), ((((1U << 31)) | (1U << 3)))},
        {(VR4300_OPCODE_BGTZ), ((((1U << 31)) | (1U << 3)))},
        {(VR4300_OPCODE_ADDI), (((1U << 3)))},
        {(VR4300_OPCODE_ADDIU), (((1U << 3)))},
        {(VR4300_OPCODE_SLTI), (((1U << 3)))},
        {(VR4300_OPCODE_SLTIU), (((1U << 3)))},
        {(VR4300_OPCODE_ANDI), (((1U << 3)))},
        {(VR4300_OPCODE_ORI), (((1U << 3)))},
        {(VR4300_OPCODE_XORI), (((1U << 3)))},
        {(VR4300_OPCODE_LUI), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_BEQL), (((((1U << 31)) | (1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_BNEL), (((((1U << 31)) | (1U << 3)) | (1U << 4)))},
        {(VR4300_OPCODE_BLEZL), ((((1U << 31)) | (1U << 3)))},
        {(VR4300_OPCODE_BGTZL), ((((1U << 31)) | (1U << 3)))},
        {(VR4300_OPCODE_DADDI), (((1U << 3)))},
        {(VR4300_OPCODE_DADDIU), (((1U << 3)))},
        {(VR4300_OPCODE_LDL), (((((1U << 3)) | (1U << 4)) | (1U << 5)))},
        {(VR4300_OPCODE_LDR), (((((1U << 3)) | (1U << 4)) | (1U << 5)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_LB), ((((1U << 3)) | (1U << 5)))},
        {(VR4300_OPCODE_LH), ((((1U << 3)) | (1U << 5)))},
        {(VR4300_OPCODE_LWL), (((((1U << 3)) | (1U << 4)) | (1U << 5)))},
        {(VR4300_OPCODE_LW), ((((1U << 3)) | (1U << 5)))},
        {(VR4300_OPCODE_LBU), ((((1U << 3)) | (1U << 5)))},
        {(VR4300_OPCODE_LHU), ((((1U << 3)) | (1U << 5)))},
        {(VR4300_OPCODE_LWR), (((((1U << 3)) | (1U << 4)) | (1U << 5)))},
        {(VR4300_OPCODE_LWU), ((((1U << 3)) | (1U << 5)))},
        {(VR4300_OPCODE_SB), (((((1U << 3)) | (1U << 4)) | (1U << 6)))},
        {(VR4300_OPCODE_SH), (((((1U << 3)) | (1U << 4)) | (1U << 6)))},
        {(VR4300_OPCODE_SWL), (((((1U << 3)) | (1U << 4)) | (1U << 6)))},
        {(VR4300_OPCODE_SW), (((((1U << 3)) | (1U << 4)) | (1U << 6)))},
        {(VR4300_OPCODE_SDL), (((((1U << 3)) | (1U << 4)) | (1U << 6)))},
        {(VR4300_OPCODE_SDR), (((((1U << 3)) | (1U << 4)) | (1U << 6)))},
        {(VR4300_OPCODE_SWR), (((((1U << 3)) | (1U << 4)) | (1U << 6)))},
        {(VR4300_OPCODE_CACHE), (((1U << 3)))},
        {(VR4300_OPCODE_LL), ((((1U << 3)) | (1U << 5)))},
        {(VR4300_OPCODE_LWC1), ((((((1U << 2)) | (1U << 3)) | ((1U << 4) | (1U << 1))) | (1U << 5)))},
        {(VR4300_OPCODE_LWC2), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_LLD), (((0)))},
        {(VR4300_OPCODE_LDC1), (((((1U << 2)) | (1U << 3)) | (1U << 5)))},
        {(VR4300_OPCODE_LDC2), (((0)))},
        {(VR4300_OPCODE_LD), ((((1U << 3)) | (1U << 5)))},
        {(VR4300_OPCODE_SC), (((((1U << 3)) | (1U << 4)) | (1U << 6)))},
        {(VR4300_OPCODE_SWC1), ((((((1U << 2)) | (1U << 3)) | ((1U << 4) | (1U << 1))) | (1U << 6)))},
        {(VR4300_OPCODE_SWC2), (((0)))},
        {(VR4300_OPCODE_INVALID), (((0)))},
        {(VR4300_OPCODE_SCD), (((0)))},
        {(VR4300_OPCODE_SDC1), ((((((1U << 2)) | (1U << 3)) | ((1U << 4) | (1U << 1))) | (1U << 6)))},
        {(VR4300_OPCODE_SDC2), (((0)))},
        {(VR4300_OPCODE_SD), (((((1U << 3)) | (1U << 4)) | (1U << 6)))}};

    const struct vr4300_opcode_escape vr4300_escape_table[128] = {
        {0, 0, 0x3F},    {0, 0, 0x3F},    {64, 16, 0x1F},  {64, 16, 0x1F},  {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {96, 21, 0x0F},  {112, 0, 0x3F},  {176, 21, 0x0F}, {192, 0, 0x3F},
        {256, 21, 0x1F}, {256, 21, 0x1F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F}, {288, 26, 0x3F},
        {288, 26, 0x3F}, {288, 26, 0x3F},
    };

  public:
    const vr4300_function vr4300_function_table[NUM_VR4300_OPCODES] = {
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_ADD_SUB),
        (&vr4300::VR4300_ADDI_SUBI),
        (&vr4300::VR4300_ADDIU_LUI_SUBIU),
        (&vr4300::VR4300_ADDU_SUBU),
        (&vr4300::VR4300_AND_OR_XOR),
        (&vr4300::VR4300_ANDI_ORI_XORI),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_BC1),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_BEQ_BEQL_BNE_BNEL),
        (&vr4300::VR4300_BEQ_BEQL_BNE_BNEL),
        (&vr4300::VR4300_BGEZ_BGEZL_BLTZ_BLTZL),
        (&vr4300::VR4300_BGEZAL_BGEZALL_BLTZAL_BLTZALL),
        (&vr4300::VR4300_BGEZAL_BGEZALL_BLTZAL_BLTZALL),
        (&vr4300::VR4300_BGEZ_BGEZL_BLTZ_BLTZL),
        (&vr4300::VR4300_BGTZ_BGTZL_BLEZ_BLEZL),
        (&vr4300::VR4300_BGTZ_BGTZL_BLEZ_BLEZL),
        (&vr4300::VR4300_BGTZ_BGTZL_BLEZ_BLEZL),
        (&vr4300::VR4300_BGTZ_BGTZL_BLEZ_BLEZL),
        (&vr4300::VR4300_BGEZ_BGEZL_BLTZ_BLTZL),
        (&vr4300::VR4300_BGEZAL_BGEZALL_BLTZAL_BLTZALL),
        (&vr4300::VR4300_BGEZAL_BGEZALL_BLTZAL_BLTZALL),
        (&vr4300::VR4300_BGEZ_BGEZL_BLTZ_BLTZL),
        (&vr4300::VR4300_BEQ_BEQL_BNE_BNEL),
        (&vr4300::VR4300_BEQ_BEQL_BNE_BNEL),
        (&vr4300::VR4300_BREAK),
        (&vr4300::VR4300_CACHE),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_CFC1),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_CP1_ABS),
        (&vr4300::VR4300_CP1_ADD),
        (&vr4300::VR4300_CP1_C_EQ_C_SEQ),
        (&vr4300::VR4300_CP1_C_F_C_SF),
        (&vr4300::VR4300_CP1_C_OLE_C_LE),
        (&vr4300::VR4300_CP1_C_OLT_C_LT),
        (&vr4300::VR4300_CP1_C_ULT_C_NGE),
        (&vr4300::VR4300_CP1_C_UEQ_C_NGL),
        (&vr4300::VR4300_CP1_C_UN_C_NGLE),
        (&vr4300::VR4300_CP1_C_ULE_C_NGT),
        (&vr4300::VR4300_CP1_C_OLE_C_LE),
        (&vr4300::VR4300_CP1_C_OLT_C_LT),
        (&vr4300::VR4300_CP1_C_UEQ_C_NGL),
        (&vr4300::VR4300_CP1_C_ULE_C_NGT),
        (&vr4300::VR4300_CP1_C_ULT_C_NGE),
        (&vr4300::VR4300_CP1_C_UN_C_NGLE),
        (&vr4300::VR4300_CP1_C_EQ_C_SEQ),
        (&vr4300::VR4300_CP1_C_F_C_SF),
        (&vr4300::VR4300_CP1_CEIL_L),
        (&vr4300::VR4300_CP1_CEIL_W),
        (&vr4300::VR4300_CP1_CVT_D),
        (&vr4300::VR4300_CP1_CVT_L),
        (&vr4300::VR4300_CP1_CVT_S),
        (&vr4300::VR4300_CP1_CVT_W),
        (&vr4300::VR4300_CP1_DIV),
        (&vr4300::VR4300_CP1_FLOOR_L),
        (&vr4300::VR4300_CP1_FLOOR_W),
        (&vr4300::VR4300_CP1_MOV),
        (&vr4300::VR4300_CP1_MUL),
        (&vr4300::VR4300_CP1_NEG),
        (&vr4300::VR4300_CP1_ROUND_L),
        (&vr4300::VR4300_CP1_ROUND_W),
        (&vr4300::VR4300_CP1_SQRT),
        (&vr4300::VR4300_CP1_SUB),
        (&vr4300::VR4300_CP1_TRUNC_L),
        (&vr4300::VR4300_CP1_TRUNC_W),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_CTC1),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_DADD_DSUB),
        (&vr4300::VR4300_DADDI_DSUBI),
        (&vr4300::VR4300_DADDIU_DSUBIU),
        (&vr4300::VR4300_DADDU_DSUBU),
        (&vr4300::VR4300_DDIV),
        (&vr4300::VR4300_DDIVU),
        (&vr4300::VR4300_DIV_DIVU),
        (&vr4300::VR4300_DIV_DIVU),
        (&vr4300::VR4300_DMFC0),
        (&vr4300::VR4300_DMFC1),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_DMTC0),
        (&vr4300::VR4300_DMTC1),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_DMULT),
        (&vr4300::VR4300_DMULTU),
        (&vr4300::VR4300_DSLL_DSLL32),
        (&vr4300::VR4300_DSLLV),
        (&vr4300::VR4300_DSLL_DSLL32),
        (&vr4300::VR4300_DSRA_DSRA32),
        (&vr4300::VR4300_DSRAV),
        (&vr4300::VR4300_DSRA_DSRA32),
        (&vr4300::VR4300_DSRL_DSRL32),
        (&vr4300::VR4300_DSRLV),
        (&vr4300::VR4300_DSRL_DSRL32),
        (&vr4300::VR4300_DADD_DSUB),
        (&vr4300::VR4300_DADDU_DSUBU),
        (&vr4300::VR4300_ERET),
        (&vr4300::VR4300_J_JAL),
        (&vr4300::VR4300_J_JAL),
        (&vr4300::VR4300_JALR_JR),
        (&vr4300::VR4300_JALR_JR),
        (&vr4300::VR4300_LOAD_STORE),
        (&vr4300::VR4300_LOAD_STORE),
        (&vr4300::VR4300_LD_SD),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_LDC1),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_LDL_LDR),
        (&vr4300::VR4300_LDL_LDR),
        (&vr4300::VR4300_LOAD_STORE),
        (&vr4300::VR4300_LOAD_STORE),
        (&vr4300::VR4300_LOAD_STORE),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_ADDIU_LUI_SUBIU),
        (&vr4300::VR4300_LOAD_STORE),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_LWC1),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_LWL_LWR),
        (&vr4300::VR4300_LWL_LWR),
        (&vr4300::VR4300_LOAD_STORE),
        (&vr4300::VR4300_MFC0),
        (&vr4300::VR4300_MFC1),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_MFHI_MFLO),
        (&vr4300::VR4300_MFHI_MFLO),
        (&vr4300::VR4300_MTC0),
        (&vr4300::VR4300_MTC1),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_MTHI_MTLO),
        (&vr4300::VR4300_MTHI_MTLO),
        (&vr4300::VR4300_MULT_MULTU),
        (&vr4300::VR4300_MULT_MULTU),
        (&vr4300::VR4300_NOR),
        (&vr4300::VR4300_AND_OR_XOR),
        (&vr4300::VR4300_ANDI_ORI_XORI),
        (&vr4300::VR4300_LOAD_STORE),
        (&vr4300::VR4300_LOAD_STORE),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_LD_SD),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_SDC1),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_SDL_SDR),
        (&vr4300::VR4300_SDL_SDR),
        (&vr4300::VR4300_LOAD_STORE),
        (&vr4300::VR4300_SLL_SLLV),
        (&vr4300::VR4300_SLL_SLLV),
        (&vr4300::VR4300_SLT),
        (&vr4300::VR4300_SLTI),
        (&vr4300::VR4300_SLTIU),
        (&vr4300::VR4300_SLTU),
        (&vr4300::VR4300_SRA),
        (&vr4300::VR4300_SRAV),
        (&vr4300::VR4300_SRL),
        (&vr4300::VR4300_SRLV),
        (&vr4300::VR4300_ADD_SUB),
        (&vr4300::VR4300_ADDU_SUBU),
        (&vr4300::VR4300_LOAD_STORE),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_SWC1),
        (&vr4300::VR4300_INVALID),
        (&vr4300::VR4300_SWL_SWR),
        (&vr4300::VR4300_SWL_SWR),
        (&vr4300::VR4300_SLL_SLLV),
        (&vr4300::VR4300_SYSCALL),
        (&vr4300::VR4300_TEQ_TNE),
        (&vr4300::VR4300_TEQI_TNEI),
        (&vr4300::VR4300_TGE_TLT),
        (&vr4300::VR4300_TGEI_TLTI),
        (&vr4300::VR4300_TGEIU_TLTIU),
        (&vr4300::VR4300_TGEU_TLTU),
        (&vr4300::VR4300_TLBP),
        (&vr4300::VR4300_TLBR),
        (&vr4300::VR4300_TLBWI),
        (&vr4300::VR4300_TLBWR),
        (&vr4300::VR4300_TGE_TLT),
        (&vr4300::VR4300_TGEI_TLTI),
        (&vr4300::VR4300_TGEIU_TLTIU),
        (&vr4300::VR4300_TGEU_TLTU),
        (&vr4300::VR4300_TEQ_TNE),
        (&vr4300::VR4300_TEQI_TNEI),
        (&vr4300::VR4300_AND_OR_XOR),
        (&vr4300::VR4300_ANDI_ORI_XORI),
    };
};
#endif
