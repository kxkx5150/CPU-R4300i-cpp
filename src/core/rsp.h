#ifndef __arch_rsp_h__
#define __arch_rsp_h__
#include "../../utils/common.h"
#include <emmintrin.h>
#include <stddef.h>

#define SP_STATUS_HALT       0x0001
#define SP_STATUS_BROKE      0x0002
#define SP_STATUS_DMA_BUSY   0x0004
#define SP_STATUS_DMA_FULL   0x0008
#define SP_STATUS_IO_FULL    0x0010
#define SP_STATUS_SSTEP      0x0020
#define SP_STATUS_INTR_BREAK 0x0040
#define SP_STATUS_SIG0       0x0080
#define SP_STATUS_SIG1       0x0100
#define SP_STATUS_SIG2       0x0200
#define SP_STATUS_SIG3       0x0400
#define SP_STATUS_SIG4       0x0800
#define SP_STATUS_SIG5       0x1000
#define SP_STATUS_SIG6       0x2000
#define SP_STATUS_SIG7       0x4000

#define SP_CLR_HALT       0x00000001
#define SP_SET_HALT       0x00000002
#define SP_CLR_BROKE      0x00000004
#define SP_CLR_INTR       0x00000008
#define SP_SET_INTR       0x00000010
#define SP_CLR_SSTEP      0x00000020
#define SP_SET_SSTEP      0x00000040
#define SP_CLR_INTR_BREAK 0x00000080
#define SP_SET_INTR_BREAK 0x00000100
#define SP_CLR_SIG0       0x00000200
#define SP_SET_SIG0       0x00000400
#define SP_CLR_SIG1       0x00000800
#define SP_SET_SIG1       0x00001000
#define SP_CLR_SIG2       0x00002000
#define SP_SET_SIG2       0x00004000
#define SP_CLR_SIG3       0x00008000
#define SP_SET_SIG3       0x00010000
#define SP_CLR_SIG4       0x00020000
#define SP_SET_SIG4       0x00040000
#define SP_CLR_SIG5       0x00080000
#define SP_SET_SIG5       0x00100000
#define SP_CLR_SIG6       0x00200000
#define SP_SET_SIG6       0x00400000
#define SP_CLR_SIG7       0x00800000
#define SP_SET_SIG7       0x01000000
#define SP_SET_BROKE      0x80000000

#define GET_RS(iw)         ((iw) >> 21 & 0x1F)
#define GET_RT(iw)         ((iw) >> 16 & 0x1F)
#define GET_RD(iw)         ((iw) >> 11 & 0x1F)
#define GET_VS(iw)         ((iw) >> 11 & 0x1F)
#define GET_VT(iw)         ((iw) >> 16 & 0x1F)
#define GET_VD(iw)         ((iw) >> 6 & 0x1F)
#define GET_DE(iw)         ((iw) >> 11 & 0x1F)
#define GET_EL(iw)         ((iw) >> 7 & 0xF)
#define GET_E(iw)          ((iw) >> 21 & 0xF)
#define OPCODE_INFO_NONE   (0)
#define OPCODE_INFO_VECTOR (1U << 1)
#define OPCODE_INFO_BRANCH (1U << 31)
#define OPCODE_INFO_NEEDRS (1U << 3)
#define OPCODE_INFO_NEEDRT (1U << 4)
#define OPCODE_INFO_NEEDVS (1U << 3)
#define OPCODE_INFO_NEEDVT (1U << 4)
#define OPCODE_INFO_LOAD   (1U << 5)
#define OPCODE_INFO_STORE  (1U << 6)

#define RSP_BUILD_OP(op, func, flags) (RSP_OPCODE_##op), (flags)

#define INFO1(x)             (OPCODE_INFO_##x)
#define INFO2(x, y)          (INFO1(x) | OPCODE_INFO_##y)
#define INFO3(x, y, z)       (INFO2(x, y) | OPCODE_INFO_##z)
#define INFO4(x, y, z, a)    (INFO3(x, y, z) | OPCODE_INFO_##a)
#define INFO5(x, y, z, a, b) (INFO4(x, y, z, a) | OPCODE_INFO_##b)

#define INVALID  RSP_BUILD_OP(INVALID, INVALID, INFO1(NONE))
#define VINVALID RSP_BUILD_OP(VINVALID, VINVALID, INFO1(VECTOR))
#define ADDIU    RSP_BUILD_OP(ADDIU, ADDIU_LUI_SUBIU, INFO1(NEEDRS))
#define ADDU     RSP_BUILD_OP(ADDU, ADDU_SUBU, INFO2(NEEDRS, NEEDRT))
#define AND      RSP_BUILD_OP(AND, AND_OR_XOR, INFO2(NEEDRS, NEEDRT))
#define ANDI     RSP_BUILD_OP(ANDI, ANDI_ORI_XORI, INFO1(NEEDRS))
#define BEQ      RSP_BUILD_OP(BEQ, BEQ_BNE, INFO3(BRANCH, NEEDRS, NEEDRT))
#define BGEZ     RSP_BUILD_OP(BGEZ, BGEZ_BLTZ, INFO2(BRANCH, NEEDRS))
#define BGEZAL   RSP_BUILD_OP(BGEZAL, BGEZAL_BLTZAL, INFO2(BRANCH, NEEDRS))
#define BGTZ     RSP_BUILD_OP(BGTZ, BGTZ_BLEZ, INFO2(BRANCH, NEEDRS))
#define BLEZ     RSP_BUILD_OP(BLEZ, BGTZ_BLEZ, INFO2(BRANCH, NEEDRS))
#define BLTZ     RSP_BUILD_OP(BLTZ, BGEZ_BLTZ, INFO2(BRANCH, NEEDRS))
#define BLTZAL   RSP_BUILD_OP(BLTZAL, BGEZAL_BLTZAL, INFO2(BRANCH, NEEDRS))
#define BNE      RSP_BUILD_OP(BNE, BEQ_BNE, INFO3(BRANCH, NEEDRS, NEEDRT))
#define BREAK    RSP_BUILD_OP(BREAK, BREAK, INFO1(NONE))
#define J        RSP_BUILD_OP(J, J_JAL, INFO1(BRANCH))
#define JAL      RSP_BUILD_OP(JAL, J_JAL, INFO1(BRANCH))
#define JALR     RSP_BUILD_OP(JALR, JALR_JR, INFO2(BRANCH, NEEDRS))
#define JR       RSP_BUILD_OP(JR, JALR_JR, INFO2(BRANCH, NEEDRS))
#define LB       RSP_BUILD_OP(LB, INT_MEM, INFO2(NEEDRS, LOAD))
#define LBU      RSP_BUILD_OP(LBU, INT_MEM, INFO2(NEEDRS, LOAD))
#define LH       RSP_BUILD_OP(LH, INT_MEM, INFO2(NEEDRS, LOAD))
#define LHU      RSP_BUILD_OP(LHU, INT_MEM, INFO2(NEEDRS, LOAD))
#define LUI      RSP_BUILD_OP(LUI, ADDIU_LUI_SUBIU, INFO1(NONE))
#define LW       RSP_BUILD_OP(LW, INT_MEM, INFO2(NEEDRS, LOAD))
#define NOP      RSP_BUILD_OP(NOP, INVALID, INFO1(NONE))
#define NOR      RSP_BUILD_OP(NOR, NOR, INFO2(NEEDRS, NEEDRT))
#define OR       RSP_BUILD_OP(OR, AND_OR_XOR, INFO2(NEEDRS, NEEDRT))
#define ORI      RSP_BUILD_OP(ORI, ANDI_ORI_XORI, INFO1(NEEDRS))
#define SB       RSP_BUILD_OP(SB, INT_MEM, INFO3(NEEDRS, NEEDRT, STORE))
#define SH       RSP_BUILD_OP(SH, INT_MEM, INFO3(NEEDRS, NEEDRT, STORE))
#define SLL      RSP_BUILD_OP(SLL, SLL_SLLV, INFO1(NEEDRT))
#define SLLV     RSP_BUILD_OP(SLLV, SLL_SLLV, INFO2(NEEDRS, NEEDRT))
#define SLT      RSP_BUILD_OP(SLT, SLT, INFO2(NEEDRS, NEEDRT))
#define SLTI     RSP_BUILD_OP(SLTI, SLTI, INFO1(NEEDRS))
#define SLTIU    RSP_BUILD_OP(SLTIU, SLTIU, INFO1(NEEDRS))
#define SLTU     RSP_BUILD_OP(SLTU, SLTU, INFO2(NEEDRS, NEEDRT))
#define SRA      RSP_BUILD_OP(SRA, SRA, INFO1(NEEDRT))
#define SRAV     RSP_BUILD_OP(SRAV, SRAV, INFO2(NEEDRS, NEEDRT))
#define SRL      RSP_BUILD_OP(SRL, SRL, INFO1(NEEDRT))
#define SRLV     RSP_BUILD_OP(SRLV, SRLV, INFO2(NEEDRS, NEEDRT))
#define SUBU     RSP_BUILD_OP(SUBU, ADDU_SUBU, INFO1(NEEDRS))
#define SW       RSP_BUILD_OP(SW, INT_MEM, INFO3(NEEDRS, NEEDRT, STORE))
#define XOR      RSP_BUILD_OP(XOR, AND_OR_XOR, INFO2(NEEDRS, NEEDRT))
#define XORI     RSP_BUILD_OP(XORI, ANDI_ORI_XORI, INFO1(NEEDRS))
#define MFC0     RSP_BUILD_OP(MFC0, MFC0, INFO1(NONE))
#define MTC0     RSP_BUILD_OP(MTC0, MTC0, INFO1(NEEDRT))
#define CFC2     RSP_BUILD_OP(CFC2, CFC2, INFO1(NONE))
#define CTC2     RSP_BUILD_OP(CTC2, CTC2, INFO1(NEEDRT))
#define MFC2     RSP_BUILD_OP(MFC2, MFC2, INFO1(NONE))
#define MTC2     RSP_BUILD_OP(MTC2, MTC2, INFO1(NEEDRT))
#define VABS     RSP_BUILD_OP(VABS, VABS, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VADD     RSP_BUILD_OP(VADD, VADD, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VADDC    RSP_BUILD_OP(VADDC, VADDC, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VAND     RSP_BUILD_OP(VAND, VAND_VNAND, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VCH      RSP_BUILD_OP(VCH, VCH, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VCL      RSP_BUILD_OP(VCL, VCL, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VCR      RSP_BUILD_OP(VCR, VCR, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VEQ      RSP_BUILD_OP(VEQ, VEQ_VGE_VLT_VNE, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VGE      RSP_BUILD_OP(VGE, VEQ_VGE_VLT_VNE, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VLT      RSP_BUILD_OP(VLT, VEQ_VGE_VLT_VNE, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VINVALID RSP_BUILD_OP(VINVALID, VINVALID, INFO1(VECTOR))
#define VMACF    RSP_BUILD_OP(VMACF, VMACF_VMACU, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VMACQ    RSP_BUILD_OP(VMACQ, VINVALID, INFO1(VECTOR))
#define VMACU    RSP_BUILD_OP(VMACU, VMACF_VMACU, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VMADH    RSP_BUILD_OP(VMADH, VMADH_VMUDH, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VMADL    RSP_BUILD_OP(VMADL, VMADL_VMUDL, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VMADM    RSP_BUILD_OP(VMADM, VMADM_VMUDM, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VMADN    RSP_BUILD_OP(VMADN, VMADN_VMUDN, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VMOV     RSP_BUILD_OP(VMOV, VMOV, INFO2(VECTOR, NEEDVT))
#define VMRG     RSP_BUILD_OP(VMRG, VMRG, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VMUDH    RSP_BUILD_OP(VMUDH, VMADH_VMUDH, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VMUDL    RSP_BUILD_OP(VMUDL, VMADL_VMUDL, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VMUDM    RSP_BUILD_OP(VMUDM, VMADM_VMUDM, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VMUDN    RSP_BUILD_OP(VMUDN, VMADN_VMUDN, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VMULF    RSP_BUILD_OP(VMULF, VMULF_VMULU, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VMULQ    RSP_BUILD_OP(VMULQ, VINVALID, INFO1(VECTOR))
#define VMULU    RSP_BUILD_OP(VMULU, VMULF_VMULU, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VNAND    RSP_BUILD_OP(VNAND, VAND_VNAND, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VNE      RSP_BUILD_OP(VNE, VEQ_VGE_VLT_VNE, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VNOP     RSP_BUILD_OP(VNOP, VNOP, INFO1(VECTOR))
#define VNOR     RSP_BUILD_OP(VNOR, VOR_VNOR, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VNULL    RSP_BUILD_OP(VNULL, VNOP, INFO1(VECTOR))
#define VNXOR    RSP_BUILD_OP(VNXOR, VXOR_VNXOR, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VOR      RSP_BUILD_OP(VOR, VOR_VNOR, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VRCP     RSP_BUILD_OP(VRCP, VRCP_VRSQ, INFO2(VECTOR, NEEDVT))
#define VRCPH    RSP_BUILD_OP(VRCPH, VRCPH_VRSQH, INFO2(VECTOR, NEEDVT))
#define VRCPL    RSP_BUILD_OP(VRCPL, VRCP_VRSQ, INFO2(VECTOR, NEEDVT))
#define VRNDN    RSP_BUILD_OP(VRNDN, VINVALID, INFO1(VECTOR))
#define VRNDP    RSP_BUILD_OP(VRNDP, VINVALID, INFO1(VECTOR))
#define VRSQ     RSP_BUILD_OP(VRSQ, VRCP_VRSQ, INFO2(VECTOR, NEEDVT))
#define VRSQH    RSP_BUILD_OP(VRSQH, VRCPH_VRSQH, INFO2(VECTOR, NEEDVT))
#define VRSQL    RSP_BUILD_OP(VRSQL, VRCP_VRSQ, INFO2(VECTOR, NEEDVT))
#define VSAR     RSP_BUILD_OP(VSAR, VSAR, INFO1(VECTOR))
#define VSUB     RSP_BUILD_OP(VSUB, VSUB, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VSUBC    RSP_BUILD_OP(VSUBC, VSUBC, INFO3(VECTOR, NEEDVS, NEEDVT))
#define VXOR     RSP_BUILD_OP(VXOR, VXOR_VNXOR, INFO3(VECTOR, NEEDVS, NEEDVT))
#define LBV      RSP_BUILD_OP(LBV, LBDLSV_SBDLSV, INFO3(NEEDRS, NEEDVT, LOAD))
#define LDV      RSP_BUILD_OP(LDV, LBDLSV_SBDLSV, INFO3(NEEDRS, NEEDVT, LOAD))
#define LFV      RSP_BUILD_OP(LFV, INVALID, INFO1(NONE))
#define LHV      RSP_BUILD_OP(LHV, INVALID, INFO1(NONE))
#define LLV      RSP_BUILD_OP(LLV, LBDLSV_SBDLSV, INFO3(NEEDRS, NEEDVT, LOAD))
#define LPV      RSP_BUILD_OP(LPV, LFHPUV_SFHPUV, INFO3(NEEDRS, NEEDVT, LOAD))
#define LQV      RSP_BUILD_OP(LQV, LQRV_SQRV, INFO3(NEEDRS, NEEDVT, LOAD))
#define LRV      RSP_BUILD_OP(LRV, LQRV_SQRV, INFO3(NEEDRS, NEEDVT, LOAD))
#define LSV      RSP_BUILD_OP(LSV, LBDLSV_SBDLSV, INFO3(NEEDRS, NEEDVT, LOAD))
#define LTV      RSP_BUILD_OP(LTV, LTV_STV, INFO3(NEEDRS, NEEDVT, LOAD))
#define LUV      RSP_BUILD_OP(LUV, LFHPUV_SFHPUV, INFO3(NEEDRS, NEEDVT, LOAD))
#define SBV      RSP_BUILD_OP(SBV, LBDLSV_SBDLSV, INFO3(NEEDRS, NEEDVT, STORE))
#define SDV      RSP_BUILD_OP(SDV, LBDLSV_SBDLSV, INFO3(NEEDRS, NEEDVT, STORE))
#define SFV      RSP_BUILD_OP(SFV, INVALID, INFO1(NONE))
#define SHV      RSP_BUILD_OP(SHV, INVALID, INFO1(NONE))
#define SLV      RSP_BUILD_OP(SLV, LBDLSV_SBDLSV, INFO3(NEEDRS, NEEDVT, STORE))
#define SPV      RSP_BUILD_OP(SPV, LFHPUV_SFHPUV, INFO3(NEEDRS, NEEDVT, STORE))
#define SQV      RSP_BUILD_OP(SQV, LQRV_SQRV, INFO3(NEEDRS, NEEDVT, STORE))
#define SRV      RSP_BUILD_OP(SRV, LQRV_SQRV, INFO3(NEEDRS, NEEDVT, STORE))
#define SSV      RSP_BUILD_OP(SSV, LBDLSV_SBDLSV, INFO3(NEEDRS, NEEDVT, STORE))
#define STV      RSP_BUILD_OP(STV, LTV_STV, INFO3(NEEDRS, NEEDVT, LOAD))
#define SUV      RSP_BUILD_OP(SUV, LFHPUV_SFHPUV, INFO3(NEEDRS, NEEDVT, STORE))
#define SWV      RSP_BUILD_OP(SWV, INVALID, INFO1(NONE))

typedef __m128i rsp_vect_t;

extern const uint16_t rsp_reciprocal_rom[1024];
extern const uint16_t rsp_vlogic_mask[2][8];
extern const uint16_t vdiv_mask_table[8][8];

struct rsp;

enum rsp_cp0_register
{
    RSP_CP0_REGISTER_DMA_CACHE        = 32,
    RSP_CP0_REGISTER_DMA_DRAM         = 33,
    RSP_CP0_REGISTER_DMA_READ_LENGTH  = 34,
    RSP_CP0_REGISTER_DMA_WRITE_LENGTH = 35,
    RSP_CP0_REGISTER_SP_STATUS        = 36,
    RSP_CP0_REGISTER_DMA_FULL         = 37,
    RSP_CP0_REGISTER_DMA_BUSY         = 38,
    RSP_CP0_REGISTER_SP_RESERVED      = 39,
    RSP_CP0_REGISTER_CMD_START        = 40,
    RSP_CP0_REGISTER_CMD_END          = 41,
    RSP_CP0_REGISTER_CMD_CURRENT      = 42,
    RSP_CP0_REGISTER_CMD_STATUS       = 43,
    RSP_CP0_REGISTER_CMD_CLOCK        = 44,
    RSP_CP0_REGISTER_CMD_BUSY         = 45,
    RSP_CP0_REGISTER_CMD_PIPE_BUSY    = 46,
    RSP_CP0_REGISTER_CMD_TMEM_BUSY    = 47,
};
enum rsp_flags_t
{
    RSP_VCO = 0,
    RSP_VCC = 1,
    RSP_VCE = 2
};
enum rsp_acc_t
{
    RSP_ACC_LO = 16,
    RSP_ACC_MD = 8,
    RSP_ACC_HI = 0,
};
union aligned_rsp_3vect_t
{
    rsp_vect_t __align[3];
    uint16_t   e[24];
};
union aligned_rsp_2vect_t
{
    rsp_vect_t __align[2];
    uint16_t   e[16];
};
union aligned_rsp_1vect_t
{
    rsp_vect_t __align;
    uint16_t   e[8];
};
struct rsp_opcode_escape
{
    uint16_t offset;
    uint8_t  shift, mask;
};
struct rsp_cp2
{
    union aligned_rsp_1vect_t regs[32];
    union aligned_rsp_2vect_t flags[3];
    union aligned_rsp_3vect_t acc;
    int16_t                   div_out;
    int16_t                   div_in;
    char                      dp_flag;
};
struct rsp_opcode
{
    uint32_t id;
    uint32_t flags;
};
enum rsp_opcode_id
{
    RSP_OPCODE_INVALID,
    RSP_OPCODE_ADDU,
    RSP_OPCODE_ADDIU,
    RSP_OPCODE_AND,
    RSP_OPCODE_ANDI,
    RSP_OPCODE_BEQ,
    RSP_OPCODE_BGEZ,
    RSP_OPCODE_BGEZAL,
    RSP_OPCODE_BGTZ,
    RSP_OPCODE_BLEZ,
    RSP_OPCODE_BLTZ,
    RSP_OPCODE_BLTZAL,
    RSP_OPCODE_BNE,
    RSP_OPCODE_BREAK,
    RSP_OPCODE_CFC2,
    RSP_OPCODE_CTC2,
    RSP_OPCODE_J,
    RSP_OPCODE_JAL,
    RSP_OPCODE_JALR,
    RSP_OPCODE_JR,
    RSP_OPCODE_LB,
    RSP_OPCODE_LBU,
    RSP_OPCODE_LBV,
    RSP_OPCODE_LDV,
    RSP_OPCODE_LFV,
    RSP_OPCODE_LH,
    RSP_OPCODE_LHU,
    RSP_OPCODE_LHV,
    RSP_OPCODE_LLV,
    RSP_OPCODE_LPV,
    RSP_OPCODE_LQV,
    RSP_OPCODE_LRV,
    RSP_OPCODE_LSV,
    RSP_OPCODE_LTV,
    RSP_OPCODE_LUI,
    RSP_OPCODE_LUV,
    RSP_OPCODE_LW,
    RSP_OPCODE_MFC0,
    RSP_OPCODE_MFC2,
    RSP_OPCODE_MTC0,
    RSP_OPCODE_MTC2,
    RSP_OPCODE_NOP,
    RSP_OPCODE_NOR,
    RSP_OPCODE_OR,
    RSP_OPCODE_ORI,
    RSP_OPCODE_SB,
    RSP_OPCODE_SBV,
    RSP_OPCODE_SDV,
    RSP_OPCODE_SFV,
    RSP_OPCODE_SH,
    RSP_OPCODE_SHV,
    RSP_OPCODE_SLL,
    RSP_OPCODE_SLLV,
    RSP_OPCODE_SLT,
    RSP_OPCODE_SLTI,
    RSP_OPCODE_SLTIU,
    RSP_OPCODE_SLTU,
    RSP_OPCODE_SLV,
    RSP_OPCODE_SPV,
    RSP_OPCODE_SQV,
    RSP_OPCODE_SRA,
    RSP_OPCODE_SRAV,
    RSP_OPCODE_SRL,
    RSP_OPCODE_SRLV,
    RSP_OPCODE_SRV,
    RSP_OPCODE_SSV,
    RSP_OPCODE_STV,
    RSP_OPCODE_SUBU,
    RSP_OPCODE_SUV,
    RSP_OPCODE_SW,
    RSP_OPCODE_SWV,
    RSP_OPCODE_XOR,
    RSP_OPCODE_XORI,
    NUM_RSP_OPCODES
};
enum rsp_vector_opcode_id
{
    RSP_OPCODE_VINVALID,
    RSP_OPCODE_VABS,
    RSP_OPCODE_VADD,
    RSP_OPCODE_VADDC,
    RSP_OPCODE_VAND,
    RSP_OPCODE_VCH,
    RSP_OPCODE_VCL,
    RSP_OPCODE_VCR,
    RSP_OPCODE_VEQ,
    RSP_OPCODE_VGE,
    RSP_OPCODE_VLT,
    RSP_OPCODE_VMACF,
    RSP_OPCODE_VMACQ,
    RSP_OPCODE_VMACU,
    RSP_OPCODE_VMADH,
    RSP_OPCODE_VMADL,
    RSP_OPCODE_VMADM,
    RSP_OPCODE_VMADN,
    RSP_OPCODE_VMOV,
    RSP_OPCODE_VMRG,
    RSP_OPCODE_VMUDH,
    RSP_OPCODE_VMUDL,
    RSP_OPCODE_VMUDM,
    RSP_OPCODE_VMUDN,
    RSP_OPCODE_VMULF,
    RSP_OPCODE_VMULQ,
    RSP_OPCODE_VMULU,
    RSP_OPCODE_VNAND,
    RSP_OPCODE_VNE,
    RSP_OPCODE_VNOP,
    RSP_OPCODE_VNOR,
    RSP_OPCODE_VNULL,
    RSP_OPCODE_VNXOR,
    RSP_OPCODE_VOR,
    RSP_OPCODE_VRCP,
    RSP_OPCODE_VRCPH,
    RSP_OPCODE_VRCPL,
    RSP_OPCODE_VRNDN,
    RSP_OPCODE_VRNDP,
    RSP_OPCODE_VRSQ,
    RSP_OPCODE_VRSQH,
    RSP_OPCODE_VRSQL,
    RSP_OPCODE_VSAR,
    RSP_OPCODE_VSUB,
    RSP_OPCODE_VSUBC,
    RSP_OPCODE_VXOR,
    NUM_RSP_VECTOR_OPCODES
};
struct dynarec_slab
{
    size_t   size;
    uint8_t *ptr;
};
enum rsp_register
{
    RSP_REGISTER_R0,
    RSP_REGISTER_AT,
    RSP_REGISTER_V0,
    RSP_REGISTER_V1,
    RSP_REGISTER_A0,
    RSP_REGISTER_A1,
    RSP_REGISTER_A2,
    RSP_REGISTER_A3,
    RSP_REGISTER_T0,
    RSP_REGISTER_T1,
    RSP_REGISTER_T2,
    RSP_REGISTER_T3,
    RSP_REGISTER_T4,
    RSP_REGISTER_R5,
    RSP_REGISTER_T6,
    RSP_REGISTER_T7,
    RSP_REGISTER_S0,
    RSP_REGISTER_S1,
    RSP_REGISTER_S2,
    RSP_REGISTER_S3,
    RSP_REGISTER_S4,
    RSP_REGISTER_S5,
    RSP_REGISTER_S6,
    RSP_REGISTER_S7,
    RSP_REGISTER_T8,
    RSP_REGISTER_T9,
    RSP_REGISTER_K0,
    RSP_REGISTER_K1,
    RSP_REGISTER_GP,
    RSP_REGISTER_SP,
    RSP_REGISTER_FP,
    RSP_REGISTER_RA,
    // CP0 registers.
    RSP_REGISTER_CP0_0,
    RSP_REGISTER_CP0_1,
    RSP_REGISTER_CP0_2,
    RSP_REGISTER_CP0_3,
    RSP_REGISTER_CP0_4,
    RSP_REGISTER_CP0_5,
    RSP_REGISTER_CP0_6,
    RSP_REGISTER_CP0_7,
    // Miscellanious registers.
    NUM_RSP_REGISTERS
};
enum sp_register
{
    SP_MEM_ADDR_REG,
    SP_DRAM_ADDR_REG,
    SP_RD_LEN_REG,
    SP_WR_LEN_REG,
    SP_STATUS_REG,
    SP_DMA_FULL_REG,
    SP_DMA_BUSY_REG,
    SP_SEMAPHORE_REG,
    CMD_START,
    CMD_END,
    CMD_CURRENT,
    CMD_STATUS,
    CMD_CLOCK,
    CMD_BUSY,
    CMD_PIPE_BUSY,
    CMD_TMEM_BUSY,
    SP_PC_REG,
    SP_IBIST_REG,

    NUM_SP_REGISTERS,
    SP_REGISTER_OFFSET = RSP_REGISTER_CP0_0
};
enum rsp_mem_request_type
{
    RSP_MEM_REQUEST_NONE,
    RSP_MEM_REQUEST_INT_MEM,
    RSP_MEM_REQUEST_VECTOR,
    RSP_MEM_REQUEST_FOURTH,
    RSP_MEM_REQUEST_HALF,
    RSP_MEM_REQUEST_PACK,
    RSP_MEM_REQUEST_QUAD,
    RSP_MEM_REQUEST_REST,
    RSP_MEM_REQUEST_UPACK,
    RSP_MEM_REQUEST_TRANSPOSE,
};
struct rsp_int_mem_packet
{
    uint32_t data;
    uint32_t rdqm;
    uint32_t wdqm;
    unsigned rshift;
};
struct rsp_transpose_mem_packet
{
    void (*transpose_func)(struct rsp *rsp, uint32_t addr, unsigned element, unsigned vt);
    unsigned element;
    unsigned vt;
};
struct rsp_vect_mem_packet
{
    union aligned_rsp_1vect_t vdqm;
    void (*vldst_func)(struct rsp *rsp, uint32_t addr, unsigned element, uint16_t *regp, rsp_vect_t reg,
                       rsp_vect_t dqm);
    unsigned element;
    unsigned dest;
};
union rsp_mem_packet
{
    struct rsp_int_mem_packet       p_int;
    struct rsp_transpose_mem_packet p_transpose;
    struct rsp_vect_mem_packet      p_vect;
};
struct rsp_mem_request
{
    uint32_t                  addr;
    enum rsp_mem_request_type type;
    union rsp_mem_packet      packet;
};
struct rsp_latch
{
    uint32_t pc;
};
struct rsp_result
{
    uint32_t result;
    unsigned dest;
};
struct rsp_ifrd_latch
{
    struct rsp_latch  common;
    struct rsp_opcode opcode;
    uint32_t          pc, iw;
};
struct rsp_rdex_latch
{
    struct rsp_latch  common;
    struct rsp_opcode opcode;
    uint32_t          iw;
};
struct rsp_exdf_latch
{
    struct rsp_latch       common;
    struct rsp_result      result;
    struct rsp_mem_request request;
};
struct rsp_dfwb_latch
{
    struct rsp_latch  common;
    struct rsp_result result;
};
struct rsp_pipeline
{
    struct rsp_dfwb_latch dfwb_latch;
    struct rsp_exdf_latch exdf_latch;
    struct rsp_rdex_latch rdex_latch;
    struct rsp_ifrd_latch ifrd_latch;
};
struct rsp
{
    struct bus_controller *bus;
    struct rsp_pipeline    pipeline;
    struct rsp_cp2         cp2;
    uint32_t               regs[NUM_RSP_REGISTERS];
    uint8_t                mem[0x2000];
    // Instead of redecoding the instructions (there's only 256 words)
    // every cycle, we maintain a 256-word decoded instruction cache.
    struct rsp_opcode opcode_cache[0x1000 / 4];
    // TODO: Only for IA32/x86_64 SSE2; sloppy?
    struct dynarec_slab vload_dynarec;
    struct dynarec_slab vstore_dynarec;
};

typedef void (*rsp_function)(struct rsp *, uint32_t, uint32_t, uint32_t);
typedef rsp_vect_t (*rsp_vector_function)(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs,
                                          rsp_vect_t zero);

extern const rsp_function        rsp_function_table[NUM_RSP_OPCODES];
extern const char               *rsp_opcode_mnemonics[NUM_RSP_OPCODES];
extern const rsp_vector_function rsp_vector_function_table[NUM_RSP_VECTOR_OPCODES];
extern const char               *rsp_vector_opcode_mnemonics[NUM_RSP_VECTOR_OPCODES];

void *alloc_dynarec_slab(struct dynarec_slab *slab, size_t size);
void  free_dynarec_slab(struct dynarec_slab *slab);

void    arch_rsp_destroy(struct rsp *rsp);
int     arch_rsp_init(struct rsp *rsp);
__m128i rsp_vect_load_and_shuffle_operand(const uint16_t *src, unsigned element);

void rsp_set_flags(uint16_t *flags, uint16_t rt);
void rsp_vload_group1(struct rsp *rsp, uint32_t addr, unsigned element, uint16_t *regp, __m128i reg, __m128i dqm);
void rsp_vload_group2(struct rsp *rsp, uint32_t addr, unsigned element, uint16_t *regp, __m128i reg, __m128i dqm);
void rsp_vload_group4(struct rsp *rsp, uint32_t addr, unsigned element, uint16_t *regp, rsp_vect_t reg, rsp_vect_t dqm);
void rsp_vstore_group1(struct rsp *rsp, uint32_t addr, unsigned element, uint16_t *regp, __m128i reg, __m128i dqm);
void rsp_vstore_group2(struct rsp *rsp, uint32_t addr, unsigned element, uint16_t *regp, __m128i reg, __m128i dqm);
void rsp_vstore_group4(struct rsp *rsp, uint32_t addr, unsigned element, uint16_t *regp, rsp_vect_t reg,
                       rsp_vect_t dqm);

void rsp_ltv(struct rsp *rsp, uint32_t addr, unsigned vt, unsigned element);
void rsp_stv(struct rsp *rsp, uint32_t addr, unsigned vt, unsigned element);

__m128i rsp_vdivh(struct rsp *rsp, unsigned src, unsigned e, unsigned dest, unsigned de);
__m128i rsp_vmov(struct rsp *rsp, unsigned src, unsigned e, unsigned dest, rsp_vect_t vt_shuffle);
__m128i rsp_vrcp_vrsq(struct rsp *rsp, uint32_t iw, int dp, unsigned src, unsigned e, unsigned dest, unsigned de);

void     RSP_MFC0(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt);
void     RSP_MTC0(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt);
uint32_t rsp_read_cp0_reg(struct rsp *rsp, unsigned src);
void     rsp_write_cp0_reg(struct rsp *rsp, unsigned dest, uint32_t rt);
void     rsp_status_write(struct rsp *rsp, uint32_t rt);
void     rsp_cp0_init(struct rsp *rsp);

void RSP_CFC2(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt);
void RSP_CTC2(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt);
void RSP_MFC2(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt);
void RSP_MTC2(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt);
void rsp_cp2_init(struct rsp *rsp);
void RSP_INVALID(struct rsp *, uint32_t, uint32_t, uint32_t);

rsp_vect_t RSP_VINVALID(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero);

void rsp_pipeline_init(struct rsp_pipeline *pipeline);
int  rsp_init(struct rsp *rsp, struct bus_controller *bus);
void rsp_late_init(struct rsp *rsp);
void rsp_destroy(struct rsp *rsp);
void rsp_cycle_(struct rsp *rsp);
void rsp_dma_read(struct rsp *rsp);
void rsp_dma_write(struct rsp *rsp);

int read_sp_mem(void *opaque, uint32_t address, uint32_t *word);
int read_sp_regs(void *opaque, uint32_t address, uint32_t *word);
int read_sp_regs2(void *opaque, uint32_t address, uint32_t *word);
int write_sp_mem(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);
int write_sp_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);
int write_sp_regs2(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);

const struct rsp_opcode *rsp_decode_instruction(uint32_t);

static inline __m128i rsp_vect_load_unshuffled_operand(const uint16_t *src)
{
    return _mm_load_si128((__m128i *)src);
}
static inline void rsp_vect_write_operand(uint16_t *dest, __m128i src)
{
    _mm_store_si128((__m128i *)dest, src);
}
static inline __m128i read_acc_lo(const uint16_t *acc)
{
    return rsp_vect_load_unshuffled_operand(acc + 16);
}
static inline __m128i read_acc_md(const uint16_t *acc)
{
    return rsp_vect_load_unshuffled_operand(acc + 8);
}
static inline __m128i read_acc_hi(const uint16_t *acc)
{
    return rsp_vect_load_unshuffled_operand(acc);
}
static inline __m128i read_vcc_lo(const uint16_t *vcc)
{
    return rsp_vect_load_unshuffled_operand(vcc + 8);
}
static inline __m128i read_vcc_hi(const uint16_t *vcc)
{
    return rsp_vect_load_unshuffled_operand(vcc);
}
static inline __m128i read_vco_lo(const uint16_t *vco)
{
    return rsp_vect_load_unshuffled_operand(vco + 8);
}
static inline __m128i read_vco_hi(const uint16_t *vco)
{
    return rsp_vect_load_unshuffled_operand(vco);
}
static inline __m128i read_vce(const uint16_t *vce)
{
    return rsp_vect_load_unshuffled_operand(vce + 8);
}
static inline void write_acc_lo(uint16_t *acc, __m128i acc_lo)
{
    rsp_vect_write_operand(acc + 16, acc_lo);
}
static inline void write_acc_md(uint16_t *acc, __m128i acc_md)
{
    rsp_vect_write_operand(acc + 8, acc_md);
}
static inline void write_acc_hi(uint16_t *acc, __m128i acc_hi)
{
    rsp_vect_write_operand(acc, acc_hi);
}
static inline void write_vcc_lo(uint16_t *vcc, __m128i vcc_lo)
{
    rsp_vect_write_operand(vcc + 8, vcc_lo);
}
static inline void write_vcc_hi(uint16_t *vcc, __m128i vcc_hi)
{
    rsp_vect_write_operand(vcc, vcc_hi);
}
static inline void write_vco_lo(uint16_t *vco, __m128i vco_lo)
{
    rsp_vect_write_operand(vco + 8, vco_lo);
}
static inline void write_vco_hi(uint16_t *vco, __m128i vco_hi)
{
    rsp_vect_write_operand(vco, vco_hi);
}
static inline void write_vce(uint16_t *vce, __m128i vce_r)
{
    rsp_vect_write_operand(vce + 8, vce_r);
}
static inline int16_t rsp_get_flags(const uint16_t *flags)
{
    return (int16_t)_mm_movemask_epi8(
        _mm_packs_epi16(_mm_load_si128((__m128i *)(flags + 8)), _mm_load_si128((__m128i *)(flags + 0))));
}
static inline __m128i rsp_vzero(void)
{
    return _mm_setzero_si128();
}
static inline __m128i rsp_sclamp_acc_tomd(__m128i acc_md, __m128i acc_hi)
{
    __m128i l = _mm_unpacklo_epi16(acc_md, acc_hi);
    __m128i h = _mm_unpackhi_epi16(acc_md, acc_hi);
    return _mm_packs_epi32(l, h);
}
static inline __m128i rsp_uclamp_acc(__m128i val, __m128i acc_md, __m128i acc_hi, __m128i zero)
{
    __m128i clamp_mask, clamped_val;
    __m128i hi_sign_check, md_sign_check;
    __m128i md_negative, hi_negative;

    hi_negative = _mm_srai_epi16(acc_hi, 15);
    md_negative = _mm_srai_epi16(acc_md, 15);

    hi_sign_check = _mm_cmpeq_epi16(hi_negative, acc_hi);
    md_sign_check = _mm_cmpeq_epi16(hi_negative, md_negative);
    clamp_mask    = _mm_and_si128(md_sign_check, hi_sign_check);

    clamped_val = _mm_cmpeq_epi16(hi_negative, zero);

#ifndef __SSE4_1__
    md_sign_check = _mm_and_si128(clamp_mask, val);
    hi_sign_check = _mm_andnot_si128(clamp_mask, clamped_val);
    return _mm_or_si128(hi_sign_check, md_sign_check);
#else
    return _mm_blendv_epi8(clamped_val, val, clamp_mask);
#endif
}
static inline __m128i rsp_vabs(__m128i vs, __m128i vt, __m128i zero, __m128i *acc_lo)
{
    __m128i vs_zero = _mm_cmpeq_epi16(vs, zero);
    __m128i sign_lt = _mm_srai_epi16(vs, 15);
    __m128i vd      = _mm_andnot_si128(vs_zero, vt);

    vd      = _mm_xor_si128(vd, sign_lt);
    *acc_lo = _mm_sub_epi16(vd, sign_lt);
    return _mm_subs_epi16(vd, sign_lt);
}
static inline __m128i rsp_vadd(__m128i vs, __m128i vt, __m128i carry, __m128i *acc_lo)
{
    __m128i vd, minimum, maximum;

    vd      = _mm_add_epi16(vs, vt);
    *acc_lo = _mm_sub_epi16(vd, carry);

    minimum = _mm_min_epi16(vs, vt);
    maximum = _mm_max_epi16(vs, vt);
    minimum = _mm_subs_epi16(minimum, carry);
    return _mm_adds_epi16(minimum, maximum);
}
static inline __m128i rsp_vaddc(__m128i vs, __m128i vt, __m128i zero, __m128i *sn)
{
    __m128i sat_sum, unsat_sum;
    sat_sum   = _mm_adds_epu16(vs, vt);
    unsat_sum = _mm_add_epi16(vs, vt);
    *sn       = _mm_cmpeq_epi16(sat_sum, unsat_sum);
    *sn       = _mm_cmpeq_epi16(*sn, zero);
    return unsat_sum;
}
static inline __m128i rsp_vand_vnand(uint32_t iw, __m128i vs, __m128i vt)
{
    __m128i vmask = _mm_load_si128((__m128i *)rsp_vlogic_mask[iw & 0x1]);
    __m128i vd    = _mm_and_si128(vs, vt);
    return _mm_xor_si128(vd, vmask);
}
static inline __m128i rsp_vch(__m128i vs, __m128i vt, __m128i zero, __m128i *ge, __m128i *le, __m128i *eq,
                              __m128i *sign, __m128i *vce)
{
    __m128i sign_negvt, vt_neg;
    __m128i diff, diff_zero, diff_sel_mask;
    __m128i diff_gez, diff_lez;

    *sign = _mm_xor_si128(vs, vt);
    *sign = _mm_cmplt_epi16(*sign, zero);

    sign_negvt = _mm_xor_si128(vt, *sign);
    sign_negvt = _mm_sub_epi16(sign_negvt, *sign);

    diff      = _mm_sub_epi16(vs, sign_negvt);
    diff_zero = _mm_cmpeq_epi16(diff, zero);

    vt_neg   = _mm_cmplt_epi16(vt, zero);
    diff_lez = _mm_cmpgt_epi16(diff, zero);
    diff_gez = _mm_or_si128(diff_lez, diff_zero);
    diff_lez = _mm_cmpeq_epi16(zero, diff_lez);
#ifdef __SSE4_1__
    *ge = _mm_blendv_epi8(diff_gez, vt_neg, *sign);
    *le = _mm_blendv_epi8(vt_neg, diff_lez, *sign);
#else
    *ge           = _mm_and_si128(*sign, vt_neg);
    diff_gez      = _mm_andnot_si128(*sign, diff_gez);
    *ge           = _mm_or_si128(*ge, diff_gez);
    *le           = _mm_and_si128(*sign, diff_lez);
    diff_lez      = _mm_andnot_si128(*sign, vt_neg);
    *le           = _mm_or_si128(*le, diff_lez);
#endif

    *vce = _mm_cmpeq_epi16(diff, *sign);
    *vce = _mm_and_si128(*vce, *sign);

    *eq = _mm_or_si128(diff_zero, *vce);
    *eq = _mm_cmpeq_epi16(*eq, zero);

#ifdef __SSE4_1__
    diff_sel_mask = _mm_blendv_epi8(*ge, *le, *sign);
    return _mm_blendv_epi8(vs, sign_negvt, diff_sel_mask);
#else
    diff_lez      = _mm_and_si128(*sign, *le);
    diff_gez      = _mm_andnot_si128(*sign, *ge);
    diff_sel_mask = _mm_or_si128(diff_lez, diff_gez);
    diff_lez      = _mm_and_si128(diff_sel_mask, sign_negvt);
    diff_gez      = _mm_andnot_si128(diff_sel_mask, vs);
    return _mm_or_si128(diff_lez, diff_gez);
#endif
}
static inline __m128i rsp_veq_vge_vlt_vne(uint32_t iw, __m128i vs, __m128i vt, __m128i zero, __m128i *le, __m128i eq,
                                          __m128i sign)
{
    __m128i equal = _mm_cmpeq_epi16(vs, vt);

    if (iw & 0x2) {

        if (iw & 0x1) {
            __m128i gt        = _mm_cmpgt_epi16(vs, vt);
            __m128i equalsign = _mm_and_si128(eq, sign);
            equal             = _mm_andnot_si128(equalsign, equal);
            *le               = _mm_or_si128(gt, equal);
        }

        else {
            __m128i nequal = _mm_cmpeq_epi16(equal, zero);
            *le            = _mm_and_si128(eq, equal);
            *le            = _mm_or_si128(*le, nequal);
        }
    }

    else {

        if (iw & 0x1)
            *le = _mm_andnot_si128(eq, equal);

        else {
            __m128i lt = _mm_cmplt_epi16(vs, vt);
            equal      = _mm_and_si128(eq, equal);
            equal      = _mm_and_si128(sign, equal);
            *le        = _mm_or_si128(lt, equal);
        }
    }
#ifdef __SSE4_1__
    return _mm_blendv_epi8(vt, vs, *le);
#else
    vs = _mm_and_si128(*le, vs);
    vt = _mm_andnot_si128(*le, vt);
    return _mm_or_si128(vs, vt);
#endif
}
static inline __m128i rsp_vcl(__m128i vs, __m128i vt, __m128i zero, __m128i *ge, __m128i *le, __m128i eq, __m128i sign,
                              __m128i vce)
{
    __m128i sign_negvt, diff, ncarry, nvce, diff_zero;
    __m128i le_case1, le_case2, le_eq, do_le;
    __m128i ge_eq, do_ge, mux_mask;

    sign_negvt = _mm_xor_si128(vt, sign);
    sign_negvt = _mm_sub_epi16(sign_negvt, sign);

    diff      = _mm_sub_epi16(vs, sign_negvt);
    ncarry    = _mm_adds_epu16(vs, vt);
    ncarry    = _mm_cmpeq_epi16(diff, ncarry);
    nvce      = _mm_cmpeq_epi16(vce, zero);
    diff_zero = _mm_cmpeq_epi16(diff, zero);

    le_case1 = _mm_and_si128(diff_zero, ncarry);
    le_case1 = _mm_and_si128(nvce, le_case1);
    le_case2 = _mm_or_si128(diff_zero, ncarry);
    le_case2 = _mm_and_si128(vce, le_case2);
    le_eq    = _mm_or_si128(le_case1, le_case2);

    ge_eq = _mm_subs_epu16(vt, vs);
    ge_eq = _mm_cmpeq_epi16(ge_eq, zero);

    do_le = _mm_andnot_si128(eq, sign);
#ifdef __SSE4_1__
    *le = _mm_blendv_epi8(*le, le_eq, do_le);
#else
    le_eq      = _mm_and_si128(do_le, le_eq);
    *le        = _mm_andnot_si128(do_le, *le);
    *le        = _mm_or_si128(le_eq, *le);
#endif
    do_ge = _mm_or_si128(sign, eq);
#ifdef __SSE4_1__
    *ge = _mm_blendv_epi8(ge_eq, *ge, do_ge);
#else
    *ge        = _mm_and_si128(do_ge, *ge);
    ge_eq      = _mm_andnot_si128(do_ge, ge_eq);
    *ge        = _mm_or_si128(ge_eq, *ge);
#endif

#ifdef __SSE4_1__
    mux_mask = _mm_blendv_epi8(*ge, *le, sign);
#else
    do_le      = _mm_and_si128(sign, *le);
    do_ge      = _mm_andnot_si128(sign, *ge);
    mux_mask   = _mm_or_si128(do_le, do_ge);
#endif
#ifdef __SSE4_1__
    return _mm_blendv_epi8(vs, sign_negvt, mux_mask);
#else
    sign_negvt = _mm_and_si128(mux_mask, sign_negvt);
    vs         = _mm_andnot_si128(mux_mask, vs);
    return _mm_or_si128(sign_negvt, vs);
#endif
}
static inline __m128i rsp_vcr(__m128i vs, __m128i vt, __m128i zero, __m128i *ge, __m128i *le)
{
    __m128i diff_sel_mask, diff_gez, diff_lez;
    __m128i sign, sign_notvt;

    sign = _mm_xor_si128(vs, vt);
    sign = _mm_srai_epi16(sign, 15);

    diff_lez = _mm_and_si128(vs, sign);
    diff_lez = _mm_add_epi16(diff_lez, vt);
    *le      = _mm_srai_epi16(diff_lez, 15);

    diff_gez = _mm_or_si128(vs, sign);
    diff_gez = _mm_min_epi16(diff_gez, vt);
    *ge      = _mm_cmpeq_epi16(diff_gez, vt);

    sign_notvt = _mm_xor_si128(vt, sign);

#ifdef __SSE4_1__
    diff_sel_mask = _mm_blendv_epi8(*ge, *le, sign);
    return _mm_blendv_epi8(vs, sign_notvt, diff_sel_mask);
#else
    diff_sel_mask = _mm_sub_epi16(*le, *ge);
    diff_sel_mask = _mm_and_si128(diff_sel_mask, sign);
    diff_sel_mask = _mm_add_epi16(diff_sel_mask, *ge);
    zero          = _mm_sub_epi16(sign_notvt, vs);
    zero          = _mm_and_si128(zero, diff_sel_mask);
    return _mm_add_epi16(zero, vs);
#endif
}
static inline __m128i rsp_vmacf_vmacu(uint32_t iw, __m128i vs, __m128i vt, __m128i zero, __m128i *acc_lo,
                                      __m128i *acc_md, __m128i *acc_hi)
{
    __m128i overflow_hi_mask, overflow_md_mask;
    __m128i lo, md, hi, carry, overflow_mask;

    lo = _mm_mullo_epi16(vs, vt);
    hi = _mm_mulhi_epi16(vs, vt);

    md    = _mm_slli_epi16(hi, 1);
    carry = _mm_srli_epi16(lo, 15);
    hi    = _mm_srai_epi16(hi, 15);
    md    = _mm_or_si128(md, carry);
    lo    = _mm_slli_epi16(lo, 1);

    overflow_mask = _mm_adds_epu16(*acc_lo, lo);
    *acc_lo       = _mm_add_epi16(*acc_lo, lo);

    overflow_mask = _mm_cmpeq_epi16(*acc_lo, overflow_mask);
    overflow_mask = _mm_cmpeq_epi16(overflow_mask, zero);

    md    = _mm_sub_epi16(md, overflow_mask);
    carry = _mm_cmpeq_epi16(md, zero);
    carry = _mm_and_si128(carry, overflow_mask);
    hi    = _mm_sub_epi16(hi, carry);

    overflow_mask = _mm_adds_epu16(*acc_md, md);
    *acc_md       = _mm_add_epi16(*acc_md, md);

    overflow_mask = _mm_cmpeq_epi16(*acc_md, overflow_mask);
    overflow_mask = _mm_cmpeq_epi16(overflow_mask, zero);

    *acc_hi = _mm_add_epi16(*acc_hi, hi);
    *acc_hi = _mm_sub_epi16(*acc_hi, overflow_mask);

    if (iw & 0x1) {
        overflow_hi_mask = _mm_srai_epi16(*acc_hi, 15);
        overflow_md_mask = _mm_srai_epi16(*acc_md, 15);
        md               = _mm_or_si128(overflow_md_mask, *acc_md);
        overflow_mask    = _mm_cmpgt_epi16(*acc_hi, zero);
        md               = _mm_andnot_si128(overflow_hi_mask, md);
        return _mm_or_si128(overflow_mask, md);
    }

    else
        return rsp_sclamp_acc_tomd(*acc_md, *acc_hi);
}
static inline __m128i rsp_vmrg(__m128i vs, __m128i vt, __m128i le)
{
#ifdef __SSE4_1__
    return _mm_blendv_epi8(vt, vs, le);
#else
    vs = _mm_and_si128(le, vs);
    vt = _mm_andnot_si128(le, vt);
    return _mm_or_si128(vs, vt);
#endif
}
static inline __m128i rsp_vmulf_vmulu(uint32_t iw, __m128i vs, __m128i vt, __m128i zero, __m128i *acc_lo,
                                      __m128i *acc_md, __m128i *acc_hi)
{
    __m128i lo, hi, round, sign1, sign2, eq, neq, neg;

    lo      = _mm_mullo_epi16(vs, vt);
    round   = _mm_cmpeq_epi16(zero, zero);
    sign1   = _mm_srli_epi16(lo, 15);
    lo      = _mm_add_epi16(lo, lo);
    round   = _mm_slli_epi16(round, 15);
    hi      = _mm_mulhi_epi16(vs, vt);
    sign2   = _mm_srli_epi16(lo, 15);
    *acc_lo = _mm_add_epi16(round, lo);
    sign1   = _mm_add_epi16(sign1, sign2);

    hi  = _mm_slli_epi16(hi, 1);
    neq = eq = _mm_cmpeq_epi16(vs, vt);
    *acc_md  = _mm_add_epi16(hi, sign1);

    neg = _mm_srai_epi16(*acc_md, 15);

    if (iw & 0x1) {
        *acc_hi = _mm_andnot_si128(eq, neg);
        hi      = _mm_or_si128(*acc_md, neg);
        return _mm_andnot_si128(*acc_hi, hi);
    }

    else {
        eq      = _mm_and_si128(eq, neg);
        *acc_hi = _mm_andnot_si128(neq, neg);
        return _mm_add_epi16(*acc_md, eq);
    }
}
static inline __m128i rsp_vmadh_vmudh(uint32_t iw, __m128i vs, __m128i vt, __m128i zero, __m128i *acc_lo,
                                      __m128i *acc_md, __m128i *acc_hi)
{
    __m128i lo, hi, overflow_mask;
    lo = _mm_mullo_epi16(vs, vt);
    hi = _mm_mulhi_epi16(vs, vt);

    if (iw & 0x8) {

        overflow_mask = _mm_adds_epu16(*acc_md, lo);
        *acc_md       = _mm_add_epi16(*acc_md, lo);
        overflow_mask = _mm_cmpeq_epi16(*acc_md, overflow_mask);
        overflow_mask = _mm_cmpeq_epi16(overflow_mask, zero);
        hi            = _mm_sub_epi16(hi, overflow_mask);
        *acc_hi       = _mm_add_epi16(*acc_hi, hi);
    }

    else {
        *acc_lo = zero;
        *acc_md = lo;
        *acc_hi = hi;
    }
    return rsp_sclamp_acc_tomd(*acc_md, *acc_hi);
}
static inline __m128i rsp_vmadl_vmudl(uint32_t iw, __m128i vs, __m128i vt, __m128i zero, __m128i *acc_lo,
                                      __m128i *acc_md, __m128i *acc_hi)
{
    __m128i hi, overflow_mask;
    hi = _mm_mulhi_epu16(vs, vt);

    if (iw & 0x8) {

        overflow_mask = _mm_adds_epu16(*acc_lo, hi);
        *acc_lo       = _mm_add_epi16(*acc_lo, hi);
        overflow_mask = _mm_cmpeq_epi16(*acc_lo, overflow_mask);
        overflow_mask = _mm_cmpeq_epi16(overflow_mask, zero);
        hi            = _mm_sub_epi16(zero, overflow_mask);

        overflow_mask = _mm_adds_epu16(*acc_md, hi);
        *acc_md       = _mm_add_epi16(*acc_md, hi);
        overflow_mask = _mm_cmpeq_epi16(*acc_md, overflow_mask);
        overflow_mask = _mm_cmpeq_epi16(overflow_mask, zero);

        *acc_hi = _mm_sub_epi16(*acc_hi, overflow_mask);
        return rsp_uclamp_acc(*acc_lo, *acc_md, *acc_hi, zero);
    }

    else {
        *acc_lo = hi;
        *acc_md = zero;
        *acc_hi = zero;
        return hi;
    }
}
static inline __m128i rsp_vmadm_vmudm(uint32_t iw, __m128i vs, __m128i vt, __m128i zero, __m128i *acc_lo,
                                      __m128i *acc_md, __m128i *acc_hi)
{
    __m128i lo, hi, sign, overflow_mask;
    lo = _mm_mullo_epi16(vs, vt);
    hi = _mm_mulhi_epu16(vs, vt);

    sign = _mm_srai_epi16(vs, 15);
    vt   = _mm_and_si128(vt, sign);
    hi   = _mm_sub_epi16(hi, vt);

    if (iw & 0x8) {

        overflow_mask = _mm_adds_epu16(*acc_lo, lo);
        *acc_lo       = _mm_add_epi16(*acc_lo, lo);
        overflow_mask = _mm_cmpeq_epi16(*acc_lo, overflow_mask);
        overflow_mask = _mm_cmpeq_epi16(overflow_mask, zero);

        hi = _mm_sub_epi16(hi, overflow_mask);

        overflow_mask = _mm_adds_epu16(*acc_md, hi);
        *acc_md       = _mm_add_epi16(*acc_md, hi);
        overflow_mask = _mm_cmpeq_epi16(*acc_md, overflow_mask);
        overflow_mask = _mm_cmpeq_epi16(overflow_mask, zero);

        *acc_hi = _mm_add_epi16(*acc_hi, _mm_srai_epi16(hi, 15));
        *acc_hi = _mm_sub_epi16(*acc_hi, overflow_mask);
        return rsp_sclamp_acc_tomd(*acc_md, *acc_hi);
    }

    else {
        *acc_lo = lo;
        *acc_md = hi;
        *acc_hi = _mm_srai_epi16(hi, 15);
        return hi;
    }
}
static inline __m128i rsp_vmadn_vmudn(uint32_t iw, __m128i vs, __m128i vt, __m128i zero, __m128i *acc_lo,
                                      __m128i *acc_md, __m128i *acc_hi)
{
    __m128i lo, hi, sign, overflow_mask;
    lo = _mm_mullo_epi16(vs, vt);
    hi = _mm_mulhi_epu16(vs, vt);

    sign = _mm_srai_epi16(vt, 15);
    vs   = _mm_and_si128(vs, sign);
    hi   = _mm_sub_epi16(hi, vs);

    if (iw & 0x8) {

        overflow_mask = _mm_adds_epu16(*acc_lo, lo);
        *acc_lo       = _mm_add_epi16(*acc_lo, lo);
        overflow_mask = _mm_cmpeq_epi16(*acc_lo, overflow_mask);
        overflow_mask = _mm_cmpeq_epi16(overflow_mask, zero);

        hi = _mm_sub_epi16(hi, overflow_mask);

        overflow_mask = _mm_adds_epu16(*acc_md, hi);
        *acc_md       = _mm_add_epi16(*acc_md, hi);
        overflow_mask = _mm_cmpeq_epi16(*acc_md, overflow_mask);
        overflow_mask = _mm_cmpeq_epi16(overflow_mask, zero);

        *acc_hi = _mm_add_epi16(*acc_hi, _mm_srai_epi16(hi, 15));
        *acc_hi = _mm_sub_epi16(*acc_hi, overflow_mask);
        return rsp_uclamp_acc(*acc_lo, *acc_md, *acc_hi, zero);
    }

    else {
        *acc_lo = lo;
        *acc_md = hi;
        *acc_hi = _mm_srai_epi16(hi, 15);
        return lo;
    }
}
static inline __m128i rsp_vor_vnor(uint32_t iw, __m128i vs, __m128i vt)
{
    __m128i vmask = _mm_load_si128((__m128i *)rsp_vlogic_mask[iw & 0x1]);
    __m128i vd    = _mm_or_si128(vs, vt);
    return _mm_xor_si128(vd, vmask);
}
static inline __m128i rsp_vsub(__m128i vs, __m128i vt, __m128i carry, __m128i *acc_lo)
{
    __m128i unsat_diff, sat_diff, overflow, vd;

    unsat_diff = _mm_sub_epi16(vt, carry);
    sat_diff   = _mm_subs_epi16(vt, carry);
    *acc_lo    = _mm_sub_epi16(vs, unsat_diff);
    vd         = _mm_subs_epi16(vs, sat_diff);

    overflow = _mm_cmpgt_epi16(sat_diff, unsat_diff);
    return _mm_adds_epi16(vd, overflow);
}
static inline __m128i rsp_vsubc(__m128i vs, __m128i vt, __m128i zero, __m128i *eq, __m128i *sn)
{
    __m128i equal, sat_udiff, sat_udiff_zero;
    sat_udiff      = _mm_subs_epu16(vs, vt);
    equal          = _mm_cmpeq_epi16(vs, vt);
    sat_udiff_zero = _mm_cmpeq_epi16(sat_udiff, zero);
    *eq            = _mm_cmpeq_epi16(equal, zero);
    *sn            = _mm_andnot_si128(equal, sat_udiff_zero);
    return _mm_sub_epi16(vs, vt);
}
static inline __m128i rsp_vxor_vnxor(uint32_t iw, __m128i vs, __m128i vt)
{
    __m128i vmask = _mm_load_si128((__m128i *)rsp_vlogic_mask[iw & 0x1]);
    __m128i vd    = _mm_xor_si128(vs, vt);
    return _mm_xor_si128(vd, vmask);
}
static inline void rsp_cycle(struct rsp *rsp)
{
    if (unlikely(rsp->regs[RSP_CP0_REGISTER_SP_STATUS] & SP_STATUS_HALT))
        return;
    rsp_cycle_(rsp);
}

#endif
