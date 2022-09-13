#include <stddef.h>
#include "../utils/common.h"

#include "bus.h"
#include "cpu.h"
#include "rdp.h"
#include "rsp.h"

extern vr4300 *g_vr4300;

static void rsp_connect_bus(struct rsp *rsp, struct bus_controller *bus)
{
    rsp->bus = bus;
}
void rsp_destroy(struct rsp *rsp)
{
    arch_rsp_destroy(rsp);
}
int rsp_init(struct rsp *rsp, struct bus_controller *bus)
{
    rsp_connect_bus(rsp, bus);
    rsp_cp0_init(rsp);
    rsp_pipeline_init(&rsp->pipeline);
    return arch_rsp_init(rsp);
}
void rsp_late_init(struct rsp *rsp)
{
    write_acc_lo(rsp->cp2.acc.e, rsp_vzero());
    write_acc_md(rsp->cp2.acc.e, rsp_vzero());
    write_acc_hi(rsp->cp2.acc.e, rsp_vzero());
    write_vcc_lo(rsp->cp2.flags[RSP_VCC].e, rsp_vzero());
    write_vcc_hi(rsp->cp2.flags[RSP_VCC].e, rsp_vzero());
    write_vco_lo(rsp->cp2.flags[RSP_VCO].e, rsp_vzero());
    write_vco_hi(rsp->cp2.flags[RSP_VCO].e, rsp_vzero());
    write_vce(rsp->cp2.flags[RSP_VCE].e, rsp_vzero());
}
void rsp_dma_read(struct rsp *rsp)
{
    uint32_t length = (rsp->regs[RSP_CP0_REGISTER_DMA_READ_LENGTH] & 0xFFF) + 1;
    uint32_t skip   = rsp->regs[RSP_CP0_REGISTER_DMA_READ_LENGTH] >> 20 & 0xFFF;
    unsigned count  = rsp->regs[RSP_CP0_REGISTER_DMA_READ_LENGTH] >> 12 & 0xFF;
    unsigned j, i = 0;
    length = (length + 0x7) & ~0x7;
    rsp->regs[RSP_CP0_REGISTER_DMA_CACHE] &= ~0x7;
    rsp->regs[RSP_CP0_REGISTER_DMA_DRAM] &= ~0x7;
    if (((rsp->regs[RSP_CP0_REGISTER_DMA_CACHE] & 0xFFF) + length) > 0x1000)
        length = 0x1000 - (rsp->regs[RSP_CP0_REGISTER_DMA_CACHE] & 0xFFF);
    do {
        uint32_t source = rsp->regs[RSP_CP0_REGISTER_DMA_DRAM] & 0x7FFFFC;
        uint32_t dest   = rsp->regs[RSP_CP0_REGISTER_DMA_CACHE] & 0x1FFC;
        j               = 0;
        do {
            uint32_t source_addr = (source + j) & 0x7FFFFC;
            uint32_t dest_addr   = (dest + j) & 0x1FFC;
            uint32_t word;
            bus_read_word(rsp->bus, source_addr, &word);
            if (dest_addr & 0x1000) {
                rsp->opcode_cache[(dest_addr - 0x1000) >> 2] = *rsp_decode_instruction(word);
            } else {
                word = byteswap_32(word);
            }
            memcpy(rsp->mem + dest_addr, &word, sizeof(word));
            j += 4;
        } while (j < length);
        rsp->regs[RSP_CP0_REGISTER_DMA_DRAM] += length + skip;
        rsp->regs[RSP_CP0_REGISTER_DMA_CACHE] += length;
    } while (++i <= count);
}
void rsp_dma_write(struct rsp *rsp)
{
    uint32_t length = (rsp->regs[RSP_CP0_REGISTER_DMA_WRITE_LENGTH] & 0xFFF) + 1;
    uint32_t skip   = rsp->regs[RSP_CP0_REGISTER_DMA_WRITE_LENGTH] >> 20 & 0xFFF;
    unsigned count  = rsp->regs[RSP_CP0_REGISTER_DMA_WRITE_LENGTH] >> 12 & 0xFF;
    unsigned j, i = 0;
    length = (length + 0x7) & ~0x7;
    rsp->regs[RSP_CP0_REGISTER_DMA_CACHE] &= ~0x7;
    rsp->regs[RSP_CP0_REGISTER_DMA_DRAM] &= ~0x7;
    if (((rsp->regs[RSP_CP0_REGISTER_DMA_CACHE] & 0xFFF) + length) > 0x1000)
        length = 0x1000 - (rsp->regs[RSP_CP0_REGISTER_DMA_CACHE] & 0xFFF);
    do {
        uint32_t dest   = rsp->regs[RSP_CP0_REGISTER_DMA_DRAM] & 0x7FFFFC;
        uint32_t source = rsp->regs[RSP_CP0_REGISTER_DMA_CACHE] & 0x1FFC;
        j               = 0;
        do {
            uint32_t source_addr = (source + j) & 0x1FFC;
            uint32_t dest_addr   = (dest + j) & 0x7FFFFC;
            uint32_t word;
            memcpy(&word, rsp->mem + source_addr, sizeof(word));
            if (!(source_addr & 0x1000))
                word = byteswap_32(word);
            bus_write_word(rsp->bus, dest_addr, word, ~0U);
            j += 4;
        } while (j < length);
        rsp->regs[RSP_CP0_REGISTER_DMA_CACHE] += length;
        rsp->regs[RSP_CP0_REGISTER_DMA_DRAM] += length + skip;
    } while (++i <= count);
}
int read_sp_mem(void *opaque, uint32_t address, uint32_t *word)
{
    struct rsp *rsp    = (struct rsp *)opaque;
    unsigned    offset = address & 0x1FFC;
    memcpy(word, rsp->mem + offset, sizeof(*word));
    if (!(offset & 0x1000))
        *word = byteswap_32(*word);
    return 0;
}
int read_sp_regs(void *opaque, uint32_t address, uint32_t *word)
{
    struct rsp      *rsp    = (struct rsp *)opaque;
    uint32_t         offset = address - SP_REGS_BASE_ADDRESS;
    enum sp_register reg    = (sp_register)(offset >> 2);
    *word                   = rsp_read_cp0_reg(rsp, reg);
    debug_mmio_read(sp, sp_register_mnemonics[reg], *word);
    return 0;
}
int read_sp_regs2(void *opaque, uint32_t address, uint32_t *word)
{
    struct rsp      *rsp    = (struct rsp *)opaque;
    uint32_t         offset = address - SP_REGS2_BASE_ADDRESS;
    enum sp_register reg    = (sp_register)((offset >> 2) + SP_PC_REG);
    if (reg == SP_PC_REG)
        *word = rsp->pipeline.dfwb_latch.common.pc;
    else
        abort();
    debug_mmio_read(sp, sp_register_mnemonics[reg], *word);
    return 0;
}
int write_sp_mem(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    struct rsp *rsp    = (struct rsp *)opaque;
    unsigned    offset = address & 0x1FFC;
    if (offset & 0x1000) {
        rsp->opcode_cache[(offset - 0x1000) >> 2] = *rsp_decode_instruction(word);
    } else {
        word = byteswap_32(word);
    }
    memcpy(rsp->mem + offset, &word, sizeof(word));
    return 0;
}
int write_sp_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    struct rsp      *rsp    = (struct rsp *)opaque;
    uint32_t         offset = address - SP_REGS_BASE_ADDRESS;
    enum sp_register reg    = (sp_register)(offset >> 2);
    debug_mmio_write(sp, sp_register_mnemonics[reg], word, dqm);
    rsp_write_cp0_reg(rsp, reg, word);
    return 0;
}
int write_sp_regs2(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    struct rsp      *rsp    = (struct rsp *)opaque;
    uint32_t         offset = address - SP_REGS2_BASE_ADDRESS;
    enum sp_register reg    = (sp_register)((offset >> 2) + SP_PC_REG);
    debug_mmio_write(sp, sp_register_mnemonics[reg], word, dqm);
    if (reg == SP_PC_REG) {
        assert((rsp->regs[RSP_CP0_REGISTER_SP_STATUS] & SP_STATUS_HALT) && "SP PC set while the RSP is running");
        rsp_pipeline_init(&rsp->pipeline);
        rsp->pipeline.ifrd_latch.pc = word & 0xFFC;
    } else
        abort();
    return 0;
}
const uint16_t rsp_reciprocal_rom[1024] __attribute__((aligned(64))) = {
    0xFFFF, 0xFF00, 0xFE01, 0xFD04, 0xFC07, 0xFB0C, 0xFA11, 0xF918, 0xF81F, 0xF727, 0xF631, 0xF53B, 0xF446, 0xF352,
    0xF25F, 0xF16D, 0xF07C, 0xEF8B, 0xEE9C, 0xEDAE, 0xECC0, 0xEBD3, 0xEAE8, 0xE9FD, 0xE913, 0xE829, 0xE741, 0xE65A,
    0xE573, 0xE48D, 0xE3A9, 0xE2C5, 0xE1E1, 0xE0FF, 0xE01E, 0xDF3D, 0xDE5D, 0xDD7E, 0xDCA0, 0xDBC2, 0xDAE6, 0xDA0A,
    0xD92F, 0xD854, 0xD77B, 0xD6A2, 0xD5CA, 0xD4F3, 0xD41D, 0xD347, 0xD272, 0xD19E, 0xD0CB, 0xCFF8, 0xCF26, 0xCE55,
    0xCD85, 0xCCB5, 0xCBE6, 0xCB18, 0xCA4B, 0xC97E, 0xC8B2, 0xC7E7, 0xC71C, 0xC652, 0xC589, 0xC4C0, 0xC3F8, 0xC331,
    0xC26B, 0xC1A5, 0xC0E0, 0xC01C, 0xBF58, 0xBE95, 0xBDD2, 0xBD10, 0xBC4F, 0xBB8F, 0xBACF, 0xBA10, 0xB951, 0xB894,
    0xB7D6, 0xB71A, 0xB65E, 0xB5A2, 0xB4E8, 0xB42E, 0xB374, 0xB2BB, 0xB203, 0xB14B, 0xB094, 0xAFDE, 0xAF28, 0xAE73,
    0xADBE, 0xAD0A, 0xAC57, 0xABA4, 0xAAF1, 0xAA40, 0xA98E, 0xA8DE, 0xA82E, 0xA77E, 0xA6D0, 0xA621, 0xA574, 0xA4C6,
    0xA41A, 0xA36E, 0xA2C2, 0xA217, 0xA16D, 0xA0C3, 0xA01A, 0x9F71, 0x9EC8, 0x9E21, 0x9D79, 0x9CD3, 0x9C2D, 0x9B87,
    0x9AE2, 0x9A3D, 0x9999, 0x98F6, 0x9852, 0x97B0, 0x970E, 0x966C, 0x95CB, 0x952B, 0x948B, 0x93EB, 0x934C, 0x92AD,
    0x920F, 0x9172, 0x90D4, 0x9038, 0x8F9C, 0x8F00, 0x8E65, 0x8DCA, 0x8D30, 0x8C96, 0x8BFC, 0x8B64, 0x8ACB, 0x8A33,
    0x899C, 0x8904, 0x886E, 0x87D8, 0x8742, 0x86AD, 0x8618, 0x8583, 0x84F0, 0x845C, 0x83C9, 0x8336, 0x82A4, 0x8212,
    0x8181, 0x80F0, 0x8060, 0x7FD0, 0x7F40, 0x7EB1, 0x7E22, 0x7D93, 0x7D05, 0x7C78, 0x7BEB, 0x7B5E, 0x7AD2, 0x7A46,
    0x79BA, 0x792F, 0x78A4, 0x781A, 0x7790, 0x7706, 0x767D, 0x75F5, 0x756C, 0x74E4, 0x745D, 0x73D5, 0x734F, 0x72C8,
    0x7242, 0x71BC, 0x7137, 0x70B2, 0x702E, 0x6FA9, 0x6F26, 0x6EA2, 0x6E1F, 0x6D9C, 0x6D1A, 0x6C98, 0x6C16, 0x6B95,
    0x6B14, 0x6A94, 0x6A13, 0x6993, 0x6914, 0x6895, 0x6816, 0x6798, 0x6719, 0x669C, 0x661E, 0x65A1, 0x6524, 0x64A8,
    0x642C, 0x63B0, 0x6335, 0x62BA, 0x623F, 0x61C5, 0x614B, 0x60D1, 0x6058, 0x5FDF, 0x5F66, 0x5EED, 0x5E75, 0x5DFD,
    0x5D86, 0x5D0F, 0x5C98, 0x5C22, 0x5BAB, 0x5B35, 0x5AC0, 0x5A4B, 0x59D6, 0x5961, 0x58ED, 0x5879, 0x5805, 0x5791,
    0x571E, 0x56AC, 0x5639, 0x55C7, 0x5555, 0x54E3, 0x5472, 0x5401, 0x5390, 0x5320, 0x52AF, 0x5240, 0x51D0, 0x5161,
    0x50F2, 0x5083, 0x5015, 0x4FA6, 0x4F38, 0x4ECB, 0x4E5E, 0x4DF1, 0x4D84, 0x4D17, 0x4CAB, 0x4C3F, 0x4BD3, 0x4B68,
    0x4AFD, 0x4A92, 0x4A27, 0x49BD, 0x4953, 0x48E9, 0x4880, 0x4817, 0x47AE, 0x4745, 0x46DC, 0x4674, 0x460C, 0x45A5,
    0x453D, 0x44D6, 0x446F, 0x4408, 0x43A2, 0x433C, 0x42D6, 0x4270, 0x420B, 0x41A6, 0x4141, 0x40DC, 0x4078, 0x4014,
    0x3FB0, 0x3F4C, 0x3EE8, 0x3E85, 0x3E22, 0x3DC0, 0x3D5D, 0x3CFB, 0x3C99, 0x3C37, 0x3BD6, 0x3B74, 0x3B13, 0x3AB2,
    0x3A52, 0x39F1, 0x3991, 0x3931, 0x38D2, 0x3872, 0x3813, 0x37B4, 0x3755, 0x36F7, 0x3698, 0x363A, 0x35DC, 0x357F,
    0x3521, 0x34C4, 0x3467, 0x340A, 0x33AE, 0x3351, 0x32F5, 0x3299, 0x323E, 0x31E2, 0x3187, 0x312C, 0x30D1, 0x3076,
    0x301C, 0x2FC2, 0x2F68, 0x2F0E, 0x2EB4, 0x2E5B, 0x2E02, 0x2DA9, 0x2D50, 0x2CF8, 0x2C9F, 0x2C47, 0x2BEF, 0x2B97,
    0x2B40, 0x2AE8, 0x2A91, 0x2A3A, 0x29E4, 0x298D, 0x2937, 0x28E0, 0x288B, 0x2835, 0x27DF, 0x278A, 0x2735, 0x26E0,
    0x268B, 0x2636, 0x25E2, 0x258D, 0x2539, 0x24E5, 0x2492, 0x243E, 0x23EB, 0x2398, 0x2345, 0x22F2, 0x22A0, 0x224D,
    0x21FB, 0x21A9, 0x2157, 0x2105, 0x20B4, 0x2063, 0x2012, 0x1FC1, 0x1F70, 0x1F1F, 0x1ECF, 0x1E7F, 0x1E2E, 0x1DDF,
    0x1D8F, 0x1D3F, 0x1CF0, 0x1CA1, 0x1C52, 0x1C03, 0x1BB4, 0x1B66, 0x1B17, 0x1AC9, 0x1A7B, 0x1A2D, 0x19E0, 0x1992,
    0x1945, 0x18F8, 0x18AB, 0x185E, 0x1811, 0x17C4, 0x1778, 0x172C, 0x16E0, 0x1694, 0x1648, 0x15FD, 0x15B1, 0x1566,
    0x151B, 0x14D0, 0x1485, 0x143B, 0x13F0, 0x13A6, 0x135C, 0x1312, 0x12C8, 0x127F, 0x1235, 0x11EC, 0x11A3, 0x1159,
    0x1111, 0x10C8, 0x107F, 0x1037, 0x0FEF, 0x0FA6, 0x0F5E, 0x0F17, 0x0ECF, 0x0E87, 0x0E40, 0x0DF9, 0x0DB2, 0x0D6B,
    0x0D24, 0x0CDD, 0x0C97, 0x0C50, 0x0C0A, 0x0BC4, 0x0B7E, 0x0B38, 0x0AF2, 0x0AAD, 0x0A68, 0x0A22, 0x09DD, 0x0998,
    0x0953, 0x090F, 0x08CA, 0x0886, 0x0842, 0x07FD, 0x07B9, 0x0776, 0x0732, 0x06EE, 0x06AB, 0x0668, 0x0624, 0x05E1,
    0x059E, 0x055C, 0x0519, 0x04D6, 0x0494, 0x0452, 0x0410, 0x03CE, 0x038C, 0x034A, 0x0309, 0x02C7, 0x0286, 0x0245,
    0x0204, 0x01C3, 0x0182, 0x0141, 0x0101, 0x00C0, 0x0080, 0x0040, 0x6A09, 0xFFFF, 0x6955, 0xFF00, 0x68A1, 0xFE02,
    0x67EF, 0xFD06, 0x673E, 0xFC0B, 0x668D, 0xFB12, 0x65DE, 0xFA1A, 0x6530, 0xF923, 0x6482, 0xF82E, 0x63D6, 0xF73B,
    0x632B, 0xF648, 0x6280, 0xF557, 0x61D7, 0xF467, 0x612E, 0xF379, 0x6087, 0xF28C, 0x5FE0, 0xF1A0, 0x5F3A, 0xF0B6,
    0x5E95, 0xEFCD, 0x5DF1, 0xEEE5, 0x5D4E, 0xEDFF, 0x5CAC, 0xED19, 0x5C0B, 0xEC35, 0x5B6B, 0xEB52, 0x5ACB, 0xEA71,
    0x5A2C, 0xE990, 0x598F, 0xE8B1, 0x58F2, 0xE7D3, 0x5855, 0xE6F6, 0x57BA, 0xE61B, 0x5720, 0xE540, 0x5686, 0xE467,
    0x55ED, 0xE38E, 0x5555, 0xE2B7, 0x54BE, 0xE1E1, 0x5427, 0xE10D, 0x5391, 0xE039, 0x52FC, 0xDF66, 0x5268, 0xDE94,
    0x51D5, 0xDDC4, 0x5142, 0xDCF4, 0x50B0, 0xDC26, 0x501F, 0xDB59, 0x4F8E, 0xDA8C, 0x4EFE, 0xD9C1, 0x4E6F, 0xD8F7,
    0x4DE1, 0xD82D, 0x4D53, 0xD765, 0x4CC6, 0xD69E, 0x4C3A, 0xD5D7, 0x4BAF, 0xD512, 0x4B24, 0xD44E, 0x4A9A, 0xD38A,
    0x4A10, 0xD2C8, 0x4987, 0xD206, 0x48FF, 0xD146, 0x4878, 0xD086, 0x47F1, 0xCFC7, 0x476B, 0xCF0A, 0x46E5, 0xCE4D,
    0x4660, 0xCD91, 0x45DC, 0xCCD6, 0x4558, 0xCC1B, 0x44D5, 0xCB62, 0x4453, 0xCAA9, 0x43D1, 0xC9F2, 0x434F, 0xC93B,
    0x42CF, 0xC885, 0x424F, 0xC7D0, 0x41CF, 0xC71C, 0x4151, 0xC669, 0x40D2, 0xC5B6, 0x4055, 0xC504, 0x3FD8, 0xC453,
    0x3F5B, 0xC3A3, 0x3EDF, 0xC2F4, 0x3E64, 0xC245, 0x3DE9, 0xC198, 0x3D6E, 0xC0EB, 0x3CF5, 0xC03F, 0x3C7C, 0xBF93,
    0x3C03, 0xBEE9, 0x3B8B, 0xBE3F, 0x3B13, 0xBD96, 0x3A9C, 0xBCED, 0x3A26, 0xBC46, 0x39B0, 0xBB9F, 0x393A, 0xBAF8,
    0x38C5, 0xBA53, 0x3851, 0xB9AE, 0x37DD, 0xB90A, 0x3769, 0xB867, 0x36F6, 0xB7C5, 0x3684, 0xB723, 0x3612, 0xB681,
    0x35A0, 0xB5E1, 0x352F, 0xB541, 0x34BF, 0xB4A2, 0x344F, 0xB404, 0x33DF, 0xB366, 0x3370, 0xB2C9, 0x3302, 0xB22C,
    0x3293, 0xB191, 0x3226, 0xB0F5, 0x31B9, 0xB05B, 0x314C, 0xAFC1, 0x30DF, 0xAF28, 0x3074, 0xAE8F, 0x3008, 0xADF7,
    0x2F9D, 0xAD60, 0x2F33, 0xACC9, 0x2EC8, 0xAC33, 0x2E5F, 0xAB9E, 0x2DF6, 0xAB09, 0x2D8D, 0xAA75, 0x2D24, 0xA9E1,
    0x2CBC, 0xA94E, 0x2C55, 0xA8BC, 0x2BEE, 0xA82A, 0x2B87, 0xA799, 0x2B21, 0xA708, 0x2ABB, 0xA678, 0x2A55, 0xA5E8,
    0x29F0, 0xA559, 0x298B, 0xA4CB, 0x2927, 0xA43D, 0x28C3, 0xA3B0, 0x2860, 0xA323, 0x27FD, 0xA297, 0x279A, 0xA20B,
    0x2738, 0xA180, 0x26D6, 0xA0F6, 0x2674, 0xA06C, 0x2613, 0x9FE2, 0x25B2, 0x9F59, 0x2552, 0x9ED1, 0x24F2, 0x9E49,
    0x2492, 0x9DC2, 0x2432, 0x9D3B, 0x23D3, 0x9CB4, 0x2375, 0x9C2F, 0x2317, 0x9BA9, 0x22B9, 0x9B25, 0x225B, 0x9AA0,
    0x21FE, 0x9A1C, 0x21A1, 0x9999, 0x2145, 0x9916, 0x20E8, 0x9894, 0x208D, 0x9812, 0x2031, 0x9791, 0x1FD6, 0x9710,
    0x1F7B, 0x968F, 0x1F21, 0x960F, 0x1EC7, 0x9590, 0x1E6D, 0x9511, 0x1E13, 0x9492, 0x1DBA, 0x9414, 0x1D61, 0x9397,
    0x1D09, 0x931A, 0x1CB1, 0x929D, 0x1C59, 0x9221, 0x1C01, 0x91A5, 0x1BAA, 0x9129, 0x1B53, 0x90AF, 0x1AFC, 0x9034,
    0x1AA6, 0x8FBA, 0x1A50, 0x8F40, 0x19FA, 0x8EC7, 0x19A5, 0x8E4F, 0x1950, 0x8DD6, 0x18FB, 0x8D5E, 0x18A7, 0x8CE7,
    0x1853, 0x8C70, 0x17FF, 0x8BF9, 0x17AB, 0x8B83, 0x1758, 0x8B0D, 0x1705, 0x8A98, 0x16B2, 0x8A23, 0x1660, 0x89AE,
    0x160D, 0x893A, 0x15BC, 0x88C6, 0x156A, 0x8853, 0x1519, 0x87E0, 0x14C8, 0x876D, 0x1477, 0x86FB, 0x1426, 0x8689,
    0x13D6, 0x8618, 0x1386, 0x85A7, 0x1337, 0x8536, 0x12E7, 0x84C6, 0x1298, 0x8456, 0x1249, 0x83E7, 0x11FB, 0x8377,
    0x11AC, 0x8309, 0x115E, 0x829A, 0x1111, 0x822C, 0x10C3, 0x81BF, 0x1076, 0x8151, 0x1029, 0x80E4, 0x0FDC, 0x8078,
    0x0F8F, 0x800C, 0x0F43, 0x7FA0, 0x0EF7, 0x7F34, 0x0EAB, 0x7EC9, 0x0E60, 0x7E5E, 0x0E15, 0x7DF4, 0x0DCA, 0x7D8A,
    0x0D7F, 0x7D20, 0x0D34, 0x7CB6, 0x0CEA, 0x7C4D, 0x0CA0, 0x7BE5, 0x0C56, 0x7B7C, 0x0C0C, 0x7B14, 0x0BC3, 0x7AAC,
    0x0B7A, 0x7A45, 0x0B31, 0x79DE, 0x0AE8, 0x7977, 0x0AA0, 0x7911, 0x0A58, 0x78AB, 0x0A10, 0x7845, 0x09C8, 0x77DF,
    0x0981, 0x777A, 0x0939, 0x7715, 0x08F2, 0x76B1, 0x08AB, 0x764D, 0x0865, 0x75E9, 0x081E, 0x7585, 0x07D8, 0x7522,
    0x0792, 0x74BF, 0x074D, 0x745D, 0x0707, 0x73FA, 0x06C2, 0x7398, 0x067D, 0x7337, 0x0638, 0x72D5, 0x05F3, 0x7274,
    0x05AF, 0x7213, 0x056A, 0x71B3, 0x0526, 0x7152, 0x04E2, 0x70F2, 0x049F, 0x7093, 0x045B, 0x7033, 0x0418, 0x6FD4,
    0x03D5, 0x6F76, 0x0392, 0x6F17, 0x0350, 0x6EB9, 0x030D, 0x6E5B, 0x02CB, 0x6DFD, 0x0289, 0x6DA0, 0x0247, 0x6D43,
    0x0206, 0x6CE6, 0x01C4, 0x6C8A, 0x0183, 0x6C2D, 0x0142, 0x6BD1, 0x0101, 0x6B76, 0x00C0, 0x6B1A, 0x0080, 0x6ABF,
    0x0040, 0x6A64};
const uint16_t rsp_vlogic_mask[2][8] __attribute__((aligned(32))) = {
    {0, 0, 0, 0, 0, 0, 0, 0},
    {static_cast<uint16_t>(~0), static_cast<uint16_t>(~0), static_cast<uint16_t>(~0), static_cast<uint16_t>(~0),
     static_cast<uint16_t>(~0), static_cast<uint16_t>(~0), static_cast<uint16_t>(~0), static_cast<uint16_t>(~0)}};
static const uint16_t sll_b2l_keys[16][8] __attribute__((aligned(64))) = {
    {0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F},
    {0x8000, 0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E},
    {0x8080, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D},
    {0x8080, 0x8000, 0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C},
    {0x8080, 0x8080, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B},
    {0x8080, 0x8080, 0x8000, 0x0102, 0x0304, 0x0506, 0x0708, 0x090A},
    {0x8080, 0x8080, 0x8080, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809},
    {0x8080, 0x8080, 0x8080, 0x8000, 0x0102, 0x0304, 0x0506, 0x0708},
    {0x8080, 0x8080, 0x8080, 0x8080, 0x0001, 0x0203, 0x0405, 0x0607},
    {0x8080, 0x8080, 0x8080, 0x8080, 0x8000, 0x0102, 0x0304, 0x0506},
    {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x0001, 0x0203, 0x0405},
    {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8000, 0x0102, 0x0304},
    {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x0001, 0x0203},
    {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8000, 0x0102},
    {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x0001},
    {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8000},
};
static const uint16_t sll_l2b_keys[16][8] __attribute__((aligned(64))) = {
    {0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F},
    {0x0180, 0x0300, 0x0502, 0x0704, 0x0906, 0x0B08, 0x0D0A, 0x0E0C},
    {0x8080, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D},
    {0x8080, 0x0180, 0x0300, 0x0502, 0x0704, 0x0906, 0x0B08, 0x0D0A},
    {0x8080, 0x8080, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B},
    {0x8080, 0x8080, 0x0180, 0x0300, 0x0502, 0x0704, 0x0906, 0x0B08},
    {0x8080, 0x8080, 0x8080, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809},
    {0x8080, 0x8080, 0x8080, 0x0180, 0x0300, 0x0502, 0x0704, 0x0906},
    {0x8080, 0x8080, 0x8080, 0x8080, 0x0001, 0x0203, 0x0405, 0x0607},
    {0x8080, 0x8080, 0x8080, 0x8080, 0x0180, 0x0300, 0x0502, 0x0704},
    {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x0001, 0x0203, 0x0405},
    {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x0180, 0x0300, 0x0502},
    {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x0001, 0x0203},
    {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x0180, 0x0300},
    {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x0001},
    {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x0180},
};
static const uint16_t ror_b2l_keys[16][8] __attribute__((aligned(64))) = {
    {0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F},
    {0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F00},
    {0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F, 0x0001},
    {0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F00, 0x0102},
    {0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F, 0x0001, 0x0203},
    {0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F00, 0x0102, 0x0304},
    {0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F, 0x0001, 0x0203, 0x0405},
    {0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F00, 0x0102, 0x0304, 0x0506},
    {0x0809, 0x0A0B, 0x0C0D, 0x0E0F, 0x0001, 0x0203, 0x0405, 0x0607},
    {0x090A, 0x0B0C, 0x0D0E, 0x0F00, 0x0102, 0x0304, 0x0506, 0x0708},
    {0x0A0B, 0x0C0D, 0x0E0F, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809},
    {0x0B0C, 0x0D0E, 0x0F00, 0x0102, 0x0304, 0x0506, 0x0708, 0x090A},
    {0x0C0D, 0x0E0F, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B},
    {0x0D0E, 0x0F00, 0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C},
    {0x0E0F, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D},
    {0x0F00, 0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E},
};
static const uint16_t rol_l2b_keys[16][8] __attribute__((aligned(64))) = {
    {0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F},
    {0x010E, 0x0300, 0x0502, 0x0704, 0x0906, 0x0B08, 0x0D0A, 0x0F0C},
    {0x0E0F, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D},
    {0x0F0C, 0x010E, 0x0300, 0x0502, 0x0704, 0x0906, 0x0B08, 0x0D0A},
    {0x0C0D, 0x0E0F, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B},
    {0x0D0A, 0x0F0C, 0x010E, 0x0300, 0x0502, 0x0704, 0x0906, 0x0B08},
    {0x0A0B, 0x0C0D, 0x0E0F, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809},
    {0x0B08, 0x0D0A, 0x0F0C, 0x010E, 0x0300, 0x0502, 0x0704, 0x0906},
    {0x0809, 0x0A0B, 0x0C0D, 0x0E0F, 0x0001, 0x0203, 0x0405, 0x0607},
    {0x0906, 0x0B08, 0x0D0A, 0x0F0C, 0x010E, 0x0300, 0x0502, 0x0704},
    {0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F, 0x0001, 0x0203, 0x0405},
    {0x0704, 0x0906, 0x0B08, 0x0D0A, 0x0F0C, 0x010E, 0x0300, 0x0502},
    {0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F, 0x0001, 0x0203},
    {0x0502, 0x0704, 0x0906, 0x0B08, 0x0D0A, 0x0F0C, 0x010E, 0x0300},
    {0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F, 0x0001},
    {0x0300, 0x0502, 0x0704, 0x0906, 0x0B08, 0x0D0A, 0x0F0C, 0x010E},
};
static const uint16_t ror_l2b_keys[16][8] __attribute__((aligned(64))) = {
    {0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F},
    {0x0300, 0x0502, 0x0704, 0x0906, 0x0B08, 0x0D0A, 0x0F0C, 0x010E},
    {0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F, 0x0001},
    {0x0502, 0x0704, 0x0906, 0x0B08, 0x0D0A, 0x0F0C, 0x010E, 0x0300},
    {0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F, 0x0001, 0x0203},
    {0x0704, 0x0906, 0x0B08, 0x0D0A, 0x0F0C, 0x010E, 0x0300, 0x0502},
    {0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F, 0x0001, 0x0203, 0x0405},
    {0x0906, 0x0B08, 0x0D0A, 0x0F0C, 0x010E, 0x0300, 0x0502, 0x0704},
    {0x0809, 0x0A0B, 0x0C0D, 0x0E0F, 0x0001, 0x0203, 0x0405, 0x0607},
    {0x0B08, 0x0D0A, 0x0F0C, 0x010E, 0x0300, 0x0502, 0x0704, 0x0906},
    {0x0A0B, 0x0C0D, 0x0E0F, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809},
    {0x0D0A, 0x0F0C, 0x010E, 0x0300, 0x0502, 0x0704, 0x0906, 0x0B08},
    {0x0C0D, 0x0E0F, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B},
    {0x0F0C, 0x010E, 0x0300, 0x0502, 0x0704, 0x0906, 0x0B08, 0x0D0A},
    {0x0E0F, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D},
    {0x010E, 0x0300, 0x0502, 0x0704, 0x0906, 0x0B08, 0x0D0A, 0x0F0C},
};
static inline __m128i sse2_pshufb_loop8(__m128i v, const uint8_t *keys)
{
    uint8_t  temp[(0x80 | 128) + 1] __attribute__((aligned(16)));
    unsigned j;
    _mm_store_si128((__m128i *)temp, v);
    temp[0x80] = 0;
    for (j = 0; j < 16; j += 4) {
        temp[j + 16] = temp[keys[j + 0]];
        temp[j + 17] = temp[keys[j + 1]];
        temp[j + 18] = temp[keys[j + 2]];
        temp[j + 19] = temp[keys[j + 3]];
    }
    return _mm_load_si128(((__m128i *)temp) + 1);
}
static inline __m128i sse2_pshufb(__m128i v, const uint16_t *keys)
{
    union
    {
        const uint16_t *k16;
        const uint8_t  *k8;
    } x;
    x.k16 = keys;
    return sse2_pshufb_loop8(v, x.k8);
}
void arch_rsp_destroy(struct rsp *rsp)
{
}
void rsp_set_flags(uint16_t *flags, uint16_t rt)
{
    unsigned i;

    static const uint16_t array[16][4] = {
        {0x0000, 0x0000, 0x0000, 0x0000}, {0xFFFF, 0x0000, 0x0000, 0x0000}, {0x0000, 0xFFFF, 0x0000, 0x0000},
        {0xFFFF, 0xFFFF, 0x0000, 0x0000}, {0x0000, 0x0000, 0xFFFF, 0x0000}, {0xFFFF, 0x0000, 0xFFFF, 0x0000},
        {0x0000, 0xFFFF, 0xFFFF, 0x0000}, {0xFFFF, 0xFFFF, 0xFFFF, 0x0000}, {0x0000, 0x0000, 0x0000, 0xFFFF},
        {0xFFFF, 0x0000, 0x0000, 0xFFFF}, {0x0000, 0xFFFF, 0x0000, 0xFFFF}, {0xFFFF, 0xFFFF, 0x0000, 0xFFFF},
        {0x0000, 0x0000, 0xFFFF, 0xFFFF}, {0xFFFF, 0x0000, 0xFFFF, 0xFFFF}, {0x0000, 0xFFFF, 0xFFFF, 0xFFFF},
        {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
    };
    for (i = 0; i < 2; i++, rt >>= 4)
        memcpy(flags + 8 + i * 4, array[rt & 0xF], sizeof(array[0]));
    for (i = 0; i < 2; i++, rt >>= 4)
        memcpy(flags + 0 + i * 4, array[rt & 0xF], sizeof(array[0]));
}
int arch_rsp_init(struct rsp *rsp)
{
    return 0;
}
__m128i rsp_vect_load_and_shuffle_operand(const uint16_t *src, unsigned element)
{
    __m128i v;
    switch (element) {
        case 0:
        case 1:
            v = _mm_load_si128((__m128i *)src);
            return v;
        case 2:
            v = _mm_load_si128((__m128i *)src);
            v = ((__m128i)__builtin_ia32_pshuflw((__v8hi)(__m128i)(v),
                                                 (int)((((2) << 6) | ((2) << 4) | ((0) << 2) | (0)))));
            v = ((__m128i)__builtin_ia32_pshufhw((__v8hi)(__m128i)(v),
                                                 (int)((((2) << 6) | ((2) << 4) | ((0) << 2) | (0)))));
            return v;
        case 3:
            v = _mm_load_si128((__m128i *)src);
            v = ((__m128i)__builtin_ia32_pshuflw((__v8hi)(__m128i)(v),
                                                 (int)((((3) << 6) | ((3) << 4) | ((1) << 2) | (1)))));
            v = ((__m128i)__builtin_ia32_pshufhw((__v8hi)(__m128i)(v),
                                                 (int)((((3) << 6) | ((3) << 4) | ((1) << 2) | (1)))));
            return v;
        case 4:
        case 5:
        case 6:
        case 7:
            __asm__("" : "=x"(v));
            v = ((__m128i)__builtin_ia32_vec_set_v8hi((__v8hi)(__m128i)(v), (int)(src[element - 4]), (int)(0)));
            v = ((__m128i)__builtin_ia32_vec_set_v8hi((__v8hi)(__m128i)(v), (int)(src[element - 0]), (int)(1)));
            v = ((__m128i)__builtin_ia32_pshuflw((__v8hi)(__m128i)(v),
                                                 (int)((((1) << 6) | ((1) << 4) | ((0) << 2) | (0)))));
            v = ((__m128i)__builtin_ia32_pshufd((__v4si)(__m128i)(v),
                                                (int)((((1) << 6) | ((1) << 4) | ((0) << 2) | (0)))));
            return v;
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
            __asm__("" : "=x"(v));
            v = ((__m128i)__builtin_ia32_vec_set_v8hi((__v8hi)(__m128i)(v), (int)(src[element - 8]), (int)(0)));
            v = _mm_unpacklo_epi16(v, v);
            v = ((__m128i)__builtin_ia32_pshufd((__v4si)(__m128i)(v),
                                                (int)((((0) << 6) | ((0) << 4) | ((0) << 2) | (0)))));
            return v;
    }
    __builtin_trap();
}
void rsp_vload_group1(struct rsp *rsp, uint32_t addr, unsigned element, uint16_t *regp, rsp_vect_t reg, rsp_vect_t dqm)
{
    __m128i  ekey, data;
    unsigned offset = addr & 0x7;
    unsigned ror    = offset - element;
    if (offset) {
        uint32_t aligned_addr_lo = addr & static_cast<uint16_t>(~0x7);
        uint32_t aligned_addr_hi = (aligned_addr_lo + 8) & 0xFFF;
        __m128i  temp;
        data = _mm_loadl_epi64((__m128i *)(rsp->mem + aligned_addr_lo));
        temp = _mm_loadl_epi64((__m128i *)(rsp->mem + aligned_addr_hi));
        data = _mm_unpacklo_epi64(data, temp);
    } else
        data = _mm_loadl_epi64((__m128i *)(rsp->mem + addr));
    dqm  = sse2_pshufb(dqm, sll_b2l_keys[element]);
    data = sse2_pshufb(data, ror_b2l_keys[ror & 0xF]);
    data = _mm_and_si128(dqm, data);
    reg  = _mm_andnot_si128(dqm, reg);
    reg  = _mm_or_si128(data, reg);
    _mm_store_si128((__m128i *)regp, reg);
}
void rsp_vload_group2(struct rsp *rsp, uint32_t addr, unsigned element, uint16_t *regp, rsp_vect_t reg, rsp_vect_t dqm)
{
    unsigned offset = addr & 0x7;
    __m128i  data, zero;
    if (offset) {
        uint32_t aligned_addr_lo = addr & static_cast<uint16_t>(~0x7);
        uint32_t aligned_addr_hi = (aligned_addr_lo + 8) & 0xFFF;
        uint64_t datalow, datahigh;
        memcpy(&datalow, rsp->mem + aligned_addr_lo, sizeof(datalow));
        memcpy(&datahigh, rsp->mem + aligned_addr_hi, sizeof(datahigh));
        datalow  = __builtin_bswap64(datalow);
        datahigh = __builtin_bswap64(datahigh);
        datahigh >>= ((8 - offset) << 3);
        datalow <<= (offset << 3);
        datalow = datahigh | datalow;
        datalow = __builtin_bswap64(datalow);
        data    = _mm_loadl_epi64((__m128i *)&datalow);
    } else
        data = _mm_loadl_epi64((__m128i *)(rsp->mem + addr));
    zero = _mm_setzero_si128();
    data = _mm_unpacklo_epi8(zero, data);
    if (rsp->pipeline.exdf_latch.request.type != RSP_MEM_REQUEST_PACK)
        data = _mm_srli_epi16(data, 1);
    _mm_store_si128((__m128i *)regp, data);
}
void rsp_vload_group4(struct rsp *rsp, uint32_t addr, unsigned element, uint16_t *regp, rsp_vect_t reg, rsp_vect_t dqm)
{
    uint32_t aligned_addr = addr & 0xFF0;
    unsigned offset       = addr & 0xF;
    unsigned ror;
    __m128i  data = _mm_load_si128((__m128i *)(rsp->mem + aligned_addr));
    __m128i  dkey;
    ror = 16 - element + offset;
    if (rsp->pipeline.exdf_latch.request.type != RSP_MEM_REQUEST_QUAD)
        dqm = _mm_cmpeq_epi8(_mm_setzero_si128(), dqm);
    data = sse2_pshufb(data, ror_b2l_keys[ror & 0xF]);
    dqm  = sse2_pshufb(dqm, ror_b2l_keys[ror & 0xF]);
    data = _mm_and_si128(dqm, data);
    reg  = _mm_andnot_si128(dqm, reg);
    data = _mm_or_si128(data, reg);
    _mm_store_si128((__m128i *)regp, data);
}
void rsp_vstore_group1(struct rsp *rsp, uint32_t addr, unsigned element, uint16_t *regp, rsp_vect_t reg, rsp_vect_t dqm)
{
    unsigned offset = addr & 0x7;
    unsigned ror    = element - offset;
    __m128i  ekey, data;
    dqm = sse2_pshufb(dqm, sll_l2b_keys[offset]);
    reg = sse2_pshufb(reg, ror_l2b_keys[ror & 0xF]);
    if (offset) {
        uint32_t aligned_addr_lo = addr & static_cast<uint16_t>(~0x7);
        uint32_t aligned_addr_hi = (aligned_addr_lo + 8) & 0xFFF;
        __m128i  temp;
        data = _mm_loadl_epi64((__m128i *)(rsp->mem + aligned_addr_lo));
        temp = _mm_loadl_epi64((__m128i *)(rsp->mem + aligned_addr_hi));
        data = _mm_unpacklo_epi64(data, temp);
        data = _mm_andnot_si128(dqm, data);
        reg  = _mm_and_si128(dqm, reg);
        data = _mm_or_si128(data, reg);
        _mm_storel_epi64((__m128i *)(rsp->mem + aligned_addr_lo), data);
        data = ((__m128i)__builtin_ia32_psrldqi128((__m128i)(data), (int)(8) * 8));
        _mm_storel_epi64((__m128i *)(rsp->mem + aligned_addr_hi), data);
    } else {
        data = _mm_loadl_epi64((__m128i *)(rsp->mem + addr));
        data = _mm_andnot_si128(dqm, data);
        reg  = _mm_and_si128(dqm, reg);
        data = _mm_or_si128(data, reg);
        _mm_storel_epi64((__m128i *)(rsp->mem + addr), data);
    }
}
void rsp_vstore_group2(struct rsp *rsp, uint32_t addr, unsigned element, uint16_t *regp, rsp_vect_t reg, rsp_vect_t dqm)
{
    if (rsp->pipeline.exdf_latch.request.type != RSP_MEM_REQUEST_PACK)
        reg = _mm_slli_epi16(reg, 1);
    reg = _mm_srai_epi16(reg, 8);
    reg = _mm_packs_epi16(reg, reg);
    _mm_storel_epi64((__m128i *)(rsp->mem + addr), reg);
}
void rsp_vstore_group4(struct rsp *rsp, uint32_t addr, unsigned element, uint16_t *regp, rsp_vect_t reg, rsp_vect_t dqm)
{
    uint32_t aligned_addr = addr & 0xFF0;
    unsigned offset       = addr & 0xF;
    unsigned rol          = offset;
    __m128i  data         = _mm_load_si128((__m128i *)(rsp->mem + aligned_addr));
    __m128i  ekey;
    if (rsp->pipeline.exdf_latch.request.type == RSP_MEM_REQUEST_QUAD)
        rol -= element;
    else
        dqm = _mm_cmpeq_epi8(_mm_setzero_si128(), dqm);
    reg  = sse2_pshufb(reg, rol_l2b_keys[rol & 0xF]);
    reg  = _mm_and_si128(dqm, reg);
    data = _mm_andnot_si128(dqm, data);
    data = _mm_or_si128(data, reg);
    _mm_store_si128((__m128i *)(rsp->mem + aligned_addr), data);
}
void rsp_ltv(struct rsp *rsp, uint32_t addr, unsigned element, unsigned vt)
{
    for (int i = 0; i < 8; i++) {
        uint16_t slice;
        memcpy(&slice, rsp->mem + addr + (i << 1), sizeof(slice));
        slice                                      = byteswap_16(slice);
        rsp->cp2.regs[vt + i].e[(i - element) & 7] = slice;
    }
}
void rsp_stv(struct rsp *rsp, uint32_t addr, unsigned element, unsigned vt)
{
    for (int i = 0; i < 8; i++) {
        uint16_t slice = rsp->cp2.regs[vt + ((i + element) & 7)].e[i];
        slice          = byteswap_16(slice);
        memcpy(rsp->mem + addr + (i << 1), &slice, sizeof(slice));
    }
}
rsp_vect_t RSP_VABS(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc = rsp->cp2.acc.e;
    rsp_vect_t acc_lo;
    rsp_vect_t result = rsp_vabs(vs, vt_shuffle, zero, &acc_lo);
    write_acc_lo(acc, acc_lo);
    return result;
}
rsp_vect_t RSP_VADD(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc = rsp->cp2.acc.e;
    rsp_vect_t carry, acc_lo;
    carry             = read_vco_lo(rsp->cp2.flags[RSP_VCO].e);
    rsp_vect_t result = rsp_vadd(vs, vt_shuffle, carry, &acc_lo);
    write_vco_hi(rsp->cp2.flags[RSP_VCO].e, zero);
    write_vco_lo(rsp->cp2.flags[RSP_VCO].e, zero);
    write_acc_lo(acc, acc_lo);
    return result;
}
rsp_vect_t RSP_VADDC(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc = rsp->cp2.acc.e;
    rsp_vect_t sn;
    rsp_vect_t result = rsp_vaddc(vs, vt_shuffle, zero, &sn);
    write_vco_hi(rsp->cp2.flags[RSP_VCO].e, zero);
    write_vco_lo(rsp->cp2.flags[RSP_VCO].e, sn);
    write_acc_lo(acc, result);
    return result;
}
rsp_vect_t RSP_VAND_VNAND(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc    = rsp->cp2.acc.e;
    rsp_vect_t result = rsp_vand_vnand(iw, vs, vt_shuffle);
    write_acc_lo(acc, result);
    return result;
}
rsp_vect_t RSP_VCH(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc = rsp->cp2.acc.e;
    rsp_vect_t ge, le, sign, eq, vce;
    rsp_vect_t result = rsp_vch(vs, vt_shuffle, zero, &ge, &le, &eq, &sign, &vce);
    write_vcc_hi(rsp->cp2.flags[RSP_VCC].e, ge);
    write_vcc_lo(rsp->cp2.flags[RSP_VCC].e, le);
    write_vco_hi(rsp->cp2.flags[RSP_VCO].e, eq);
    write_vco_lo(rsp->cp2.flags[RSP_VCO].e, sign);
    write_vce(rsp->cp2.flags[RSP_VCE].e, vce);
    write_acc_lo(acc, result);
    return result;
}
rsp_vect_t RSP_VCL(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc = rsp->cp2.acc.e;
    rsp_vect_t ge, le, eq, sign, vce;
    ge                = read_vcc_hi(rsp->cp2.flags[RSP_VCC].e);
    le                = read_vcc_lo(rsp->cp2.flags[RSP_VCC].e);
    eq                = read_vco_hi(rsp->cp2.flags[RSP_VCO].e);
    sign              = read_vco_lo(rsp->cp2.flags[RSP_VCO].e);
    vce               = read_vce(rsp->cp2.flags[RSP_VCE].e);
    rsp_vect_t result = rsp_vcl(vs, vt_shuffle, zero, &ge, &le, eq, sign, vce);
    write_vcc_hi(rsp->cp2.flags[RSP_VCC].e, ge);
    write_vcc_lo(rsp->cp2.flags[RSP_VCC].e, le);
    write_vco_hi(rsp->cp2.flags[RSP_VCO].e, zero);
    write_vco_lo(rsp->cp2.flags[RSP_VCO].e, zero);
    write_vce(rsp->cp2.flags[RSP_VCE].e, zero);
    write_acc_lo(acc, result);
    return result;
}
rsp_vect_t RSP_VCR(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc = rsp->cp2.acc.e;
    rsp_vect_t ge, le;
    rsp_vect_t result = rsp_vcr(vs, vt_shuffle, zero, &ge, &le);
    write_vcc_hi(rsp->cp2.flags[RSP_VCC].e, ge);
    write_vcc_lo(rsp->cp2.flags[RSP_VCC].e, le);
    write_vco_hi(rsp->cp2.flags[RSP_VCO].e, zero);
    write_vco_lo(rsp->cp2.flags[RSP_VCO].e, zero);
    write_vce(rsp->cp2.flags[RSP_VCE].e, zero);
    write_acc_lo(acc, result);
    return result;
}
rsp_vect_t RSP_VEQ_VGE_VLT_VNE(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc = rsp->cp2.acc.e;
    rsp_vect_t le, eq, sign;
    eq                = read_vco_hi(rsp->cp2.flags[RSP_VCO].e);
    sign              = read_vco_lo(rsp->cp2.flags[RSP_VCO].e);
    rsp_vect_t result = rsp_veq_vge_vlt_vne(iw, vs, vt_shuffle, zero, &le, eq, sign);
    write_vcc_hi(rsp->cp2.flags[RSP_VCC].e, zero);
    write_vcc_lo(rsp->cp2.flags[RSP_VCC].e, le);
    write_vco_hi(rsp->cp2.flags[RSP_VCO].e, zero);
    write_vco_lo(rsp->cp2.flags[RSP_VCO].e, zero);
    write_acc_lo(acc, result);
    return result;
}
rsp_vect_t RSP_VINVALID(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    return zero;
}
rsp_vect_t RSP_VMACF_VMACU(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc = rsp->cp2.acc.e;
    rsp_vect_t acc_lo, acc_md, acc_hi, result;
    acc_lo = read_acc_lo(acc);
    acc_md = read_acc_md(acc);
    acc_hi = read_acc_hi(acc);
    result = rsp_vmacf_vmacu(iw, vs, vt_shuffle, zero, &acc_lo, &acc_md, &acc_hi);
    write_acc_lo(acc, acc_lo);
    write_acc_md(acc, acc_md);
    write_acc_hi(acc, acc_hi);
    return result;
}
rsp_vect_t RSP_VMADH_VMUDH(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc = rsp->cp2.acc.e;
    rsp_vect_t acc_lo, acc_md, acc_hi, result;
    acc_lo = read_acc_lo(acc);
    acc_md = read_acc_md(acc);
    acc_hi = read_acc_hi(acc);
    result = rsp_vmadh_vmudh(iw, vs, vt_shuffle, zero, &acc_lo, &acc_md, &acc_hi);
    write_acc_lo(acc, acc_lo);
    write_acc_md(acc, acc_md);
    write_acc_hi(acc, acc_hi);
    return result;
}
rsp_vect_t RSP_VMADL_VMUDL(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc = rsp->cp2.acc.e;
    rsp_vect_t acc_lo, acc_md, acc_hi, result;
    acc_lo = read_acc_lo(acc);
    acc_md = read_acc_md(acc);
    acc_hi = read_acc_hi(acc);
    result = rsp_vmadl_vmudl(iw, vs, vt_shuffle, zero, &acc_lo, &acc_md, &acc_hi);
    write_acc_lo(acc, acc_lo);
    write_acc_md(acc, acc_md);
    write_acc_hi(acc, acc_hi);
    return result;
}
rsp_vect_t RSP_VMADM_VMUDM(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc = rsp->cp2.acc.e;
    rsp_vect_t acc_lo, acc_md, acc_hi, result;
    acc_lo = read_acc_lo(acc);
    acc_md = read_acc_md(acc);
    acc_hi = read_acc_hi(acc);
    result = rsp_vmadm_vmudm(iw, vs, vt_shuffle, zero, &acc_lo, &acc_md, &acc_hi);
    write_acc_lo(acc, acc_lo);
    write_acc_md(acc, acc_md);
    write_acc_hi(acc, acc_hi);
    return result;
}
rsp_vect_t RSP_VMADN_VMUDN(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc = rsp->cp2.acc.e;
    rsp_vect_t acc_lo, acc_md, acc_hi, result;
    acc_lo = read_acc_lo(acc);
    acc_md = read_acc_md(acc);
    acc_hi = read_acc_hi(acc);
    result = rsp_vmadn_vmudn(iw, vs, vt_shuffle, zero, &acc_lo, &acc_md, &acc_hi);
    write_acc_lo(acc, acc_lo);
    write_acc_md(acc, acc_md);
    write_acc_hi(acc, acc_hi);
    return result;
}
__m128i rsp_vmov(struct rsp *rsp, unsigned src, unsigned e, unsigned dest, rsp_vect_t vt_shuffle)
{
    uint16_t data;
    memcpy(&data, (e & 0x7) + (uint16_t *)&vt_shuffle, sizeof(uint16_t));
    rsp->cp2.regs[dest].e[e & 0x7] = data;
    return rsp_vect_load_unshuffled_operand(rsp->cp2.regs[dest].e);
}
rsp_vect_t RSP_VMOV(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t *acc  = rsp->cp2.acc.e;
    unsigned  e    = ((iw) >> 11 & 0x1F) & 0x7;
    unsigned  dest = ((iw) >> 6 & 0x1F);
    unsigned  src  = ((iw) >> 16 & 0x1F);
    write_acc_lo(acc, vt_shuffle);
    return rsp_vmov(rsp, src, e, dest, vt_shuffle);
}
rsp_vect_t RSP_VMRG(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc = rsp->cp2.acc.e;
    rsp_vect_t le;
    le                = read_vcc_lo(rsp->cp2.flags[RSP_VCC].e);
    rsp_vect_t result = rsp_vmrg(vs, vt_shuffle, le);
    write_vco_hi(rsp->cp2.flags[RSP_VCO].e, zero);
    write_vco_lo(rsp->cp2.flags[RSP_VCO].e, zero);
    write_acc_lo(acc, result);
    return result;
}
rsp_vect_t RSP_VMULF_VMULU(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc = rsp->cp2.acc.e;
    rsp_vect_t acc_lo, acc_md, acc_hi, result;
    result = rsp_vmulf_vmulu(iw, vs, vt_shuffle, zero, &acc_lo, &acc_md, &acc_hi);
    write_acc_lo(acc, acc_lo);
    write_acc_md(acc, acc_md);
    write_acc_hi(acc, acc_hi);
    return result;
}
rsp_vect_t RSP_VNOP(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    return rsp_vect_load_unshuffled_operand(rsp->cp2.regs[((iw) >> 6 & 0x1F)].e);
}
rsp_vect_t RSP_VOR_VNOR(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc    = rsp->cp2.acc.e;
    rsp_vect_t result = rsp_vor_vnor(iw, vs, vt_shuffle);
    write_acc_lo(acc, result);
    return result;
}
__m128i rsp_vrcp_vrsq(struct rsp *rsp, uint32_t iw, int dp, unsigned src, unsigned e, unsigned dest, unsigned de)
{
    uint32_t dp_input, sp_input;
    int32_t  input, result;
    int16_t  vt;
    int32_t  input_mask, data;
    unsigned shift, idx;
    vt         = rsp->cp2.regs[src].e[e & 0x7];
    dp_input   = ((uint32_t)rsp->cp2.div_in << 16) | (uint16_t)vt;
    sp_input   = vt;
    input      = (dp) ? dp_input : sp_input;
    input_mask = input >> 31;
    data       = input ^ input_mask;
    if (input > -32768)
        data -= input_mask;
    if (data == 0)
        result = 0x7fffFFFFU;
    else if (input == -32768)
        result = 0xffff0000U;
    else {
        shift = __builtin_clz(data);
        if (iw & 0x4) {
            idx    = (((unsigned long long)data << shift) & 0x7FC00000U) >> 22;
            idx    = ((idx | 0x200) & 0x3FE) | (shift % 2);
            result = rsp_reciprocal_rom[idx];
            result = ((0x10000 | result) << 14) >> ((31 - shift) >> 1);
        } else {
            idx    = (((unsigned long long)data << shift) & 0x7FC00000U) >> 22;
            result = rsp_reciprocal_rom[idx];
            result = ((0x10000 | result) << 14) >> (31 - shift);
        }
        result = result ^ input_mask;
    }
    rsp->cp2.div_out                = result >> 16;
    rsp->cp2.regs[dest].e[de & 0x7] = result;
    return rsp_vect_load_unshuffled_operand(rsp->cp2.regs[dest].e);
}
rsp_vect_t RSP_VRCP_VRSQ(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t *acc  = rsp->cp2.acc.e;
    unsigned  de   = ((iw) >> 11 & 0x1F) & 0x7;
    unsigned  e    = ((iw) >> 21 & 0xF) & 0x7;
    unsigned  dest = ((iw) >> 6 & 0x1F);
    unsigned  src  = ((iw) >> 16 & 0x1F);
    write_acc_lo(acc, vt_shuffle);
    int dp           = iw & rsp->cp2.dp_flag;
    rsp->cp2.dp_flag = 0;
    return rsp_vrcp_vrsq(rsp, iw, dp, src, e, dest, de);
}
__m128i rsp_vdivh(struct rsp *rsp, unsigned src, unsigned e, unsigned dest, unsigned de)
{
    rsp->cp2.div_in                 = rsp->cp2.regs[src].e[e & 0x7];
    rsp->cp2.regs[dest].e[de & 0x7] = rsp->cp2.div_out;
    return rsp_vect_load_unshuffled_operand(rsp->cp2.regs[dest].e);
}
rsp_vect_t RSP_VRCPH_VRSQH(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t *acc  = rsp->cp2.acc.e;
    unsigned  de   = ((iw) >> 11 & 0x1F) & 0x7;
    unsigned  e    = ((iw) >> 21 & 0xF) & 0x7;
    unsigned  dest = ((iw) >> 6 & 0x1F);
    unsigned  src  = ((iw) >> 16 & 0x1F);
    write_acc_lo(acc, vt_shuffle);
    rsp->cp2.dp_flag = 1;
    return rsp_vdivh(rsp, src, e, dest, de);
}
rsp_vect_t RSP_VSAR(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t *acc = rsp->cp2.acc.e;
    unsigned  e   = ((iw) >> 21 & 0xF);
    switch (e) {
        case 8:
            return read_acc_hi(acc);
        case 9:
            return read_acc_md(acc);
        case 10:
            return read_acc_lo(acc);
        default:
            return zero;
    }
    return zero;
}
rsp_vect_t RSP_VSUB(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc = rsp->cp2.acc.e;
    rsp_vect_t carry, acc_lo;
    carry             = read_vco_lo(rsp->cp2.flags[RSP_VCO].e);
    rsp_vect_t result = rsp_vsub(vs, vt_shuffle, carry, &acc_lo);
    write_vco_hi(rsp->cp2.flags[RSP_VCO].e, zero);
    write_vco_lo(rsp->cp2.flags[RSP_VCO].e, zero);
    write_acc_lo(acc, acc_lo);
    return result;
}
rsp_vect_t RSP_VSUBC(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc = rsp->cp2.acc.e;
    rsp_vect_t eq, sn;
    rsp_vect_t result = rsp_vsubc(vs, vt_shuffle, zero, &eq, &sn);
    write_vco_hi(rsp->cp2.flags[RSP_VCO].e, eq);
    write_vco_lo(rsp->cp2.flags[RSP_VCO].e, sn);
    write_acc_lo(acc, result);
    return result;
}
rsp_vect_t RSP_VXOR_VNXOR(struct rsp *rsp, uint32_t iw, rsp_vect_t vt_shuffle, rsp_vect_t vs, rsp_vect_t zero)
{
    uint16_t  *acc    = rsp->cp2.acc.e;
    rsp_vect_t result = rsp_vxor_vnxor(iw, vs, vt_shuffle);
    write_acc_lo(acc, result);
    return result;
}
const rsp_vector_function rsp_vector_function_table[NUM_RSP_VECTOR_OPCODES] __attribute__((aligned(64))) = {
    (RSP_VINVALID),
    (RSP_VABS),
    (RSP_VADD),
    (RSP_VADDC),
    (RSP_VAND_VNAND),
    (RSP_VCH),
    (RSP_VCL),
    (RSP_VCR),
    (RSP_VEQ_VGE_VLT_VNE),
    (RSP_VEQ_VGE_VLT_VNE),
    (RSP_VEQ_VGE_VLT_VNE),
    (RSP_VMACF_VMACU),
    (RSP_VINVALID),
    (RSP_VMACF_VMACU),
    (RSP_VMADH_VMUDH),
    (RSP_VMADL_VMUDL),
    (RSP_VMADM_VMUDM),
    (RSP_VMADN_VMUDN),
    (RSP_VMOV),
    (RSP_VMRG),
    (RSP_VMADH_VMUDH),
    (RSP_VMADL_VMUDL),
    (RSP_VMADM_VMUDM),
    (RSP_VMADN_VMUDN),
    (RSP_VMULF_VMULU),
    (RSP_VINVALID),
    (RSP_VMULF_VMULU),
    (RSP_VAND_VNAND),
    (RSP_VEQ_VGE_VLT_VNE),
    (RSP_VNOP),
    (RSP_VOR_VNOR),
    (RSP_VNOP),
    (RSP_VXOR_VNXOR),
    (RSP_VOR_VNOR),
    (RSP_VRCP_VRSQ),
    (RSP_VRCPH_VRSQH),
    (RSP_VRCP_VRSQ),
    (RSP_VINVALID),
    (RSP_VINVALID),
    (RSP_VRCP_VRSQ),
    (RSP_VRCPH_VRSQH),
    (RSP_VRCP_VRSQ),
    (RSP_VSAR),
    (RSP_VSUB),
    (RSP_VSUBC),
    (RSP_VXOR_VNXOR),
};
void RSP_MFC0(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    unsigned               dest;
    dest                      = ((iw) >> 16 & 0x1F);
    rt                        = rsp_read_cp0_reg(rsp, ((iw) >> 11 & 0x1F) & 0x0f);
    exdf_latch->result.result = rt;
    exdf_latch->result.dest   = dest;
}
void RSP_MTC0(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    unsigned dest;
    dest = ((iw) >> 11 & 0x1F);
    rsp_write_cp0_reg(rsp, dest & 0x0f, rt);
}
uint32_t rsp_read_cp0_reg(struct rsp *rsp, unsigned src)
{
    uint32_t word;
    src = SP_REGISTER_OFFSET + src;
    switch (src) {
        case RSP_CP0_REGISTER_SP_STATUS:
            return *((volatile uint32_t *)&rsp->regs[RSP_CP0_REGISTER_SP_STATUS]);
        case RSP_CP0_REGISTER_SP_RESERVED:
            return !__sync_bool_compare_and_swap(&rsp->regs[RSP_CP0_REGISTER_SP_RESERVED], 0, 1);
        case RSP_CP0_REGISTER_DMA_FULL:
        case RSP_CP0_REGISTER_DMA_BUSY:
            return 0;
        case RSP_CP0_REGISTER_CMD_START:
        case RSP_CP0_REGISTER_CMD_END:
        case RSP_CP0_REGISTER_CMD_CURRENT:
        case RSP_CP0_REGISTER_CMD_STATUS:
        case RSP_CP0_REGISTER_CMD_CLOCK:
        case RSP_CP0_REGISTER_CMD_BUSY:
        case RSP_CP0_REGISTER_CMD_PIPE_BUSY:
        case RSP_CP0_REGISTER_CMD_TMEM_BUSY:
            src -= RSP_CP0_REGISTER_CMD_START;
            read_dp_regs(rsp->bus->rdp, 0x04100000 + src * 4, &word);
            return word;
        default:
            return rsp->regs[src];
    }
    return 0;
}
void rsp_status_write(struct rsp *rsp, uint32_t rt)
{
    uint32_t prev_status, status;
    do {
        prev_status = rsp->regs[RSP_CP0_REGISTER_SP_STATUS];
        status      = prev_status;
        if ((rt & 0x00000001) && (status & 0x0001)) {
            uint32_t pc = rsp->pipeline.rdex_latch.common.pc;
            rsp_pipeline_init(&rsp->pipeline);
            rsp->pipeline.ifrd_latch.pc = pc;
            status &= ~0x0001;
        } else if (rt & 0x00000002)
            status |= 0x0001;
        if (rt & 0x00000004)
            status &= ~0x0002;
        else if (rt & 0x80000000)
            status |= 0x0002;
        if (rt & 0x00000008)
            g_vr4300->clear_rcp_interrupt(MI_INTR_SP);
        else if (rt & 0x00000010)
            g_vr4300->signal_rcp_interrupt(MI_INTR_SP);
        if (rt & 0x00000020)
            status &= ~0x0020;
        else if (rt & 0x00000040)
            status |= 0x0020;
        if (rt & 0x00000080)
            status &= ~0x0040;
        else if (rt & 0x00000100)
            status |= 0x0040;
        if (rt & 0x00000200)
            status &= ~0x0080;
        else if (rt & 0x00000400)
            status |= 0x0080;
        if (rt & 0x00000800)
            status &= ~0x0100;
        else if (rt & 0x00001000)
            status |= 0x0100;
        if (rt & 0x00002000)
            status &= ~0x0200;
        else if (rt & 0x00004000)
            status |= 0x0200;
        if (rt & 0x00008000)
            status &= ~0x0400;
        else if (rt & 0x00010000)
            status |= 0x0400;
        if (rt & 0x00020000)
            status &= ~0x0800;
        else if (rt & 0x00040000)
            status |= 0x0800;
        if (rt & 0x00080000)
            status &= ~0x1000;
        else if (rt & 0x00100000)
            status |= 0x1000;
        if (rt & 0x00200000)
            status &= ~0x2000;
        else if (rt & 0x00400000)
            status |= 0x2000;
        if (rt & 0x00800000)
            status &= ~0x4000;
        else if (rt & 0x01000000)
            status |= 0x4000;
    } while (!__sync_bool_compare_and_swap(&rsp->regs[RSP_CP0_REGISTER_SP_STATUS], prev_status, status));
}
void rsp_write_cp0_reg(struct rsp *rsp, unsigned dest, uint32_t rt)
{
    dest = SP_REGISTER_OFFSET + dest;
    switch (dest) {
        case RSP_CP0_REGISTER_DMA_CACHE:
            rsp->regs[RSP_CP0_REGISTER_DMA_CACHE] = rt & 0x1FFF;
            break;
        case RSP_CP0_REGISTER_DMA_DRAM:
            rsp->regs[RSP_CP0_REGISTER_DMA_DRAM] = rt & 0xFFFFFF;
            break;
        case RSP_CP0_REGISTER_DMA_READ_LENGTH:
            rsp->regs[RSP_CP0_REGISTER_DMA_READ_LENGTH] = rt;
            rsp_dma_read(rsp);
            break;
        case RSP_CP0_REGISTER_DMA_WRITE_LENGTH:
            rsp->regs[RSP_CP0_REGISTER_DMA_WRITE_LENGTH] = rt;
            rsp_dma_write(rsp);
            break;
        case RSP_CP0_REGISTER_SP_STATUS:
            rsp_status_write(rsp, rt & ~0x80000000);
            break;
        case RSP_CP0_REGISTER_SP_RESERVED:
            if (rt == 0) {
                *((volatile uint32_t *)&rsp->regs[RSP_CP0_REGISTER_SP_RESERVED]) = 0;
                __asm__ __volatile__("" ::: "memory");
            }
            break;
        case RSP_CP0_REGISTER_CMD_START:
        case RSP_CP0_REGISTER_CMD_END:
        case RSP_CP0_REGISTER_CMD_CURRENT:
        case RSP_CP0_REGISTER_CMD_STATUS:
        case RSP_CP0_REGISTER_CMD_CLOCK:
        case RSP_CP0_REGISTER_CMD_BUSY:
        case RSP_CP0_REGISTER_CMD_PIPE_BUSY:
        case RSP_CP0_REGISTER_CMD_TMEM_BUSY:
            dest -= RSP_CP0_REGISTER_CMD_START;
            write_dp_regs(rsp->bus->rdp, 0x04100000 + 4 * dest, rt, ~0);
            break;
        default:
            rsp->regs[dest] = rt;
            break;
    }
}
void rsp_cp0_init(struct rsp *rsp)
{
    rsp->regs[RSP_CP0_REGISTER_SP_STATUS] = 0x0001;
}
void RSP_CFC2(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    struct rsp_cp2        *cp2        = &rsp->cp2;
    unsigned               rd, dest, src;
    dest = ((iw) >> 16 & 0x1F);
    rd   = ((iw) >> 11 & 0x1F);
    if ((src = rd & 0x3) == 0x3)
        src = 2;
    exdf_latch->result.result = rsp_get_flags(cp2->flags[src].e);
    exdf_latch->result.dest   = dest;
}
void RSP_CTC2(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_cp2 *cp2 = &rsp->cp2;
    unsigned        rd, dest;
    rd = ((iw) >> 11 & 0x1F);
    if ((dest = rd & 0x3) >= 0x2) {
        rt &= 0xFF;
        dest = 2;
    }
    rsp_set_flags(cp2->flags[dest].e, rt);
}
void RSP_MFC2(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    const uint16_t        *e          = rsp->cp2.regs[((iw) >> 11 & 0x1F)].e;
    unsigned               dest, element = ((iw) >> 7 & 0xF);
    unsigned               lo = element >> 1;
    uint32_t               data;
    uint16_t               high;
    uint8_t                low;
    dest = ((iw) >> 16 & 0x1F);
    if (element & 0x1) {
        unsigned hi = (element + 1) >> 1;
        high        = e[lo] << 8;
        low         = e[hi] >> 8;
        data        = (int16_t)(high | low);
    } else
        data = (int16_t)e[lo];
    exdf_latch->result.result = data;
    exdf_latch->result.dest   = dest;
}
void RSP_MTC2(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    uint16_t *e       = rsp->cp2.regs[((iw) >> 11 & 0x1F)].e;
    unsigned  element = ((iw) >> 7 & 0xF);
    unsigned  lo      = element >> 1;
    if (element & 0x1) {
        unsigned hi = (element + 1) >> 1;
        e[lo]       = (e[lo] & 0xFF00) | (rt >> 8 & 0xFF);
        e[hi]       = (e[hi] & 0x00FF) | ((rt & 0xFF) << 8);
    } else
        e[lo] = rt;
}
void rsp_cp2_init(struct rsp *rsp)
{
}
static inline uint32_t rsp_addsub_mask(uint32_t iw)
{
    uint32_t mask;
    __asm__("shr $2,       %k[iwiw];"
            "sbb %k[mask], %k[mask];"
            : [mask] "=r"(mask), [iwiw] "+r"(iw)
            :
            : "cc");
    return mask;
}
static const uint16_t rsp_bdls_lut[2][4][4] __attribute__((aligned(64))) = {{
                                                                                {0x00FF, 0x0000, 0x0000, 0x0000},
                                                                                {0xFFFF, 0x0000, 0x0000, 0x0000},
                                                                                {0xFFFF, 0xFFFF, 0x0000, 0x0000},
                                                                                {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
                                                                            },
                                                                            {
                                                                                {0xFF00, 0x0000, 0x0000, 0x0000},
                                                                                {0xFFFF, 0x0000, 0x0000, 0x0000},
                                                                                {0xFFFF, 0xFFFF, 0x0000, 0x0000},
                                                                                {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
                                                                            }};

static const uint16_t rsp_qr_lut[16][8]

    __attribute__((aligned(64))) = {{0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
                                    {0xFF00, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
                                    {0x0000, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
                                    {0x0000, 0xFF00, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
                                    {0x0000, 0x0000, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
                                    {0x0000, 0x0000, 0xFF00, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
                                    {0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
                                    {0x0000, 0x0000, 0x0000, 0xFF00, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
                                    {0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
                                    {0x0000, 0x0000, 0x0000, 0x0000, 0xFF00, 0xFFFF, 0xFFFF, 0xFFFF},
                                    {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0xFFFF},
                                    {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFF00, 0xFFFF, 0xFFFF},
                                    {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF},
                                    {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFF00, 0xFFFF},
                                    {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF},
                                    {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFF00}};

static inline uint32_t rsp_branch_mask(uint32_t iw, unsigned index)
{
    iw = (uint32_t)((int32_t)(iw << (31 - index)) >> 31);
    return ~iw;
}
static const uint32_t rsp_load_sex_mask[8] __attribute__((aligned(32))) = {
    ~0U, ~0U, 0U, ~0U, 0xFFU, 0xFFFFU, 0U, ~0U,
};
static inline unsigned sign_extend_6(int i)
{
    return (i << (32 - 7)) >> (32 - 7);
}
void RSP_ADDIU_LUI_SUBIU(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    unsigned               immshift   = iw >> 24 & 0x10;
    unsigned               dest;
    dest                      = ((iw) >> 16 & 0x1F);
    rt                        = (int16_t)iw;
    rt                        = rs + (rt << immshift);
    exdf_latch->result.result = rt;
    exdf_latch->result.dest   = dest;
}
void RSP_ADDU_SUBU(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    uint32_t               mask       = rsp_addsub_mask(iw);
    unsigned               dest;
    uint32_t               rd;
    dest                      = ((iw) >> 11 & 0x1F);
    rt                        = (rt ^ mask) - mask;
    rd                        = rs + rt;
    exdf_latch->result.result = rd;
    exdf_latch->result.dest   = dest;
}
void RSP_AND_OR_XOR(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    unsigned               dest;
    uint32_t               rd, rand, rxor;
    dest = ((iw) >> 11 & 0x1F);
    rand = rs & rt;
    rxor = rs ^ rt;
    rd   = rand + rxor;
    if ((iw & 1) == 0)
        rd = rxor;
    if ((iw & 3) == 0)
        rd = rand;
    exdf_latch->result.result = rd;
    exdf_latch->result.dest   = dest;
}
void RSP_ANDI_ORI_XORI(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    unsigned               dest;
    uint32_t               rd, rand, rxor;
    dest = ((iw) >> 16 & 0x1F);
    rt   = (uint16_t)iw;
    rand = rs & rt;
    rxor = rs ^ rt;
    rd   = rand + rxor;
    if ((iw & 67108864) == 0)
        rd = rxor;
    if ((iw & 201326592) == 0)
        rd = rand;
    exdf_latch->result.result = rd;
    exdf_latch->result.dest   = dest;
}
void RSP_BEQ_BNE(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_ifrd_latch *ifrd_latch = &rsp->pipeline.ifrd_latch;
    struct rsp_rdex_latch *rdex_latch = &rsp->pipeline.rdex_latch;
    uint32_t               offset     = (uint32_t)((int16_t)iw) << 2;
    bool                   is_ne      = iw >> 26 & 0x1;
    bool                   cmp        = rs == rt;
    if (cmp == is_ne)
        return;
    ifrd_latch->pc = (rdex_latch->common.pc + offset + 4) & 0xFFC;
}
void RSP_BGEZ_BLTZ(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t __attribute__((unused)) rt)
{
    struct rsp_ifrd_latch *ifrd_latch = &rsp->pipeline.ifrd_latch;
    struct rsp_rdex_latch *rdex_latch = &rsp->pipeline.rdex_latch;
    uint32_t               offset     = (uint32_t)((int16_t)iw) << 2;
    bool                   is_ge      = iw >> 16 & 0x1;
    bool                   cmp        = (int32_t)rs < 0;
    if (cmp == is_ge)
        return;
    ifrd_latch->pc = (rdex_latch->common.pc + offset + 4) & 0xFFC;
}
void RSP_BGEZAL_BLTZAL(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t __attribute__((unused)) rt)
{
    struct rsp_ifrd_latch *ifrd_latch = &rsp->pipeline.ifrd_latch;
    struct rsp_rdex_latch *rdex_latch = &rsp->pipeline.rdex_latch;
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    uint32_t               offset     = (uint32_t)((int16_t)iw) << 2;
    bool                   is_ge      = iw >> 16 & 0x1;
    bool                   cmp        = (int32_t)rs < 0;
    exdf_latch->result.result         = rdex_latch->common.pc + 8;
    exdf_latch->result.dest           = RSP_REGISTER_RA;
    if (cmp == is_ge)
        return;
    ifrd_latch->pc = (rdex_latch->common.pc + offset + 4) & 0xFFC;
}
void RSP_BGTZ_BLEZ(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t __attribute__((unused)) rt)
{
    struct rsp_ifrd_latch *ifrd_latch = &rsp->pipeline.ifrd_latch;
    struct rsp_rdex_latch *rdex_latch = &rsp->pipeline.rdex_latch;
    uint32_t               offset     = (uint32_t)((int16_t)iw) << 2;
    bool                   is_gt      = iw >> 26 & 0x1;
    bool                   cmp        = (int32_t)rs <= 0;
    if (cmp == is_gt)
        return;
    ifrd_latch->pc = (rdex_latch->common.pc + offset + 4) & 0xFFC;
}
void RSP_BREAK(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_dfwb_latch *dfwb_latch = &rsp->pipeline.dfwb_latch;
    if (rsp->regs[RSP_CP0_REGISTER_SP_STATUS] & 0x0040)
        g_vr4300->signal_rcp_interrupt(MI_INTR_SP);
    dfwb_latch->result.dest   = RSP_CP0_REGISTER_SP_STATUS;
    dfwb_latch->result.result = 0x00000002 | 0x80000000;
}
__attribute__((hot)) void RSP_INT_MEM(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch       = &rsp->pipeline.exdf_latch;
    uint32_t               address          = rs + (int16_t)iw;
    uint32_t               sel_mask         = (int32_t)(iw << 2) >> 31;
    unsigned               request_index    = (iw >> 26 & 0x7);
    uint32_t               rdqm             = rsp_load_sex_mask[request_index];
    unsigned               request_size     = request_index & 0x3;
    unsigned               lshiftamt        = (3 - request_size) << 3;
    uint32_t               wdqm             = ~0U << lshiftamt;
    exdf_latch->request.addr                = address;
    exdf_latch->request.packet.p_int.data   = rt << lshiftamt;
    exdf_latch->request.packet.p_int.rdqm   = rdqm;
    exdf_latch->request.type                = RSP_MEM_REQUEST_INT_MEM;
    exdf_latch->request.packet.p_int.rshift = lshiftamt;
    exdf_latch->request.packet.p_int.wdqm   = sel_mask & wdqm;
    exdf_latch->result.dest                 = ~sel_mask & ((iw) >> 16 & 0x1F);
}
void RSP_INVALID(struct rsp *rsp, uint32_t iw, uint32_t __attribute__((unused)) rs, uint32_t __attribute__((unused)) rt)
{
}
void RSP_J_JAL(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_ifrd_latch *ifrd_latch = &rsp->pipeline.ifrd_latch;
    struct rsp_rdex_latch *rdex_latch = &rsp->pipeline.rdex_latch;
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    uint32_t               target     = iw << 2 & 0xFFC;
    uint32_t               mask       = rsp_branch_mask(iw, 26);
    exdf_latch->result.result         = (rdex_latch->common.pc + 8) & 0xFFC;
    exdf_latch->result.dest           = RSP_REGISTER_RA & ~mask;
    ifrd_latch->pc                    = target;
}
void RSP_JALR_JR(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t __attribute__((unused)) rt)
{
    struct rsp_ifrd_latch *ifrd_latch = &rsp->pipeline.ifrd_latch;
    struct rsp_rdex_latch *rdex_latch = &rsp->pipeline.rdex_latch;
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    uint32_t               mask       = rsp_branch_mask(iw, 0);
    unsigned               rd         = ((iw) >> 11 & 0x1F);
    exdf_latch->result.result         = (rdex_latch->common.pc + 8) & 0xFFC;
    exdf_latch->result.dest           = rd & ~mask;
    ifrd_latch->pc                    = rs & 0xFFC;
}
void RSP_LBDLSV_SBDLSV(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch    = &rsp->pipeline.exdf_latch;
    unsigned               shift_and_idx = iw >> 11 & 0x3;
    unsigned               op            = iw >> 29 & 0x1;
    unsigned               dest          = ((iw) >> 16 & 0x1F);
    exdf_latch->request.addr             = rs + (sign_extend_6(iw) << shift_and_idx);
    __m128i vdqm                         = _mm_loadl_epi64((__m128i *)(rsp_bdls_lut[op][shift_and_idx]));
    _mm_store_si128((__m128i *)exdf_latch->request.packet.p_vect.vdqm.e, vdqm);
    exdf_latch->request.packet.p_vect.element    = ((iw) >> 7 & 0xF);
    exdf_latch->request.type                     = RSP_MEM_REQUEST_VECTOR;
    exdf_latch->request.packet.p_vect.vldst_func = op ? rsp_vstore_group1 : rsp_vload_group1;
    exdf_latch->request.packet.p_vect.dest       = dest;
}
void RSP_LFHPUV_SFHPUV(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch                 *exdf_latch       = &rsp->pipeline.exdf_latch;
    unsigned                               dest             = ((iw) >> 16 & 0x1F);
    static const enum rsp_mem_request_type fhpu_type_lut[4] = {RSP_MEM_REQUEST_PACK, RSP_MEM_REQUEST_UPACK,
                                                               RSP_MEM_REQUEST_HALF, RSP_MEM_REQUEST_FOURTH};
    exdf_latch->request.addr                                = rs + (sign_extend_6(iw) << 3);
    exdf_latch->request.type                                = fhpu_type_lut[(iw >> 11 & 0x1F) - 6];
    exdf_latch->request.packet.p_vect.vldst_func            = (iw >> 29 & 0x1) ? rsp_vstore_group2 : rsp_vload_group2;
    exdf_latch->request.packet.p_vect.dest                  = dest;
}
void RSP_LQRV_SQRV(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    unsigned               op         = iw >> 29 & 0x1;
    unsigned               dest       = ((iw) >> 16 & 0x1F);
    exdf_latch->request.addr          = rs + (sign_extend_6(iw) << 4);
    memcpy(exdf_latch->request.packet.p_vect.vdqm.e, rsp_qr_lut[exdf_latch->request.addr & 0xF],
           sizeof(exdf_latch->request.packet.p_vect.vdqm.e));
    exdf_latch->request.packet.p_vect.element    = ((iw) >> 7 & 0xF);
    exdf_latch->request.type                     = (iw >> 11 & 0x1) ? RSP_MEM_REQUEST_REST : RSP_MEM_REQUEST_QUAD;
    exdf_latch->request.packet.p_vect.vldst_func = op ? rsp_vstore_group4 : rsp_vload_group4;
    exdf_latch->request.packet.p_vect.dest       = dest;
}
void RSP_LTV_STV(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch                     = &rsp->pipeline.exdf_latch;
    unsigned               op                             = iw >> 29 & 0x1;
    exdf_latch->request.addr                              = rs + (sign_extend_6(iw) << 4);
    exdf_latch->request.type                              = RSP_MEM_REQUEST_TRANSPOSE;
    exdf_latch->request.packet.p_transpose.vt             = ((iw) >> 16 & 0x1F) & 0x18;
    exdf_latch->request.packet.p_transpose.element        = ((iw) >> 7 & 0xF) >> 1;
    exdf_latch->request.packet.p_transpose.transpose_func = op ? rsp_stv : rsp_ltv;
}
void RSP_NOR(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    unsigned               dest       = ((iw) >> 11 & 0x1F);
    exdf_latch->result.result         = ~(rs | rt);
    exdf_latch->result.dest           = dest;
}
void RSP_SLL_SLLV(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    unsigned               dest       = ((iw) >> 11 & 0x1F);
    unsigned               sa         = (rs & 0x1F) + (iw >> 6 & 0x1F);
    exdf_latch->result.result         = rt << sa;
    exdf_latch->result.dest           = dest;
}
void RSP_SLT(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    unsigned               dest       = ((iw) >> 11 & 0x1F);
    exdf_latch->result.result         = (int32_t)rs < (int32_t)rt;
    exdf_latch->result.dest           = dest;
}
void RSP_SLTI(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    unsigned               dest       = ((iw) >> 16 & 0x1F);
    int32_t                imm        = (int16_t)iw;
    exdf_latch->result.result         = (int32_t)rs < imm;
    exdf_latch->result.dest           = dest;
}
void RSP_SLTIU(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    unsigned               dest       = ((iw) >> 16 & 0x1F);
    uint32_t               imm        = (int16_t)iw;
    exdf_latch->result.result         = rs < imm;
    exdf_latch->result.dest           = dest;
}
void RSP_SLTU(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    unsigned               dest       = ((iw) >> 11 & 0x1F);
    exdf_latch->result.result         = rs < rt;
    exdf_latch->result.dest           = dest;
}
void RSP_SRA(struct rsp *rsp, uint32_t iw, uint32_t __attribute__((unused)) rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    unsigned               dest       = ((iw) >> 11 & 0x1F);
    unsigned               sa         = iw >> 6 & 0x1F;
    exdf_latch->result.result         = (int32_t)rt >> sa;
    exdf_latch->result.dest           = dest;
}
void RSP_SRAV(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    unsigned               dest       = ((iw) >> 11 & 0x1F);
    unsigned               sa         = rs & 0x1F;
    exdf_latch->result.result         = (int32_t)rt >> sa;
    exdf_latch->result.dest           = dest;
}
void RSP_SRL(struct rsp *rsp, uint32_t iw, uint32_t __attribute__((unused)) rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    unsigned               dest       = ((iw) >> 11 & 0x1F);
    unsigned               sa         = iw >> 6 & 0x1F;
    exdf_latch->result.result         = rt >> sa;
    exdf_latch->result.dest           = dest;
}
void RSP_SRLV(struct rsp *rsp, uint32_t iw, uint32_t rs, uint32_t rt)
{
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    unsigned               dest       = ((iw) >> 11 & 0x1F);
    unsigned               sa         = rs & 0x1F;
    exdf_latch->result.result         = rt >> sa;
    exdf_latch->result.dest           = dest;
}
const rsp_function rsp_function_table[NUM_RSP_OPCODES] __attribute__((aligned(64))) = {
    (RSP_INVALID),
    (RSP_ADDU_SUBU),
    (RSP_ADDIU_LUI_SUBIU),
    (RSP_AND_OR_XOR),
    (RSP_ANDI_ORI_XORI),
    (RSP_BEQ_BNE),
    (RSP_BGEZ_BLTZ),
    (RSP_BGEZAL_BLTZAL),
    (RSP_BGTZ_BLEZ),
    (RSP_BGTZ_BLEZ),
    (RSP_BGEZ_BLTZ),
    (RSP_BGEZAL_BLTZAL),
    (RSP_BEQ_BNE),
    (RSP_BREAK),
    (RSP_CFC2),
    (RSP_CTC2),
    (RSP_J_JAL),
    (RSP_J_JAL),
    (RSP_JALR_JR),
    (RSP_JALR_JR),
    (RSP_INT_MEM),
    (RSP_INT_MEM),
    (RSP_LBDLSV_SBDLSV),
    (RSP_LBDLSV_SBDLSV),
    (RSP_INVALID),
    (RSP_INT_MEM),
    (RSP_INT_MEM),
    (RSP_INVALID),
    (RSP_LBDLSV_SBDLSV),
    (RSP_LFHPUV_SFHPUV),
    (RSP_LQRV_SQRV),
    (RSP_LQRV_SQRV),
    (RSP_LBDLSV_SBDLSV),
    (RSP_LTV_STV),
    (RSP_ADDIU_LUI_SUBIU),
    (RSP_LFHPUV_SFHPUV),
    (RSP_INT_MEM),
    (RSP_MFC0),
    (RSP_MFC2),
    (RSP_MTC0),
    (RSP_MTC2),
    (RSP_INVALID),
    (RSP_NOR),
    (RSP_AND_OR_XOR),
    (RSP_ANDI_ORI_XORI),
    (RSP_INT_MEM),
    (RSP_LBDLSV_SBDLSV),
    (RSP_LBDLSV_SBDLSV),
    (RSP_INVALID),
    (RSP_INT_MEM),
    (RSP_INVALID),
    (RSP_SLL_SLLV),
    (RSP_SLL_SLLV),
    (RSP_SLT),
    (RSP_SLTI),
    (RSP_SLTIU),
    (RSP_SLTU),
    (RSP_LBDLSV_SBDLSV),
    (RSP_LFHPUV_SFHPUV),
    (RSP_LQRV_SQRV),
    (RSP_SRA),
    (RSP_SRAV),
    (RSP_SRL),
    (RSP_SRLV),
    (RSP_LQRV_SQRV),
    (RSP_LBDLSV_SBDLSV),
    (RSP_LTV_STV),
    (RSP_ADDU_SUBU),
    (RSP_LFHPUV_SFHPUV),
    (RSP_INT_MEM),
    (RSP_INVALID),
    (RSP_AND_OR_XOR),
    (RSP_ANDI_ORI_XORI),
};
typedef void (*pipeline_function)(struct rsp *rsp);
static inline void rsp_if_stage(struct rsp *rsp)
{
    struct rsp_ifrd_latch *ifrd_latch = &rsp->pipeline.ifrd_latch;
    uint32_t               pc         = ifrd_latch->pc;
    uint32_t               iw;
    assert(!(pc & 0x1000) || "RSP $PC points past IMEM.");
    ifrd_latch->pc = (pc + 4) & 0xFFC;
    memcpy(&iw, rsp->mem + 0x1000 + pc, sizeof(iw));
    ifrd_latch->common.pc = pc;
    ifrd_latch->opcode    = rsp->opcode_cache[pc >> 2];
    ifrd_latch->iw        = iw;
}
static inline int rsp_rd_stage(struct rsp *rsp)
{
    struct rsp_rdex_latch *rdex_latch          = &rsp->pipeline.rdex_latch;
    struct rsp_ifrd_latch *ifrd_latch          = &rsp->pipeline.ifrd_latch;
    uint32_t               previous_insn_flags = rdex_latch->opcode.flags;
    uint32_t               iw                  = ifrd_latch->iw;
    rdex_latch->common                         = ifrd_latch->common;
    rdex_latch->opcode                         = ifrd_latch->opcode;
    rdex_latch->iw                             = iw;
    if (previous_insn_flags & OPCODE_INFO_LOAD) {
        const struct rsp_opcode *opcode = &rdex_latch->opcode;
        unsigned                 dest   = rsp->pipeline.exdf_latch.result.dest;
        unsigned                 rs     = GET_RS(iw);
        unsigned                 rt     = GET_RT(iw);
        if (unlikely(dest && ((dest == rs && (opcode->flags & OPCODE_INFO_NEEDRS)) ||
                              (dest == rt && (opcode->flags & OPCODE_INFO_NEEDRT))))) {
            static const struct rsp_opcode rsp_rf_kill_op = {RSP_OPCODE_SLL, 0x0};
            rdex_latch->opcode                            = rsp_rf_kill_op;
            rdex_latch->iw                                = 0x00000000U;
            return 1;
        }
    }
    return 0;
}
static inline void rsp_ex_stage(struct rsp *rsp)
{
    struct rsp_dfwb_latch *dfwb_latch = &rsp->pipeline.dfwb_latch;
    struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
    struct rsp_rdex_latch *rdex_latch = &rsp->pipeline.rdex_latch;
    uint32_t               rs_reg, rt_reg, temp;
    unsigned               rs, rt;
    uint32_t               iw;
    exdf_latch->common = rdex_latch->common;
    if (rdex_latch->opcode.flags & OPCODE_INFO_VECTOR)
        return;
    iw                                 = rdex_latch->iw;
    rs                                 = GET_RS(iw);
    rt                                 = GET_RT(iw);
    temp                               = rsp->regs[dfwb_latch->result.dest];
    rsp->regs[dfwb_latch->result.dest] = dfwb_latch->result.result;
    rsp->regs[RSP_REGISTER_R0]         = 0x00000000U;
    rs_reg                             = rsp->regs[rs];
    rt_reg                             = rsp->regs[rt];
    rsp->regs[dfwb_latch->result.dest] = temp;
    return rsp_function_table[rdex_latch->opcode.id](rsp, iw, rs_reg, rt_reg);
}
static inline void rsp_v_ex_stage(struct rsp *rsp)
{
    struct rsp_rdex_latch *rdex_latch = &rsp->pipeline.rdex_latch;
    rsp_vect_t             vd_reg, vs_reg, vt_shuf_reg, zero;
    unsigned               vs, vt, vd, e;
    uint32_t               iw;
    if (!(rdex_latch->opcode.flags & OPCODE_INFO_VECTOR))
        return;
    iw          = rdex_latch->iw;
    vs          = GET_VS(iw);
    vt          = GET_VT(iw);
    vd          = GET_VD(iw);
    e           = GET_E(iw);
    vs_reg      = rsp_vect_load_unshuffled_operand(rsp->cp2.regs[vs].e);
    vt_shuf_reg = rsp_vect_load_and_shuffle_operand(rsp->cp2.regs[vt].e, e);
    zero        = rsp_vzero();
    vd_reg      = rsp_vector_function_table[rdex_latch->opcode.id](rsp, iw, vt_shuf_reg, vs_reg, zero);
    rsp_vect_write_operand(rsp->cp2.regs[vd].e, vd_reg);
}
static inline void rsp_df_stage(struct rsp *rsp)
{
    struct rsp_dfwb_latch        *dfwb_latch = &rsp->pipeline.dfwb_latch;
    struct rsp_exdf_latch        *exdf_latch = &rsp->pipeline.exdf_latch;
    const struct rsp_mem_request *request    = &exdf_latch->request;
    uint32_t                      addr;
    dfwb_latch->common = exdf_latch->common;
    dfwb_latch->result = exdf_latch->result;
    if (request->type == RSP_MEM_REQUEST_NONE)
        return;
    addr = request->addr & 0xFFF;
    if (request->type == RSP_MEM_REQUEST_INT_MEM) {
        uint32_t rdqm   = request->packet.p_int.rdqm;
        uint32_t wdqm   = request->packet.p_int.wdqm;
        uint32_t data   = request->packet.p_int.data;
        unsigned rshift = request->packet.p_int.rshift;
        uint32_t word;
        memcpy(&word, rsp->mem + addr, sizeof(word));
        word                      = byteswap_32(word);
        dfwb_latch->result.result = rdqm & (((int32_t)word) >> rshift);
        word                      = byteswap_32((word & ~wdqm) | (data & wdqm));
        memcpy(rsp->mem + addr, &word, sizeof(word));
    } else if (request->type == RSP_MEM_REQUEST_TRANSPOSE) {
        unsigned element = request->packet.p_transpose.element;
        unsigned vt      = request->packet.p_transpose.vt;
        exdf_latch->request.packet.p_transpose.transpose_func(rsp, addr, element, vt);
    } else {
        uint16_t  *regp    = rsp->cp2.regs[request->packet.p_vect.dest].e;
        unsigned   element = request->packet.p_vect.element;
        rsp_vect_t reg, dqm;
        reg                     = rsp_vect_load_unshuffled_operand(regp);
        dqm                     = rsp_vect_load_unshuffled_operand(exdf_latch->request.packet.p_vect.vdqm.e);
        dfwb_latch->result.dest = 0;
        exdf_latch->request.packet.p_vect.vldst_func(rsp, addr, element, regp, reg, dqm);
    }
}
static inline bool rsp_wb_stage(struct rsp *rsp)
{
    const struct rsp_dfwb_latch *dfwb_latch = &rsp->pipeline.dfwb_latch;
    if (dfwb_latch->result.dest == RSP_CP0_REGISTER_SP_STATUS) {
        rsp_status_write(rsp, dfwb_latch->result.result);
        return !(rsp->regs[RSP_CP0_REGISTER_SP_STATUS] & SP_STATUS_HALT);
    } else
        rsp->regs[dfwb_latch->result.dest] = dfwb_latch->result.result;
    return true;
}
void rsp_cycle_(struct rsp *rsp)
{
    if (unlikely(!rsp_wb_stage(rsp)))
        return;
    rsp_df_stage(rsp);
    rsp->pipeline.exdf_latch.result.dest  = RSP_REGISTER_R0;
    rsp->pipeline.exdf_latch.request.type = RSP_MEM_REQUEST_NONE;
    rsp_v_ex_stage(rsp);
    rsp_ex_stage(rsp);
    if (likely(!rsp_rd_stage(rsp)))
        rsp_if_stage(rsp);
}
void rsp_pipeline_init(struct rsp_pipeline *pipeline)
{
    memset(pipeline, 0, sizeof(*pipeline));
}
n64_align(static const struct rsp_opcode rsp_opcode_table[], CACHE_LINE_SIZE) = {
    {SLL},      {INVALID},  {SRL},      {SRA},      {SLLV},     {INVALID},  {SRLV},     {SRAV},     {JR},
    {JALR},     {INVALID},  {INVALID},  {INVALID},  {BREAK},    {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {ADDU},     {ADDU},     {SUBU},     {SUBU},
    {AND},      {OR},       {XOR},      {NOR},      {INVALID},  {INVALID},  {SLT},      {SLTU},     {INVALID},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {INVALID},  {BLTZ},     {BGEZ},     {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {BLTZAL},
    {BGEZAL},   {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {MFC0},     {INVALID},  {INVALID},
    {INVALID},  {MTC0},     {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {INVALID},  {INVALID},  {MFC2},     {INVALID},  {CFC2},     {INVALID},  {MTC2},     {INVALID},  {CTC2},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {VMULF},    {VMULU},    {VRNDP},    {VMULQ},    {VMUDL},    {VMUDM},    {VMUDN},    {VMUDH},    {VMACF},
    {VMACU},    {VRNDN},    {VMACQ},    {VMADL},    {VMADM},    {VMADN},    {VMADH},    {VADD},     {VSUB},
    {VINVALID}, {VABS},     {VADDC},    {VSUBC},    {VINVALID}, {VINVALID}, {VINVALID}, {VINVALID}, {VINVALID},
    {VINVALID}, {VINVALID}, {VSAR},     {VINVALID}, {VINVALID}, {VLT},      {VEQ},      {VNE},      {VGE},
    {VCL},      {VCH},      {VCR},      {VMRG},     {VAND},     {VNAND},    {VOR},      {VNOR},     {VXOR},
    {VNXOR},    {VINVALID}, {VINVALID}, {VRCP},     {VRCPL},    {VRCPH},    {VMOV},     {VRSQ},     {VRSQL},
    {VRSQH},    {VNOP},     {VINVALID}, {VINVALID}, {VINVALID}, {VINVALID}, {VINVALID}, {VINVALID}, {VINVALID},
    {VNULL},    {LBV},      {LSV},      {LLV},      {LDV},      {LQV},      {LRV},      {LPV},      {LUV},
    {LHV},      {LFV},      {INVALID},  {LTV},      {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {SBV},      {SSV},      {SLV},
    {SDV},      {SQV},      {SRV},      {SPV},      {SUV},      {SHV},      {SFV},      {SWV},      {STV},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {J},        {JAL},      {BEQ},      {BNE},      {BLEZ},
    {BGTZ},     {ADDIU},    {ADDIU},    {SLTI},     {SLTIU},    {ANDI},     {ORI},      {XORI},     {LUI},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {LB},       {LH},
    {INVALID},  {LW},       {LBU},      {LHU},      {INVALID},  {INVALID},  {SB},       {SH},       {INVALID},
    {SW},       {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},  {INVALID},
    {INVALID},  {INVALID},  {INVALID}};

n64_align(static const struct rsp_opcode_escape rsp_escape_table[128], CACHE_LINE_SIZE) = {
    {0, 0, 0x3F},    {0, 0, 0x3F},    {64, 16, 0x1F},  {64, 16, 0x1F},  {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {96, 21, 0x1F},  {96, 21, 0x1F},  {272, 26, 0x3F}, {272, 26, 0x3F},
    {128, 21, 0x1F}, {144, 0, 0x3F},  {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {208, 11, 0x1F}, {208, 11, 0x1F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {240, 11, 0x1F}, {240, 11, 0x1F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F}, {272, 26, 0x3F},
    {272, 26, 0x3F}, {272, 26, 0x3F},
};
const struct rsp_opcode *rsp_decode_instruction(uint32_t iw)
{
    const struct rsp_opcode_escape *escape = rsp_escape_table + (iw >> 25);
    unsigned                        index  = iw >> escape->shift & escape->mask;
    const struct rsp_opcode        *group  = rsp_opcode_table + escape->offset;
    return group + index;
}
