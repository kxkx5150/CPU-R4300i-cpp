#include "../../utils/common.h"
#include "bus.h"
#include "dd.h"
#include "pi.h"
#include "ri.h"
#include "cpu.h"
#include <assert.h>
#include <cstdint>
#include <stdlib.h>
#include <string.h>

extern vr4300 *g_vr4300;

#define IS_BUFFER_SIZE IS_VIEWER_ADDRESS_LEN - 0x20 + 1


static int pi_dma_read(struct pi_controller *pi);
static int pi_dma_write(struct pi_controller *pi);

void pi_cycle_(struct pi_controller *pi)
{
    if (pi->bytes_to_copy > 0) {
        pi->is_dma_read ? pi_dma_read(pi) : pi_dma_write(pi);
        pi->regs[PI_STATUS_REG] &= ~PI_STATUS_DMA_BUSY;
        pi->regs[PI_STATUS_REG] |= PI_STATUS_INTERRUPT;
        g_vr4300->signal_rcp_interrupt(MI_INTR_PI);
        pi->bytes_to_copy = 0;
        return;
    }
}
static int pi_dma_read(struct pi_controller *pi)
{
    uint32_t dest   = pi->regs[PI_CART_ADDR_REG] & 0xFFFFFFE;
    uint32_t source = pi->regs[PI_DRAM_ADDR_REG] & 0x7FFFFE;
    uint32_t length = (pi->regs[PI_RD_LEN_REG] & 0xFFFFFF) + 1;
    if (source & 0x7)
        length -= source & 0x7;
    if (dest >= 0x08000000 && dest < 0x10000000) {
        if (pi->sram->ptr != NULL) {
            uint32_t sram_bank     = (dest >> 18) & 3;
            uint32_t sram_offset   = (sram_bank * 0x8000) + (dest & 0x7FFF);
            uint32_t sram_bank_end = (sram_offset + length - 1) / 0x8000;
            if (sram_bank == sram_bank_end && sram_offset + length <= pi->sram->size)
                memcpy((uint8_t *)(pi->sram->ptr) + sram_offset, pi->bus->ri->ram + source, length);
        } else if (pi->flashram.data != NULL && pi->flashram.mode == FLASHRAM_WRITE)
            pi->flashram.rdram_pointer = source;
    } else if ((source & 0x05000000) == 0x05000000)
        dd_dma_read(pi->bus->dd, source, dest, length);
    pi->regs[PI_RD_LEN_REG]    = 0x7F;
    pi->regs[PI_DRAM_ADDR_REG] = (pi->regs[PI_DRAM_ADDR_REG] + length + 7) & ~7;
    pi->regs[PI_CART_ADDR_REG] = (pi->regs[PI_CART_ADDR_REG] + length + 1) & ~1;
    return 0;
}
static void pi_rom_fetch(struct pi_controller *pi, uint32_t source, int32_t length, uint8_t *dest)
{
    int l = length;
    if (source >= pi->rom_size)
        l = 0;
    else if (source + length > pi->rom_size)
        l = pi->rom_size - source;
    memcpy(dest, pi->rom + source, l);
    memset(dest + l, 0xFF, length - l);
}
static int pi_dma_write(struct pi_controller *pi)
{
    uint32_t dest   = pi->regs[PI_DRAM_ADDR_REG] & 0x7FFFFE;
    uint32_t source = pi->regs[PI_CART_ADDR_REG] & 0xFFFFFFE;
    int32_t  length = (pi->regs[PI_WR_LEN_REG] & 0xFFFFFF) + 1;
    if (pi->bus->dd->ipl_rom && (source & 0x06000000) == 0x06000000) {
        source &= 0x003FFFFF;
        if (source + length > 0x003FFFFF)
            length = 0x003FFFFF - source;
        memcpy(pi->bus->ri->ram + dest, pi->bus->dd->ipl_rom + source, length);
    } else if ((source & 0x05000000) == 0x05000000)
        dd_dma_write(pi->bus->dd, source, dest, length);
    else if (source >= 0x08000000 && source < 0x10000000) {
        if (pi->sram->ptr != NULL) {
            uint32_t sram_bank     = (source >> 18) & 3;
            uint32_t sram_offset   = (sram_bank * 0x8000) + (source & 0x7FFF);
            uint32_t sram_bank_end = (sram_offset + length - 1) / 0x8000;
            if (sram_bank == sram_bank_end && sram_offset + length <= pi->sram->size)
                memcpy(pi->bus->ri->ram + dest, (const uint8_t *)(pi->sram->ptr) + sram_offset, length);
        } else if (pi->flashram.data != NULL) {
            uint32_t flashram_offset = source & 0x1FFFF;
            if (pi->flashram.mode == FLASHRAM_STATUS) {
                uint64_t status = htonll(pi->flashram.status);
                memcpy(pi->bus->ri->ram + dest, &status, 8);
            } else if (pi->flashram.mode == FLASHRAM_READ)
                memcpy(pi->bus->ri->ram + dest, pi->flashram.data + flashram_offset * 2, length);
        }
    } else if (source >= 0x18000000 && source < 0x18400000) {
    } else if (pi->rom) {
        pi->regs[PI_WR_LEN_REG] = 0x7F;
        if (length <= 8)
            pi->regs[PI_WR_LEN_REG] -= pi->regs[PI_DRAM_ADDR_REG] & 7;
        uint8_t mem[128];
        bool    first_block = true;
        while (length > 0) {
            uint32_t dest      = pi->regs[PI_DRAM_ADDR_REG] & 0x7FFFFE;
            int32_t  misalign  = dest & 0x7;
            int32_t  cur_len   = length;
            int32_t  block_len = 128 - misalign;
            if (cur_len > block_len)
                cur_len = block_len;
            length -= cur_len;
            if (length & 1)
                length += 1;
            uint32_t source        = pi->regs[PI_CART_ADDR_REG] & 0xFFFFFFE;
            int32_t  rom_fetch_len = (cur_len + 1) & ~1;
            pi_rom_fetch(pi, source, rom_fetch_len, mem);
            pi->regs[PI_CART_ADDR_REG] += rom_fetch_len;
            if (first_block) {
                if (cur_len == block_len - 1)
                    cur_len++;
                cur_len -= misalign;
                if (cur_len < 0)
                    cur_len = 0;
            }
            memcpy(pi->bus->ri->ram + dest, mem, cur_len);
            pi->regs[PI_DRAM_ADDR_REG] += cur_len;
            pi->regs[PI_DRAM_ADDR_REG] = (pi->regs[PI_DRAM_ADDR_REG] + 7) & ~7;
            first_block                = false;
        }
    }
    return 0;
}
int pi_init(struct pi_controller *pi, struct bus_controller *bus, const uint8_t *rom, size_t rom_size,
            const struct save_file *sram, const struct save_file *flashram, struct is_viewer *is_viewer)
{
    pi->bus                 = bus;
    pi->rom                 = rom;
    pi->rom_size            = rom_size;
    pi->sram                = sram;
    pi->flashram_file       = flashram;
    pi->flashram.data       = (uint8_t *)flashram->ptr;
    pi->is_viewer           = is_viewer;
    pi->regs[PI_RD_LEN_REG] = 0x7F;
    pi->regs[PI_WR_LEN_REG] = 0x7F;
    pi->bytes_to_copy       = 0;
    return 0;
}
int read_cart_rom(void *opaque, uint32_t address, uint32_t *word)
{
    struct pi_controller *pi     = (struct pi_controller *)opaque;
    unsigned              offset = (address - ROM_CART_BASE_ADDRESS) & ~0x3;
    if (pi->is_viewer && is_viewer_map(pi->is_viewer, address))
        return read_is_viewer(pi->is_viewer, address, word);
    if (pi->rom == NULL || offset > (pi->rom_size - sizeof(*word))) {
        *word = (address >> 16) | (address & 0xFFFF0000);
        return 0;
    }
    memcpy(word, pi->rom + offset, sizeof(*word));
    *word = byteswap_32(*word);
    return 0;
}
int read_pi_regs(void *opaque, uint32_t address, uint32_t *word)
{
    struct pi_controller *pi     = (struct pi_controller *)opaque;
    unsigned              offset = (unsigned)address - PI_REGS_BASE_ADDRESS;
    enum pi_register      reg    = (pi_register)(offset >> 2);
    *word                        = pi->regs[reg];
    debug_mmio_read(pi, pi_register_mnemonics[reg], *word);
    return 0;
}
int write_cart_rom(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    struct pi_controller *pi = (struct pi_controller *)opaque;
    if (pi->is_viewer && is_viewer_map(pi->is_viewer, address))
        return write_is_viewer(pi->is_viewer, address, word, dqm);
    return 0;
}
int write_pi_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    struct pi_controller *pi     = (struct pi_controller *)opaque;
    unsigned              offset = address - PI_REGS_BASE_ADDRESS;
    enum pi_register      reg    = (pi_register)(offset >> 2);
    debug_mmio_write(pi, pi_register_mnemonics[reg], word, dqm);
    if (reg == PI_STATUS_REG) {
        if (word & PI_STATUS_RESET_CONTROLLER)
            pi->regs[reg] &= ~(PI_STATUS_IS_BUSY | PI_STATUS_ERROR);
        if (word & PI_STATUS_CLEAR_INTERRUPT) {
            g_vr4300->clear_rcp_interrupt(MI_INTR_PI);
            pi->regs[reg] &= ~PI_STATUS_INTERRUPT;
        }
    } else {
        if (pi->regs[PI_STATUS_REG] & PI_STATUS_IS_BUSY) {
            pi->regs[PI_STATUS_REG] |= PI_STATUS_ERROR;
            return 0;
        }
        pi->regs[reg] &= ~dqm;
        pi->regs[reg] |= word;
        if (reg == PI_DRAM_ADDR_REG) {
            pi->regs[reg] &= 0x00FFFFFE;
        } else if (reg == PI_CART_ADDR_REG) {
            pi->regs[reg] &= 0xFFFFFFFE;
            dd_pi_write(pi->bus->dd, word);
        } else if (reg == PI_WR_LEN_REG) {
            if (pi->regs[PI_DRAM_ADDR_REG] == 0xFFFFFFFF) {
                pi->regs[PI_STATUS_REG] &= ~PI_STATUS_DMA_BUSY;
                pi->regs[PI_STATUS_REG] |= PI_STATUS_INTERRUPT;
                g_vr4300->signal_rcp_interrupt(MI_INTR_PI);
                return 0;
            }
            pi->bytes_to_copy = (pi->regs[PI_WR_LEN_REG] & 0xFFFFFF) + 1;
            pi->counter       = pi->bytes_to_copy / 2 + 100;
            pi->regs[PI_STATUS_REG] |= PI_STATUS_DMA_BUSY;
            pi->is_dma_read = false;
        } else if (reg == PI_RD_LEN_REG) {
            if (pi->regs[PI_DRAM_ADDR_REG] == 0xFFFFFFFF) {
                pi->regs[PI_STATUS_REG] &= ~PI_STATUS_DMA_BUSY;
                pi->regs[PI_STATUS_REG] |= PI_STATUS_INTERRUPT;
                g_vr4300->signal_rcp_interrupt(MI_INTR_PI);
                return 0;
            }
            pi->bytes_to_copy = (pi->regs[PI_RD_LEN_REG] & 0xFFFFFF) + 1;
            pi->counter       = pi->bytes_to_copy / 2 + 100;
            pi->regs[PI_STATUS_REG] |= PI_STATUS_DMA_BUSY;
            pi->is_dma_read = true;
        }
    }
    return 0;
}
int read_flashram(void *opaque, uint32_t address, uint32_t *word)
{
    struct pi_controller *pi = (struct pi_controller *)opaque;
    if (pi->flashram.data == NULL)
        return -1;
    *word = pi->flashram.status >> 32;
    return 0;
}
int write_flashram(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    struct pi_controller *pi = (struct pi_controller *)opaque;
    if (pi->flashram.data == NULL) {
        return 1;
    }
    if (address == 0x08000000) {
        return 0;
    }
    switch (word >> 24) {
        case 0x4B:
            pi->flashram.offset = (word & 0xFFFF) * 128;
            break;
        case 0x78:
            pi->flashram.mode   = FLASHRAM_ERASE;
            pi->flashram.status = 0x1111800800C20000LL;
            break;
        case 0xA5:
            pi->flashram.offset = (word & 0xFFFF) * 128;
            pi->flashram.status = 0x1111800400C20000LL;
            break;
        case 0xB4:
            pi->flashram.mode = FLASHRAM_WRITE;
            break;
        case 0xD2:
            if (pi->flashram.mode == FLASHRAM_ERASE)
                memset(pi->flashram.data + pi->flashram.offset, 0xFF, 0x80);
            else if (pi->flashram.mode == FLASHRAM_WRITE)
                memcpy(pi->flashram.data + pi->flashram.offset, pi->bus->ri->ram + pi->flashram.rdram_pointer, 0x80);
            break;
        case 0xE1:
            pi->flashram.mode   = FLASHRAM_STATUS;
            pi->flashram.status = 0x1111800100C20000LL;
            break;
        case 0xF0:
            pi->flashram.mode   = FLASHRAM_READ;
            pi->flashram.status = 0x11118004F0000000LL;
            break;
    }
    return 0;
}
int read_sram(void *opaque, uint32_t address, uint32_t *word)
{
    fprintf(stderr, "SRAM read\n");
    return 0;
}
int write_sram(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    fprintf(stderr, "SRAM write\n");
    return 0;
}
int is_viewer_init(struct is_viewer *is, int is_viewer_output)
{
    memset(is, 0, sizeof(*is));
    is->base_address       = IS_VIEWER_BASE_ADDRESS;
    is->len                = IS_VIEWER_ADDRESS_LEN;
    is->buffer             = (uint8_t *)calloc(IS_VIEWER_ADDRESS_LEN, 1);
    is->output_buffer      = (uint8_t *)calloc(IS_BUFFER_SIZE, 1);
    is->output_buffer_conv = (uint8_t *)calloc(IS_BUFFER_SIZE * 3, 1);
    is->show_output        = is_viewer_output;
    is->cd                 = iconv_open("UTF-8", "EUC-JP");
    if (is->buffer == NULL || is->output_buffer == NULL || is->output_buffer_conv == NULL)
        return 0;
    else
        return 1;
}
int is_viewer_map(struct is_viewer *is, uint32_t address)
{
    return address >= is->base_address && address + 4 <= is->base_address + is->len;
}
int read_is_viewer(struct is_viewer *is, uint32_t address, uint32_t *word)
{
    uint32_t offset = address - is->base_address;
    assert(offset + 4 <= is->len);
    memcpy(word, is->buffer + offset, sizeof(*word));
    *word = byteswap_32(*word);
    return 0;
}
int write_is_viewer(struct is_viewer *is, uint32_t address, uint32_t word, uint32_t dqm)
{
    uint32_t offset = address - is->base_address;
    assert(offset + 4 <= is->len);
    if (offset == 0x14) {
        if (word > 0) {
            assert(is->output_buffer_pos + word + 0x20 < is->len);
            memcpy(is->output_buffer + is->output_buffer_pos, is->buffer + 0x20, word);
            is->output_buffer_pos += word;
            is->output_buffer[is->output_buffer_pos] = '\0';
            if (memchr(is->output_buffer, '\n', is->output_buffer_pos)) {
                char  *inptr  = (char *)is->output_buffer;
                size_t len    = strlen(inptr);
                size_t outlen = 3 * len;
                char  *outptr = (char *)is->output_buffer_conv;
                memset(is->output_buffer_conv, 0, IS_BUFFER_SIZE * 3);
                iconv(is->cd, &inptr, &len, &outptr, &outlen);
                if (is->show_output)
                    printf("%s", is->output_buffer_conv);
                else if (!is->output_warning) {
                    printf("ISViewer debugging output detected and suppressed.\nRun n64 with option -is-viewer to "
                           "display it\n");
                    is->output_warning = 1;
                }
                memset(is->output_buffer, 0, is->output_buffer_pos);
                is->output_buffer_pos = 0;
            }
        }
        memset(is->buffer + 0x20, 0, word);
    } else {
        word = byteswap_32(word);
        memcpy(is->buffer + offset, &word, sizeof(word));
    }
    return 0;
}
