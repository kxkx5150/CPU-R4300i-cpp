#include "bus.h"
#include "ri.h"
#include "si.h"
#include "thread.h"
#include "vi.h"
#include "cpu.h"
#include <assert.h>
#include <cstdint>
#include <stddef.h>

extern vr4300 *g_vr4300;


int get_cic_seed(const uint8_t *rom_data, uint32_t *cic_seed);

#define CIC_SEED_NUS_5101 0x0000AC00U
#define CIC_SEED_NUS_6101 0x00043F3FU
#define CIC_SEED_NUS_6102 0x00003F3FU
#define CIC_SEED_NUS_6103 0x0000783FU
#define CIC_SEED_NUS_6105 0x0000913FU
#define CIC_SEED_NUS_6106 0x0000853FU
#define CIC_SEED_NUS_8303 0x0000DD00U
#define CRC_NUS_5101      0x587BD543U
#define CRC_NUS_6101      0x6170A4A1U
#define CRC_NUS_7102      0x009E9EA3U
#define CRC_NUS_6102      0x90BB6CB5U
#define CRC_NUS_6103      0x0B050EE0U
#define CRC_NUS_6105      0x98BC2C86U
#define CRC_NUS_6106      0xACC8580AU
#define CRC_NUS_8303      0x0E018159U
#define CRC_iQue_1        0xCD19FEF1U
#define CRC_iQue_2        0xB98CED9AU
#define CRC_iQue_3        0xE71C2766U

static const uint16_t CARTRIDGE_TYPE_ADDRESS   = 0x0147;
static const uint16_t ROM_SIZE_CODE_ADDRESS    = 0x0148;
static const uint16_t EXTRAM_SIZE_CODE_ADDRESS = 0x0149;

static int  read_pif_ram(void *opaque, uint32_t address, uint32_t *word);
static int  write_pif_ram(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);
static void pif_process(struct si_controller *si);
static int  pif_perform_command(struct si_controller *si, unsigned channel, uint8_t *send_buf, uint8_t send_bytes,
                                uint8_t *recv_buf, uint8_t recv_bytes);
static int  eeprom_read(struct eeprom *eeprom, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf,
                        uint8_t recv_bytes);
static int  eeprom_write(struct eeprom *eeprom, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf,
                         uint8_t recv_bytes);

static uint32_t si_crc32(const uint8_t *data, size_t size);
static uint8_t  controller_pak_crc(uint8_t *data);
static void     gameboy_read(struct controller *controller, uint16_t address, uint8_t *buffer);
static void     gameboy_write(struct controller *controller, uint16_t address, uint8_t *buffer);

void cart_reset_mbc(struct controller *controller);
void cart_default_cleanup();


int si_init(struct si_controller *si, struct bus_controller *bus, const uint8_t *pif_rom, const uint8_t *cart_rom,
            const struct dd_variant *dd_variant, uint8_t *eeprom, size_t eeprom_size,
            const struct controller *controller)
{
    uint32_t cic_seed;
    si->bus = bus;
    si->rom = pif_rom;
    if (cart_rom) {
        if (get_cic_seed(cart_rom, &cic_seed)) {
            printf("Unknown CIC type; is this a byte-swapped ROM?\n");
            return 1;
        }
        si->ram[0x24] = cic_seed >> 24;
        si->ram[0x25] = cic_seed >> 16;
        si->ram[0x26] = cic_seed >> 8;
        si->ram[0x27] = cic_seed >> 0;
    } else if (dd_variant != NULL) {
        si->ram[0x24] = 0x00;
        si->ram[0x25] = 0x0A;
        si->ram[0x26] = dd_variant->seed;
        si->ram[0x27] = 0x3F;
    }
    if (si->ram[0x26] == 0x3F && si->ram[0x27] == 0x3F)
        bus_write_word(si->bus, 0x318, 0x800000, ~0U);
    else if (si->ram[0x26] == 0x91 && si->ram[0x27] == 0x3F)
        bus_write_word(si->bus, 0x3F0, 0x800000, ~0U);
    si->eeprom.data = eeprom;
    si->eeprom.size = eeprom_size;
    rtc_init(&si->rtc);
    memcpy(si->controller, controller, sizeof(struct controller) * 4);
    return 0;
}
int pif_perform_command(struct si_controller *si, unsigned channel, uint8_t *send_buf, uint8_t send_bytes,
                        uint8_t *recv_buf, uint8_t recv_bytes)
{
    uint8_t                command = send_buf[0];
    struct bus_controller *bus;
    switch (command) {
        case 0x00:
        case 0xFF:
            switch (channel) {
                case 0:
                    recv_buf[0] = 0x05;
                    recv_buf[1] = 0x00;
                    recv_buf[2] = si->controller[channel].pak == PAK_NONE ? 0x00 : 0x01;
                    break;
                case 1:
                case 2:
                case 3:
                    if (si->controller[channel].present) {
                        recv_buf[0] = 0x05;
                        recv_buf[1] = 0x00;
                        recv_buf[2] = si->controller[channel].pak == PAK_NONE ? 0x00 : 0x01;
                    } else {
                        recv_buf[0] = recv_buf[1] = recv_buf[2] = 0xFF;
                        return 1;
                    }
                    break;
                case 4:
                    switch (si->eeprom.size) {
                        case 0x200:
                            recv_buf[0] = 0x00;
                            recv_buf[1] = 0x80;
                            recv_buf[2] = 0x00;
                            break;
                        case 0x800:
                            recv_buf[0] = 0x00;
                            recv_buf[1] = 0xC0;
                            recv_buf[2] = 0x00;
                            break;
                    }
                    break;
                default:
                    assert(0 && "Invalid channel.");
                    return 1;
            }
            break;
        case 0x01:
            switch (channel) {
                case 0:
                    memcpy(&bus, si, sizeof(bus));
                    if (likely((long int)bus->vi->window)) {
                        n64_mutex_lock(&bus->vi->window->event_mutex);
                        memcpy(recv_buf, si->input, sizeof(si->input));
                        n64_mutex_unlock(&bus->vi->window->event_mutex);
                    } else
                        memset(recv_buf, 0x0, sizeof(si->input));
                    break;
                default:
                    return 1;
            }
            break;
        case 0x02:
            if (channel > 3) {
                assert(0 && "Invalid channel for controller pak read");
                return 1;
            }
            return controller_pak_read(&si->controller[channel], send_buf, send_bytes, recv_buf, recv_bytes);
        case 0x03:
            if (channel > 3) {
                assert(0 && "Invalid channel for controller pak write");
                return 1;
            }
            return controller_pak_write(&si->controller[channel], send_buf, send_bytes, recv_buf, recv_bytes);
        case 0x04:
            if (channel != 4) {
                assert(0 && "Invalid channel for EEPROM read");
                return 1;
            }
            return eeprom_read(&si->eeprom, send_buf, send_bytes, recv_buf, recv_bytes);
        case 0x05:
            if (channel != 4) {
                assert(0 && "Invalid channel for EEPROM write");
                return 1;
            }
            return eeprom_write(&si->eeprom, send_buf, send_bytes, recv_buf, recv_bytes);
        case 0x06:
            if (channel != 4) {
                assert(0 && "Invalid channel for RTC status");
                return 1;
            }
            return rtc_status(&si->rtc, send_buf, send_bytes, recv_buf, recv_bytes);
        case 0x07:
            if (channel != 4) {
                assert(0 && "Invalid channel for RTC read");
                return 1;
            }
            return rtc_read(&si->rtc, send_buf, send_bytes, recv_buf, recv_bytes);
        case 0x08:
            if (channel != 4) {
                assert(0 && "Invalid channel for RTC write");
                return 1;
            }
            return rtc_write(&si->rtc, send_buf, send_bytes, recv_buf, recv_bytes);
        default:
            return 1;
    }
    return 0;
}
void pif_process(struct si_controller *si)
{
    unsigned channel = 0;
    unsigned ptr     = 0;
    if (si->command[0x3F] != 0x1)
        return;
    while (ptr < 0x3F) {
        int8_t send_bytes = si->command[ptr++];
        if (send_bytes == -2)
            break;
        if (send_bytes < 0)
            continue;
        if (send_bytes > 0 && (send_bytes & 0xC0) == 0) {
            int8_t  recv_bytes = si->command[ptr++];
            uint8_t recv_buf[0x40];
            uint8_t send_buf[0x40];
            if (recv_bytes == -2)
                break;
            if ((ptr + send_bytes) > sizeof(si->command) || (ptr + send_bytes + recv_bytes) > sizeof(si->command))
                break;
            memcpy(send_buf, si->command + ptr, send_bytes);
            ptr += send_bytes;
            memcpy(recv_buf, si->command + ptr, recv_bytes);
            int result = pif_perform_command(si, channel, send_buf, send_bytes, recv_buf, recv_bytes);
            if (result == 0) {
                memcpy(si->ram + ptr, recv_buf, recv_bytes);
                ptr += recv_bytes;
            } else
                si->ram[ptr - 2] |= 0x80;
        }
        channel++;
    }
    si->ram[0x3F] &= ~0x80;
}
int read_pif_ram(void *opaque, uint32_t address, uint32_t *word)
{
    struct si_controller *si     = (struct si_controller *)opaque;
    unsigned              offset = (address - PIF_RAM_BASE_ADDRESS) & 0x3F;
    if (offset == 0x24) {
        si->pif_status = 0x80;
        memcpy(word, si->ram + offset, sizeof(*word));
        *word = byteswap_32(*word);
    } else if (offset == 0x3C)
        *word = si->pif_status;
    else {
        memcpy(word, si->ram + offset, sizeof(*word));
        *word = byteswap_32(*word);
    }
    return 0;
}
int read_pif_rom_and_ram(void *opaque, uint32_t address, uint32_t *word)
{
    uint32_t              offset = address - PIF_ROM_BASE_ADDRESS;
    struct si_controller *si     = (struct si_controller *)opaque;
    if (address >= PIF_RAM_BASE_ADDRESS)
        return read_pif_ram(opaque, address, word);
    memcpy(word, si->rom + offset, sizeof(*word));
    *word = byteswap_32(*word);
    return 0;
}
int read_si_regs(void *opaque, uint32_t address, uint32_t *word)
{
    struct si_controller *si     = (struct si_controller *)opaque;
    unsigned              offset = address - SI_REGS_BASE_ADDRESS;
    enum si_register      reg    = (si_register)(offset >> 2);
    *word                        = si->regs[reg];
    debug_mmio_read(si, si_register_mnemonics[reg], *word);
    return 0;
}
int write_pif_ram(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    struct si_controller *si     = (struct si_controller *)opaque;
    unsigned              offset = (address - PIF_RAM_BASE_ADDRESS) & 0x3F;
    uint32_t              orig_word;
    memcpy(&orig_word, si->ram + offset, sizeof(orig_word));
    orig_word = byteswap_32(orig_word) & ~dqm;
    word      = byteswap_32(orig_word | word);
    memcpy(si->ram + offset, &word, sizeof(word));
    si->regs[SI_STATUS_REG] |= 0x1000;
    g_vr4300->signal_rcp_interrupt(MI_INTR_SI);
    return 0;
}
int write_pif_rom_and_ram(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    if (address >= PIF_RAM_BASE_ADDRESS)
        return write_pif_ram(opaque, address, word, dqm);
    return 0;
}
int write_si_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    struct si_controller *si     = (struct si_controller *)opaque;
    unsigned              offset = address - SI_REGS_BASE_ADDRESS;
    enum si_register      reg    = (si_register)(offset >> 2);
    debug_mmio_write(si, si_register_mnemonics[reg], word, dqm);
    if (reg == SI_STATUS_REG) {
        g_vr4300->clear_rcp_interrupt(MI_INTR_SI);
        si->regs[SI_STATUS_REG] &= ~0x1000;
    } else if (reg == SI_PIF_ADDR_RD64B_REG) {
        uint32_t offset = si->regs[SI_DRAM_ADDR_REG] & 0x1FFFFFFF;
        pif_process(si);
        memcpy(si->bus->ri->ram + offset, si->ram, sizeof(si->ram));
        g_vr4300->signal_rcp_interrupt(MI_INTR_SI);
        si->regs[SI_STATUS_REG] |= 0x1000;
    } else if (reg == SI_PIF_ADDR_WR64B_REG) {
        uint32_t offset = si->regs[SI_DRAM_ADDR_REG] & 0x1FFFFFFF;
        memcpy(si->ram, si->bus->ri->ram + offset, sizeof(si->ram));
        memcpy(si->command, si->ram, sizeof(si->command));
        g_vr4300->signal_rcp_interrupt(MI_INTR_SI);
        si->regs[SI_STATUS_REG] |= 0x1000;
    } else {
        si->regs[reg] &= ~dqm;
        si->regs[reg] |= word;
    }
    return 0;
}
int eeprom_read(struct eeprom *eeprom, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf, uint8_t recv_bytes)
{
    assert(send_bytes == 2 && recv_bytes == 8);
    uint16_t address = send_buf[1] << 3;
    if (eeprom->data != NULL && address <= eeprom->size - 8) {
        memcpy(recv_buf, &eeprom->data[address], 8);
        return 0;
    }
    return 1;
}
static int eeprom_write(struct eeprom *eeprom, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf,
                        uint8_t recv_bytes)
{
    assert(send_bytes == 10);
    uint16_t address = send_buf[1] << 3;
    if (eeprom->data != NULL && address <= eeprom->size - 8) {
        memcpy(&eeprom->data[address], send_buf + 2, 8);
        return 0;
    }
    return 1;
}
int get_cic_seed(const uint8_t *rom_data, uint32_t *cic_seed)
{
    uint32_t crc        = si_crc32(rom_data + 0x40, 0x1000 - 0x40);
    uint32_t aleck64crc = si_crc32(rom_data + 0x40, 0xC00 - 0x40);
    if (aleck64crc == CRC_NUS_5101)
        *cic_seed = CIC_SEED_NUS_5101;
    else
        switch (crc) {
            case CRC_NUS_6101:
            case CRC_NUS_7102:
            case CRC_iQue_1:
            case CRC_iQue_2:
            case CRC_iQue_3:
                *cic_seed = CIC_SEED_NUS_6101;
                break;
            case CRC_NUS_6102:
                *cic_seed = CIC_SEED_NUS_6102;
                break;
            case CRC_NUS_6103:
                *cic_seed = CIC_SEED_NUS_6103;
                break;
            case CRC_NUS_6105:
                *cic_seed = CIC_SEED_NUS_6105;
                break;
            case CRC_NUS_6106:
                *cic_seed = CIC_SEED_NUS_6106;
                break;
            case CRC_NUS_8303:
                *cic_seed = CIC_SEED_NUS_8303;
                break;
            default:
                *cic_seed = 0;
                return 1;
        }
    return 0;
}
uint32_t si_crc32(const uint8_t *data, size_t size)
{
    uint32_t table[256];
    unsigned n, k;
    uint32_t c;
    for (n = 0; n < 256; n++) {
        c = (uint32_t)n;
        for (k = 0; k < 8; k++) {
            if (c & 1)
                c = 0xEDB88320L ^ (c >> 1);
            else
                c = c >> 1;
        }
        table[n] = c;
    }
    c = 0L ^ 0xFFFFFFFF;
    for (n = 0; n < size; n++)
        c = table[(c ^ data[n]) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFF;
}
static inline uint8_t rtc_status_byte(struct rtc *rtc)
{
    return (rtc->control & 0x0004) ? 0x80 : 0x00;
}
void rtc_init(struct rtc *rtc)
{
    // Write-protected, not stopped
    rtc->control        = 0x0300;
    rtc->offset_seconds = 0;
    get_local_time(&rtc->now, rtc->offset_seconds);
}
int rtc_status(struct rtc *rtc, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf, uint8_t recv_bytes)
{
    // Check send/recv buffer lengths
    assert(send_bytes == 1);
    assert(recv_bytes == 3);
    recv_buf[0] = 0x00;
    recv_buf[1] = 0x10;
    recv_buf[2] = rtc_status_byte(rtc);
    return 0;
}
int rtc_read(struct rtc *rtc, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf, uint8_t recv_bytes)
{
    // Check send/recv buffer lengths
    assert(send_bytes == 2);
    assert(recv_bytes == 9);
    // Zero out the response buffer
    memset(recv_buf, 0, recv_bytes);
    // read RTC block
    switch (send_buf[1]) {
        case 0:
            recv_buf[0] = rtc->control >> 8;
            recv_buf[1] = rtc->control;
            break;
        case 1:
            break;
        case 2:
            // update the time if the clock is not stopped
            if ((rtc->control & 0x0004) == 0) {
                get_local_time(&rtc->now, rtc->offset_seconds);
            }
            recv_buf[0] = byte2bcd(rtc->now.sec);
            recv_buf[1] = byte2bcd(rtc->now.min);
            recv_buf[2] = byte2bcd(rtc->now.hour) + 0x80;
            recv_buf[3] = byte2bcd(rtc->now.day);
            recv_buf[4] = byte2bcd(rtc->now.week_day);
            recv_buf[5] = byte2bcd(rtc->now.month);
            recv_buf[6] = byte2bcd(rtc->now.year);
            recv_buf[7] = byte2bcd(rtc->now.year / 100);
            break;
        default:
            return 1;
    }
    recv_buf[8] = rtc_status_byte(rtc);
    return 0;
}
int rtc_write(struct rtc *rtc, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf, uint8_t recv_bytes)
{
    // Check send/recv buffer lengths
    assert(send_bytes == 10);
    assert(recv_bytes == 1);
    // write RTC block
    switch (send_buf[1]) {
        case 0:
            rtc->control = ((uint16_t)send_buf[2] << 8) | send_buf[3];
            break;
        case 1:
            break;
        case 2:
            if ((rtc->control & 0x0004) == 0) {
                break;
            }
            if (rtc->control & 0x0200) {
                break;
            }
            rtc->now.sec      = bcd2byte(send_buf[2]);
            rtc->now.min      = bcd2byte(send_buf[3]);
            rtc->now.hour     = bcd2byte(send_buf[4] - 0x80);
            rtc->now.day      = bcd2byte(send_buf[5]);
            rtc->now.week_day = bcd2byte(send_buf[6]);
            rtc->now.month    = bcd2byte(send_buf[7]);
            rtc->now.year     = bcd2byte(send_buf[8]);
            rtc->now.year += bcd2byte(send_buf[9]) * 100;
            // Set the clock offset based on current local time
            rtc->offset_seconds = get_offset_seconds(&rtc->now);
            break;
        default:
            return 1;
    }
    recv_buf[0] = rtc_status_byte(rtc);
    return 0;
}
int controller_pak_read(struct controller *controller, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf,
                        uint8_t recv_bytes)
{

    uint16_t address = send_buf[1] << 8 | send_buf[2];
    address &= ~0x1F;

    if (controller->pak == PAK_MEM) {
        if (address <= MEMPAK_SIZE - 0x20)
            memcpy(recv_buf, (uint8_t *)(controller->mempak_save.ptr) + address, 0x20);
        else
            assert(0 && "invalid mempak address");
    }

    else if (controller->pak == PAK_TRANSFER)
        transfer_pak_read(controller, send_buf, send_bytes, recv_buf, recv_bytes);

    else {
        uint8_t peripheral = 0x00;
        if (controller->pak == PAK_RUMBLE)
            peripheral = 0x80;
        memset(recv_buf, peripheral, 0x20);
    }

    recv_buf[0x20] = controller_pak_crc(recv_buf);

    return 0;
}
int controller_pak_write(struct controller *controller, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf,
                         uint8_t recv_bytes)
{

    uint16_t address = send_buf[1] << 8 | send_buf[2];
    address &= ~0x1F;

    if (address == 0x8000) {
        if (send_buf[3] == 0xfe)
            controller->pak_enabled = 0;
        else if (send_buf[3] == 0x84)
            controller->pak_enabled = 1;
    }

    else if (controller->pak == PAK_MEM) {
        if (address <= MEMPAK_SIZE - 0x20)
            memcpy((uint8_t *)(controller->mempak_save.ptr) + address, send_buf + 3, 0x20);
        else
            assert(0 && "Attempt to write past end of mempak");
    }

    else if (controller->pak == PAK_RUMBLE) {
        if (address == 0xC000) {
            if (send_buf[3] == 0x01) {
            } else {
            }
        } else {
        }
    }

    else if (controller->pak == PAK_TRANSFER)
        transfer_pak_write(controller, send_buf, send_bytes, recv_buf, recv_bytes);

    recv_buf[0] = controller_pak_crc(send_buf + 3);

    return 0;
}
uint8_t controller_pak_crc(uint8_t *data)
{
    size_t  i;
    int     mask;
    uint8_t crc = 0;

    for (i = 0; i <= 0x20; ++i) {
        for (mask = 0x80; mask >= 1; mask >>= 1) {
            uint8_t xor_tap = (crc & 0x80) ? 0x85 : 0x00;
            crc <<= 1;
            if (i < 0x20 && (data[i] & mask))
                crc |= 1;
            crc ^= xor_tap;
        }
    }

    return crc;
}
void controller_pak_format(uint8_t *ptr)
{
    static uint8_t init_id_area[] = {
        0x81, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
        0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF,
        0x05, 0x1A, 0x5F, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0xFF, 0x66, 0x25, 0x99, 0xCD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x05, 0x1A, 0x5F, 0x13, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0xFF, 0x66, 0x25,
        0x99, 0xCD, 0xFF, 0xFF, 0xFF, 0xFF, 0x05, 0x1A, 0x5F, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0xFF, 0x66, 0x25, 0x99, 0xCD, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x05, 0x1A,
        0x5F, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0x01, 0xFF, 0x66, 0x25, 0x99, 0xCD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
    };

    memset(ptr, 0, MEMPAK_SIZE);
    memcpy(ptr, init_id_area, sizeof(init_id_area));

    ptr[(MEMPAK_PAGE_SIZE * 1) + 1] = 0x71;
    ptr[(MEMPAK_PAGE_SIZE * 2) + 1] = 0x71;

    for (off_t i = 5; i < MEMPAK_NUM_PAGES; i++) {
        ptr[(MEMPAK_PAGE_SIZE * 1) + (i * 2) + 1] = 0x03;
        ptr[(MEMPAK_PAGE_SIZE * 2) + (i * 2) + 1] = 0x03;
    }
}
void transfer_pak_read(struct controller *controller, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf,
                       uint8_t recv_bytes)
{

    uint16_t address = send_buf[1] << 8 | send_buf[2];
    address &= ~0x1F;

    if (address == 0x8000)
        memset(recv_buf, controller->pak_enabled ? 0x84 : 0x00, 0x20);

    else if (address == 0xB000) {
        if (controller->pak_enabled) {
            if (controller->tpak_rom.ptr != NULL) {
                memset(recv_buf, controller->tpak_mode == 1 ? 0x89 : 0x80, 0x20);
                recv_buf[0] |= controller->tpak_mode_changed ? 0x4 : 0;
            }

            else
                memset(recv_buf, 0x40, 0x20);

            controller->tpak_mode_changed = 0;
        }
    }

    else if (address >= 0xC000) {
        if (controller->pak_enabled) {
            uint16_t gb_addr = address - 0xC000 + (controller->tpak_bank & 3) * 0x4000;
            gameboy_read(controller, gb_addr, recv_buf);
        }
    }
}
void transfer_pak_write(struct controller *controller, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf,
                        uint8_t recv_bytes)
{

    uint16_t address = send_buf[1] << 8 | send_buf[2];
    address &= ~0x1F;

    if (address == 0xA000) {
        if (controller->pak_enabled)
            controller->tpak_bank = send_buf[3];
    }

    else if (address == 0xB000) {
        if (controller->pak_enabled) {
            controller->tpak_mode         = send_buf[3] & 1;
            controller->tpak_mode_changed = 1;
        }
    }

    else if (address >= 0xC000) {
        uint16_t gb_addr = address - 0xC000 + (controller->tpak_bank & 3) * 0x4000;
        gameboy_write(controller, gb_addr, send_buf + 3);
    }
}
void gameboy_read(struct controller *controller, uint16_t address, uint8_t *buffer)
{
    for (int i = 0; i < 0x20; i++)
        buffer[i] = gb_read(controller, address + i);
}
void gameboy_write(struct controller *controller, uint16_t address, uint8_t *buffer)
{
    for (int i = 0; i < 0x20; i++)
        gb_write(controller, address + i, buffer[i]);
}
static uint8_t mbc_read_ff(struct controller *controller, uint16_t address)
{
    return 0xff;
}
static void mbc_write_dummy(struct controller *controller, uint16_t address, uint8_t data)
{
}
static void mbc_write_ram_enable(struct controller *controller, uint16_t address, uint8_t data)
{
    data &= 0xF;
    if (data == 0) {
        controller->cart.extram_enabled = false;
    } else if (data == 0x0A) {
        controller->cart.extram_enabled = true;
    }
}
static uint8_t mbc_read_bank_0(struct controller *controller, uint16_t address)
{
    return controller->cart.cartrom_bank_zero[address];
}
static uint8_t mbc_read_bank_n(struct controller *controller, uint16_t address)
{
    return controller->cart.cartrom_bank_n[address & 0x3fff];
}
static uint8_t mbc_read_extram(struct controller *controller, uint16_t address)
{
    if (controller->cart.extram_enabled) {
        return controller->cart.extram_bank[address & 0x1fff];
    } else {
        return 0xFF;
    }
}
static void mbc_write_extram(struct controller *controller, uint16_t address, uint8_t data)
{
    if (controller->cart.extram_enabled) {
        controller->cart.extram_bank[address & 0x1fff] = data;
    }
}
static void mbc_mbc1_write_rom_bank_select(struct controller *controller, uint16_t address, uint8_t data)
{
    struct gb_cart *cart = &controller->cart;
    data &= 0x1F;
    uint8_t bank = (cart->reg_rom_bank_high << 5) | data;
    if (bank == 0x00 || bank == 0x20 || bank == 0x40 || bank == 0x60) {
        bank++;
    }
    if (bank >= cart->cartrom_num_banks) {
        bank = bank % cart->cartrom_num_banks;
    }
    cart->reg_rom_bank_low = bank & 0x1F;
    cart->cart_bank_num    = bank;
    cart->cartrom_bank_n   = cart->cartrom + bank * 0x4000;
}
static void mbc_mbc1_write_rom_ram_bank_select(struct controller *controller, uint16_t address, uint8_t data)
{
    data &= 3;
    struct gb_cart *cart = &controller->cart;
    if (cart->mbc1_mode == 0 || cart->cartrom_num_banks > 0x1F) {
        if (cart->cartrom_num_banks > 0x1F) {
            uint8_t bank = (data << 5) | cart->reg_rom_bank_low;
            if (bank == 0x00 || bank == 0x20 || bank == 0x40 || bank == 0x60) {
                bank++;
            }
            if (bank >= cart->cartrom_num_banks) {
                bank = bank % cart->cartrom_num_banks;
            }
            cart->reg_rom_bank_high = bank >> 5;
            cart->cartrom_bank_n    = cart->cartrom + bank * 0x4000;
        }
        cart->cartrom_bank_zero = cart->cartrom;
        if (cart->mbc1_mode == 0) {
            return;
        }
    }
    if (cart->mbc1_mode == 1) {
        if (cart->cartrom_num_banks > 0x20) {
            uint8_t bank = data << 5;
            if (bank >= cart->cartrom_num_banks) {
                bank = bank % cart->cartrom_num_banks;
            }
            cart->cartrom_bank_zero = cart->cartrom + bank * 0x4000;
        }
        if (cart->extram_num_banks <= 1) {
            return;
        }
        cart->extram_bank_num = data;
        cart->extram_bank     = cart->extram + data * 0x2000;
    }
}
static void mbc_mbc1_write_bank_mode_select(struct controller *controller, uint16_t address, uint8_t data)
{
    struct gb_cart *cart = &controller->cart;
    cart->mbc1_mode      = data & 0x1;
    if (cart->mbc1_mode == 0) {
        cart->extram_bank_num = 0;
        cart->extram_bank     = cart->extram;
    } else {
        cart->reg_rom_bank_high = 0;
        cart->cartrom_bank_n    = cart->cartrom + cart->reg_rom_bank_low * 0x4000;
    }
}
static void mbc_mbc1_install(struct controller *controller)
{
    int i;
    controller->cart.mbc1_mode = 0;
    for (i = 0x00; i <= 0x3F; ++i) {
        controller->gb_readmem[i] = mbc_read_bank_0;
    }
    for (i = 0x40; i <= 0x7F; ++i) {
        controller->gb_readmem[i] = mbc_read_bank_n;
    }
    for (i = 0x00; i <= 0x1F; ++i) {
        controller->gb_writemem[i] = mbc_write_ram_enable;
    }
    for (i = 0x20; i <= 0x3F; ++i) {
        controller->gb_writemem[i] = mbc_mbc1_write_rom_bank_select;
    }
    for (i = 0x40; i <= 0x5F; ++i) {
        controller->gb_writemem[i] = mbc_mbc1_write_rom_ram_bank_select;
    }
    for (i = 0x60; i <= 0x7F; ++i) {
        controller->gb_writemem[i] = mbc_mbc1_write_bank_mode_select;
    }
    int extram_end = 0xA0 + (controller->cart.extram_size > 8192 ? 8192 : controller->cart.extram_size) / 256;
    for (i = 0xA0; i < extram_end; ++i) {
        controller->gb_readmem[i] = mbc_read_extram;
    }
    for (i = extram_end; i <= 0xBF; ++i) {
        controller->gb_readmem[i] = mbc_read_ff;
    }
    for (i = 0xA0; i < extram_end; ++i) {
        controller->gb_writemem[i] = mbc_write_extram;
    }
    for (i = extram_end; i <= 0xBF; ++i) {
        controller->gb_writemem[i] = mbc_write_dummy;
    }
}
static void mbc_mbc2_write_rom_bank_select(struct controller *controller, uint16_t address, uint8_t data)
{
    struct gb_cart *cart = &controller->cart;
    uint8_t         bank = data & 0xF;
    if (bank >= cart->cartrom_num_banks) {
        bank = bank % cart->cartrom_num_banks;
    }
    if (bank == 0) {
        bank++;
    }
    cart->reg_rom_bank_low = bank;
    cart->cart_bank_num    = bank;
    cart->cartrom_bank_n   = cart->cartrom + bank * 0x4000;
}
static uint8_t mbc_mbc2_read_extram(struct controller *controller, uint16_t address)
{
    if (controller->cart.extram_enabled) {
        address &= 0x1FF;
        return (controller->cart.extram_bank[address & 0x1fff] & 0xF);
    } else {
        return 0xFF;
    }
}
static void mbc_mbc2_write_extram(struct controller *controller, uint16_t address, uint8_t data)
{
    if (controller->cart.extram_enabled) {
        address &= 0x1FF;
        controller->cart.extram_bank[address & 0x1fff] = (data & 0xF);
    }
}
static void mbc_mbc2_install(struct controller *controller)
{
    int i;
    for (i = 0x00; i <= 0x3F; ++i) {
        controller->gb_readmem[i] = mbc_read_bank_0;
    }
    for (i = 0x40; i <= 0x7F; ++i) {
        controller->gb_readmem[i] = mbc_read_bank_n;
    }
    for (i = 0x00; i <= 0x3F; ++i) {
        if (i & 1) {
            controller->gb_writemem[i] = mbc_mbc2_write_rom_bank_select;
        } else {
            controller->gb_writemem[i] = mbc_write_ram_enable;
        }
    }
    for (i = 0xA0; i < 0xBF; ++i) {
        controller->gb_readmem[i] = mbc_mbc2_read_extram;
    }
    for (i = 0xA0; i <= 0xBF; ++i) {
        controller->gb_writemem[i] = mbc_mbc2_write_extram;
    }
}
void mbc_mbc3_write_rom_bank_select(struct controller *controller, uint16_t address, uint8_t data)
{
    struct gb_cart *cart = &controller->cart;
    size_t          offset;
    data &= 0x7F;
    cart->cart_bank_num = data;
    if (data == 0)
        offset = (size_t)16384;
    else
        offset = (size_t)data * 16384 % cart->cartromsize;
    cart->cartrom_bank_n = cart->cartrom + offset;
}
uint8_t mbc_mbc3_read_rtc(struct controller *controller, uint16_t address)
{
    return 0x00;
}
void mbc_mbc3_write_rtc(struct controller *controller, uint16_t address, uint8_t data)
{
}
void mbc_mbc3_write_ram_bank_select(struct controller *controller, uint16_t address, uint8_t data)
{
    struct gb_cart *cart = &controller->cart;
    int             i;
    switch (data) {
        case 0x00:
        case 0x01:
        case 0x02:
        case 0x03: {
            cart->extram_bank_num = data;
            cart->extram_bank     = cart->extram + data * 8192;
            int extram_end        = 0xA0 + (cart->extram_size > 8192 ? 8192 : cart->extram_size) / 256;
            for (i = 0xA0; i < extram_end; ++i) {
                controller->gb_readmem[i] = mbc_read_extram;
            }
            for (i = extram_end; i <= 0xBF; ++i) {
                controller->gb_readmem[i] = mbc_read_ff;
            }
            for (i = 0xA0; i < extram_end; ++i) {
                controller->gb_writemem[i] = mbc_write_extram;
            }
            for (i = extram_end; i <= 0xBF; ++i) {
                controller->gb_writemem[i] = mbc_write_dummy;
            }
        } break;
        case 0x08:
        case 0x09:
        case 0x0A:
        case 0x0B:
        case 0x0C:
            cart->extram_bank_num = data;
            for (i = 0xA0; i <= 0xBF; ++i) {
                controller->gb_readmem[i] = mbc_mbc3_read_rtc;
            }
            for (i = 0xA0; i <= 0xBF; ++i) {
                controller->gb_writemem[i] = mbc_mbc3_write_rtc;
            }
            break;
        default:
            printf("Switching to invalid extram bank %02X \n", data);
            break;
    }
}
void mbc_mbc3_write_clock_data_latch(struct controller *controller, uint16_t address, uint8_t data)
{
}
void mbc_mbc3_install(struct controller *controller)
{
    int i;
    for (i = 0x00; i <= 0x3F; ++i) {
        controller->gb_readmem[i] = mbc_read_bank_0;
    }
    for (i = 0x40; i <= 0x7F; ++i) {
        controller->gb_readmem[i] = mbc_read_bank_n;
    }
    for (i = 0x00; i <= 0x1F; ++i) {
        controller->gb_writemem[i] = mbc_write_ram_enable;
    }
    for (i = 0x20; i <= 0x3F; ++i) {
        controller->gb_writemem[i] = mbc_mbc3_write_rom_bank_select;
    }
    for (i = 0x40; i <= 0x5F; ++i) {
        controller->gb_writemem[i] = mbc_mbc3_write_ram_bank_select;
    }
    for (i = 0x60; i <= 0x7F; ++i) {
        controller->gb_writemem[i] = mbc_mbc3_write_clock_data_latch;
    }
    int extram_end = 0xA0 + (controller->cart.extram_size > 8192 ? 8192 : controller->cart.extram_size) / 256;
    for (i = 0xA0; i < extram_end; ++i) {
        controller->gb_readmem[i] = mbc_read_extram;
    }
    for (i = extram_end; i <= 0xBF; ++i) {
        controller->gb_readmem[i] = mbc_read_ff;
    }
    for (i = 0xA0; i < extram_end; ++i) {
        controller->gb_writemem[i] = mbc_write_extram;
    }
    for (i = extram_end; i <= 0xBF; ++i) {
        controller->gb_writemem[i] = mbc_write_dummy;
    }
}
static void mbc_rom_only_install(struct controller *controller)
{
    int i;
    for (i = 0x00; i <= 0x3F; ++i) {
        controller->gb_readmem[i] = mbc_read_bank_0;
    }
    for (i = 0x40; i <= 0x7F; ++i) {
        controller->gb_readmem[i] = mbc_read_bank_n;
    }
    for (i = 0x00; i <= 0x7F; ++i) {
        controller->gb_writemem[i] = mbc_write_dummy;
    }
    for (i = 0xA0; i < 0xBF; ++i) {
        controller->gb_readmem[i] = mbc_read_ff;
    }
    for (i = 0xA0; i < 0xBF; ++i) {
        controller->gb_writemem[i] = mbc_write_dummy;
    }
}
static void mbc_mbc5_write_ram_enable(struct controller *controller, uint16_t address, uint8_t data)
{
    if (data == 0x0A) {
        controller->cart.extram_enabled = true;
    } else {
        controller->cart.extram_enabled = false;
    }
}
static void mbc_mbc5_write_rom_bank_select_lower(struct controller *controller, uint16_t address, uint8_t data)
{
    struct gb_cart *cart = &controller->cart;
    uint16_t        bank = (cart->reg_rom_bank_high << 8) | data;
    if (bank >= cart->cartrom_num_banks) {
        bank = bank % cart->cartrom_num_banks;
    }
    cart->reg_rom_bank_low = data;
    cart->cart_bank_num    = bank;
    cart->cartrom_bank_n   = cart->cartrom + bank * 0x4000;
}
static void mbc_mbc5_write_rom_bank_select_upper(struct controller *controller, uint16_t address, uint8_t data)
{
    struct gb_cart *cart = &controller->cart;
    data &= 1;
    uint16_t bank = data << 8 | cart->reg_rom_bank_low;
    if (bank >= cart->cartrom_num_banks) {
        bank = bank % cart->cartrom_num_banks;
    }
    cart->reg_rom_bank_high = data;
    cart->cart_bank_num     = bank;
    cart->cartrom_bank_n    = cart->cartrom + bank * 0x4000;
}
static void mbc_mbc5_write_ram_bank_select(struct controller *controller, uint16_t address, uint8_t data)
{
    data &= 0xF;
    controller->cart.extram_bank_num = data;
    controller->cart.extram_bank     = controller->cart.extram + data * 0x2000;
}
static void mbc_mbc5_install(struct controller *controller)
{
    int i;
    for (i = 0x00; i <= 0x3F; ++i) {
        controller->gb_readmem[i] = mbc_read_bank_0;
    }
    for (i = 0x40; i <= 0x7F; ++i) {
        controller->gb_readmem[i] = mbc_read_bank_n;
    }
    for (i = 0x00; i <= 0x1F; ++i) {
        controller->gb_writemem[i] = mbc_mbc5_write_ram_enable;
    }
    for (i = 0x20; i <= 0x2F; ++i) {
        controller->gb_writemem[i] = mbc_mbc5_write_rom_bank_select_lower;
    }
    for (i = 0x30; i <= 0x3F; ++i) {
        controller->gb_writemem[i] = mbc_mbc5_write_rom_bank_select_upper;
    }
    for (i = 0x40; i <= 0x5f; ++i) {
        controller->gb_writemem[i] = mbc_mbc5_write_ram_bank_select;
    }
    int extram_end = 0xA0 + (controller->cart.extram_size > 8192 ? 8192 : controller->cart.extram_size) / 256;
    for (i = 0xA0; i < extram_end; ++i) {
        controller->gb_readmem[i] = mbc_read_extram;
    }
    for (i = extram_end; i <= 0xBF; ++i) {
        controller->gb_readmem[i] = mbc_read_ff;
    }
    for (i = 0xA0; i < extram_end; ++i) {
        controller->gb_writemem[i] = mbc_write_extram;
    }
    for (i = extram_end; i <= 0xBF; ++i) {
        controller->gb_writemem[i] = mbc_write_dummy;
    }
}
static void mbc_unsupported_install(struct controller *controller)
{
    printf("Transfer Pak ROM type not fully supported. Defaulting to MBC3 behaviour\n");
    mbc_mbc3_install(controller);
}
uint8_t gb_read(struct controller *controller, uint16_t address)
{
    uint8_t (*myfunc)(struct controller *, uint16_t) = controller->gb_readmem[address >> 8];
    uint8_t data;
    if (myfunc != NULL)
        data = myfunc(controller, address);
    else
        data = 0xFF;
    return data;
}
void gb_write(struct controller *controller, uint16_t address, uint8_t data)
{
    void (*myfunc)(struct controller *, uint16_t, uint8_t) = controller->gb_writemem[address >> 8];
    if (myfunc != NULL)
        myfunc(controller, address, data);
}
void gb_init(struct controller *controller)
{
    for (int i = 0; i < 0x100; i++) {
        controller->gb_readmem[i]  = NULL;
        controller->gb_writemem[i] = NULL;
    }
    struct gb_cart *cart    = &controller->cart;
    cart->cartromsize       = controller->tpak_rom.size;
    cart->cartrom           = (uint8_t *)(controller->tpak_rom.ptr);
    cart->cartrom_bank_zero = cart->cartrom;
    cart->cartrom_bank_n    = cart->cartrom + 0x4000;
    if (controller->tpak_save.ptr != NULL) {
        cart->extram      = (uint8_t *)(controller->tpak_save.ptr);
        cart->extram_size = controller->tpak_save.size;
    } else {
        cart->extram      = (uint8_t *)calloc(1, 65536);
        cart->extram_size = 65536;
    }
    cart->extram_bank     = cart->extram;
    uint8_t rom_size_code = cart->cartrom_bank_zero[ROM_SIZE_CODE_ADDRESS];
    if (rom_size_code <= 8) {
        cart->cartrom_num_banks = 1 << (rom_size_code + 1);
    } else {
        switch (rom_size_code) {
            case 0x52:
                cart->cartrom_num_banks = 72;
                break;
            case 0x53:
                cart->cartrom_num_banks = 80;
                break;
            case 0x54:
                cart->cartrom_num_banks = 96;
                break;
        }
    }
    switch (cart->cartrom_bank_zero[EXTRAM_SIZE_CODE_ADDRESS]) {
        case 0:
            cart->extram_num_banks = 0;
            break;
        case 1:
            cart->extram_num_banks = 1;
            break;
        case 2:
            cart->extram_num_banks = 1;
            break;
        case 3:
            cart->extram_num_banks = 4;
            break;
        case 4:
            cart->extram_num_banks = 16;
            break;
        case 5:
            cart->extram_num_banks = 8;
            break;
    }
    switch (cart->cartrom_bank_zero[CARTRIDGE_TYPE_ADDRESS]) {
        case 0:
            mbc_rom_only_install(controller);
            break;
        case 1:
        case 2:
        case 3:
            mbc_mbc1_install(controller);
            break;
        case 5:
        case 6:
            mbc_mbc2_install(controller);
            break;
        case 15:
        case 16:
        case 17:
        case 18:
        case 19:
            mbc_mbc3_install(controller);
            break;
        case 25:
        case 26:
        case 27:
        case 28:
        case 29:
        case 30:
            mbc_mbc5_install(controller);
            break;
        default:
            mbc_unsupported_install(controller);
            break;
    }
}
