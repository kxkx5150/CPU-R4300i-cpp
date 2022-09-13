#ifndef __si_controller_h__
#define __si_controller_h__
#include "../../utils/common.h"
#include "../../utils/common.h"
#include "../../utils/common/rom_file.h"
#include "../../utils/common/save_file.h"
#include "local_time.h"
#include "dd.h"

#define MEMPAK_SIZE      0x8000
#define MEMPAK_NUM_PAGES 128
#define MEMPAK_PAGE_SIZE (MEMPAK_SIZE / MEMPAK_NUM_PAGES)

struct gb_cart
{
    int      mbc_type;
    int      cartrom_num_banks;
    int      reg_rom_bank_low;
    int      reg_rom_bank_high;
    uint8_t *bootrom;
    size_t   bootromsize;
    uint8_t *cartrom;
    uint8_t *cartromValid;
    uint8_t *cartrom_bank_zero;
    uint8_t *cartrom_bank_n;
    uint8_t *cartromValid_bank_n;
    uint16_t cart_bank_num;
    size_t   cartromsize;
    uint8_t *extram;
    uint8_t *extramValidRead;
    uint8_t *extramValidWrite;
    uint8_t *extram_bank;
    uint8_t *extram_bank_validRead;
    uint8_t *extram_bank_validWrite;
    uint8_t  extram_bank_num;
    int      extram_enabled;
    int      mbc1_mode;
    size_t   extram_size;
    int      extram_num_banks;
    int      battery_backed;
    void (*cleanup)(void);
    char  savename[256];
    int   huc3_ram_mode;
    FILE *fd;
    int   chardev_mode;
};
struct controller;
uint8_t gb_read(struct controller *controller, uint16_t address);
void    gb_write(struct controller *controller, uint16_t address, uint8_t data);
void    gb_init(struct controller *controller);
enum pak_type
{
    PAK_NONE = 0,
    PAK_MEM,
    PAK_RUMBLE,
    PAK_TRANSFER,
};
struct controller
{
    const char      *mempak_path;
    struct save_file mempak_save;
    enum pak_type    pak;
    int              pak_enabled;
    int              present;
    const char      *tpak_rom_path;
    struct rom_file  tpak_rom;
    const char      *tpak_save_path;
    struct save_file tpak_save;
    int              tpak_mode;
    int              tpak_mode_changed;
    int              tpak_bank;
    uint8_t (*gb_readmem[0x100])(struct controller *controller, uint16_t address);
    void (*gb_writemem[0x100])(struct controller *controller, uint16_t address, uint8_t data);
    struct gb_cart cart;
};
void controller_pak_format(uint8_t *ptr);
int  controller_pak_read(struct controller *controller, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf,
                         uint8_t recv_bytes);
int  controller_pak_write(struct controller *controller, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf,
                          uint8_t recv_bytes);


void transfer_pak_read(struct controller *controller, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf,
                       uint8_t recv_bytes);
void transfer_pak_write(struct controller *controller, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf,
                        uint8_t recv_bytes);


enum si_register
{
    SI_DRAM_ADDR_REG,
    SI_PIF_ADDR_RD64B_REG,
    SI_RESERVED_1_REG,
    SI_RESERVED_2_REG,
    SI_PIF_ADDR_WR64B_REG,
    SI_RESERVED_3_REG,
    SI_STATUS_REG,
    NUM_SI_REGISTERS
};
#ifdef DEBUG_MMIO_REGISTER_ACCESS
extern const char *si_register_mnemonics[NUM_SI_REGISTERS];
#endif
struct eeprom
{
    uint8_t *data;
    size_t   size;
};
struct rtc
{
    uint16_t          control;
    struct time_stamp now;
    int32_t           offset_seconds;
};
struct si_controller
{
    struct bus_controller *bus;
    const uint8_t         *rom;
    uint8_t                command[64];
    uint8_t                ram[64];
    uint32_t               regs[NUM_SI_REGISTERS];
    uint32_t               pif_status;
    uint8_t                input[4];
    struct eeprom          eeprom;
    struct rtc             rtc;
    struct controller      controller[4];
};
int si_init(struct si_controller *si, struct bus_controller *bus, const uint8_t *pif_rom, const uint8_t *cart_rom,
            const struct dd_variant *dd_variant, uint8_t *eeprom, size_t eeprom_size,
            const struct controller *controller);
int read_pif_rom_and_ram(void *opaque, uint32_t address, uint32_t *word);
int write_pif_rom_and_ram(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);
int read_si_regs(void *opaque, uint32_t address, uint32_t *word);
int write_si_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);

void rtc_init(struct rtc *rtc);
int  rtc_status(struct rtc *rtc, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf, uint8_t recv_bytes);
int  rtc_read(struct rtc *rtc, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf, uint8_t recv_bytes);
int  rtc_write(struct rtc *rtc, uint8_t *send_buf, uint8_t send_bytes, uint8_t *recv_buf, uint8_t recv_bytes);
#endif
