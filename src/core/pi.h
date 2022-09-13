#ifndef __pi_controller_h__
#define __pi_controller_h__
#include <iconv.h>
#include "../../utils/common.h"
#include "../../utils/common/save_file.h"

#define IS_VIEWER_BASE_ADDRESS 0x13FF0000
#define IS_VIEWER_ADDRESS_LEN  0x00001000
#define FLASHRAM_SIZE          0x20000

struct is_viewer
{
    uint32_t base_address;
    uint32_t len;

    uint8_t *buffer;
    uint8_t *output_buffer;
    size_t   output_buffer_pos;
    uint8_t *output_buffer_conv;
    int      show_output;
    int      output_warning;

    iconv_t cd;
};
enum pi_register
{
    PI_DRAM_ADDR_REG,
    PI_CART_ADDR_REG,
    PI_RD_LEN_REG,
    PI_WR_LEN_REG,
    PI_STATUS_REG,
    PI_BSD_DOM1_LAT_REG,
    PI_BSD_DOM1_PWD_REG,
    PI_BSD_DOM1_PGS_REG,
    PI_BSD_DOM1_RLS_REG,
    PI_BSD_DOM2_LAT_REG,
    PI_BSD_DOM2_PWD_REG,
    PI_BSD_DOM2_PGS_REG,
    PI_BSD_DOM2_RLS_REG,
    NUM_PI_REGISTERS
};
enum pi_status
{
    PI_STATUS_DMA_BUSY  = 1 << 0,
    PI_STATUS_IO_BUSY   = 1 << 1,
    PI_STATUS_ERROR     = 1 << 2,
    PI_STATUS_INTERRUPT = 1 << 3,
    PI_STATUS_IS_BUSY   = PI_STATUS_DMA_BUSY | PI_STATUS_IO_BUSY
};
enum pi_status_write
{
    PI_STATUS_RESET_CONTROLLER = 1 << 0,
    PI_STATUS_CLEAR_INTERRUPT  = 1 << 1
};
enum flashram_mode
{
    FLASHRAM_IDLE = 0,
    FLASHRAM_ERASE,
    FLASHRAM_WRITE,
    FLASHRAM_READ,
    FLASHRAM_STATUS,
};
struct flashram
{
    uint8_t           *data;
    uint64_t           status;
    enum flashram_mode mode;
    size_t             offset;
    size_t             rdram_pointer;
};
struct pi_controller
{
    struct bus_controller  *bus;
    const uint8_t          *rom;
    size_t                  rom_size;
    const struct save_file *sram;
    const struct save_file *flashram_file;
    struct flashram         flashram;
    struct is_viewer       *is_viewer;
    uint64_t                counter;
    uint32_t                bytes_to_copy;
    bool                    is_dma_read;
    uint32_t                regs[NUM_PI_REGISTERS];
};

void pi_cycle_(struct pi_controller *pi);

static inline void pi_cycle(struct pi_controller *pi)
{
    if (unlikely(pi->counter-- == 0))
        pi_cycle_(pi);
}

int is_viewer_init(struct is_viewer *is, int show_output);
int is_viewer_map(struct is_viewer *is, uint32_t address);
int read_is_viewer(struct is_viewer *is, uint32_t address, uint32_t *word);
int write_is_viewer(struct is_viewer *is, uint32_t address, uint32_t word, uint32_t dqm);

int pi_init(struct pi_controller *pi, struct bus_controller *bus, const uint8_t *rom, size_t rom_size,
            const struct save_file *sram, const struct save_file *flashram, struct is_viewer *is);

int read_cart_rom(void *opaque, uint32_t address, uint32_t *word);
int read_pi_regs(void *opaque, uint32_t address, uint32_t *word);
int write_cart_rom(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);
int write_pi_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);
int read_flashram(void *opaque, uint32_t address, uint32_t *word);
int write_flashram(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);
int read_sram(void *opaque, uint32_t address, uint32_t *word);
int write_sram(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);

#endif
