#ifndef __bus_controller_h__
#define __bus_controller_h__
#include "../../utils/common.h"
#include <setjmp.h>


#define AI_REGS_BASE_ADDRESS 0x04500000
#define AI_REGS_ADDRESS_LEN  0x00000018

#define DD_REGS_BASE_ADDRESS 0x05000500
#define DD_MS_RAM_ADDRESS    0x05000580

#define DD_C2S_BUFFER_LEN   0x00000400
#define DD_DS_BUFFER_LEN    0x00000100
#define DD_REGS_ADDRESS_LEN 0x00000080
#define DD_MS_RAM_LEN       0x00000040

#define DD_CONTROLLER_ADDRESS 0x05000000
#define DD_CONTROLLER_LEN     (DD_C2S_BUFFER_LEN + DD_DS_BUFFER_LEN + DD_REGS_ADDRESS_LEN + DD_MS_RAM_LEN)

#define DD_IPL_ROM_ADDRESS 0x06000000
#define DD_IPL_ROM_LEN     0x00400000

#define DP_REGS_BASE_ADDRESS 0x04100000
#define DP_REGS_ADDRESS_LEN  0x00000020

#define FLASHRAM_BASE_ADDRESS 0x08000000
#define FLASHRAM_ADDRESS_LEN  0x00010004

#define MI_REGS_BASE_ADDRESS 0x04300000
#define MI_REGS_ADDRESS_LEN  0x00000010

#define PI_REGS_BASE_ADDRESS 0x04600000
#define PI_REGS_ADDRESS_LEN  0x00100000

#define PIF_ROM_BASE_ADDRESS 0x1FC00000
#define PIF_ROM_ADDRESS_LEN  0x000007C0

#define PIF_RAM_BASE_ADDRESS 0x1FC007C0
#define PIF_RAM_ADDRESS_LEN  0x00000040

#define PIF_BASE_ADDRESS PIF_ROM_BASE_ADDRESS
#define PIF_ADDRESS_LEN  (PIF_ROM_ADDRESS_LEN + PIF_RAM_ADDRESS_LEN)

#define RDRAM_BASE_ADDRESS     0x00000000
#define RDRAM_BASE_ADDRESS_LEN 0x00800000

#define RDRAM_REGS_BASE_ADDRESS 0x03F00000
#define RDRAM_REGS_ADDRESS_LEN  0x00000028

#define RI_REGS_BASE_ADDRESS 0x04700000
#define RI_REGS_ADDRESS_LEN  0x00000020

#define ROM_CART_BASE_ADDRESS 0x10000000
#define ROM_CART_ADDRESS_LEN  0x0FC00000

#define SI_REGS_BASE_ADDRESS 0x04800000
#define SI_REGS_ADDRESS_LEN  0x0000001C

#define SP_MEM_BASE_ADDRESS 0x04000000
#define SP_MEM_ADDRESS_LEN  0x00002000

#define SP_REGS_BASE_ADDRESS 0x04040000
#define SP_REGS_ADDRESS_LEN  0x00000020

#define SP_REGS2_BASE_ADDRESS 0x04080000
#define SP_REGS2_ADDRESS_LEN  0x00000008

#define SRAM_BASE_ADDRESS 0x08000000
#define SRAM_ADDRESS_LEN  0x0801FFFF

#define VI_REGS_BASE_ADDRESS 0x04400000
#define VI_REGS_ADDRESS_LEN  0x00000038

typedef int (*memory_rd_function)(void *, uint32_t, uint32_t *);
typedef int (*memory_wr_function)(void *, uint32_t, uint32_t, uint32_t);

enum memory_map_color
{
    MEMORY_MAP_BLACK,
    MEMORY_MAP_RED
};
struct memory_mapping
{
    void              *instance;
    memory_rd_function on_read;
    memory_wr_function on_write;
    uint32_t           length;
    uint32_t           start;
    uint32_t           end;
};
struct memory_map_node
{
    struct memory_map_node *left;
    struct memory_map_node *parent;
    struct memory_map_node *right;
    struct memory_mapping   mapping;
    enum memory_map_color   color;
};
struct memory_map
{
    struct memory_map_node  mappings[19];
    struct memory_map_node *nil;
    struct memory_map_node *root;
    unsigned                next_map_index;
};
struct ai_controller;
struct dd_controller;
struct pi_controller;
struct ri_controller;
struct si_controller;
struct vi_controller;
struct rdp;
struct rsp;
struct vr4300;
struct bus_controller
{
    struct ai_controller *ai;
    struct dd_controller *dd;
    struct pi_controller *pi;
    struct ri_controller *ri;
    struct si_controller *si;
    struct vi_controller *vi;
    struct rdp           *rdp;
    struct rsp           *rsp;
    struct vr4300        *vr4300;
    struct memory_map     map;
    jmp_buf               unwind_data;
};

void create_memory_map(struct memory_map *map);
int  map_address_range(struct memory_map *memory_map, uint32_t start, uint32_t length, void *instance,
                       memory_rd_function on_read, memory_wr_function on_write);
const struct memory_mapping *resolve_mapped_address(const struct memory_map *memory_map, uint32_t address);

int bus_init(struct bus_controller *bus, int dd_present);
int bus_read_word(const struct bus_controller *bus, uint32_t address, uint32_t *word);
int bus_write_word(struct bus_controller *bus, uint32_t address, uint32_t word, uint32_t dqm);

#endif
