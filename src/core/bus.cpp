#include "../../utils/common.h"
#include "ai.h"
#include "bus.h"
#include "dd.h"
#include "pi.h"
#include "ri.h"
#include "si.h"
#include "rdp.h"
#include "rsp.h"
#include "vi.h"
#include "cpu.h"

extern vr4300 *g_vr4300;

#define NUM_MAPPINGS 17

struct bus_controller_mapping
{
    memory_rd_function read;
    memory_wr_function write;
    uint32_t           address;
    uint32_t           length;
};

static int  bus_open_read(void *opaque, uint32_t address, uint32_t *word);
static int  bus_dead_write(void *opaque, uint32_t address, uint32_t word, uint32_t dqm);
static void fixup(struct memory_map *, struct memory_map_node *);
static void rotate_left(struct memory_map *, struct memory_map_node *);
static void rotate_right(struct memory_map *, struct memory_map_node *);

int _read_mi_regs(void *opaque, uint32_t address, uint32_t *word)
{
    g_vr4300->read_mi_regs(address, word);
    return 0;
}
int _write_mi_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    g_vr4300->write_mi_regs(address, word, dqm);
    return 0;
}
int bus_init(struct bus_controller *bus, int dd_present)
{
    unsigned                                   i;
    static const struct bus_controller_mapping mappings[NUM_MAPPINGS] = {
        {read_ai_regs, write_ai_regs, AI_REGS_BASE_ADDRESS, AI_REGS_ADDRESS_LEN},
        {read_dp_regs, write_dp_regs, DP_REGS_BASE_ADDRESS, DP_REGS_ADDRESS_LEN},
        {_read_mi_regs, _write_mi_regs, MI_REGS_BASE_ADDRESS, MI_REGS_ADDRESS_LEN},
        {read_pi_regs, write_pi_regs, PI_REGS_BASE_ADDRESS, PI_REGS_ADDRESS_LEN},
        {read_ri_regs, write_ri_regs, RI_REGS_BASE_ADDRESS, RI_REGS_ADDRESS_LEN},
        {read_si_regs, write_si_regs, SI_REGS_BASE_ADDRESS, SI_REGS_ADDRESS_LEN},
        {read_sp_regs, write_sp_regs, SP_REGS_BASE_ADDRESS, SP_REGS_ADDRESS_LEN},
        {read_vi_regs, write_vi_regs, VI_REGS_BASE_ADDRESS, VI_REGS_ADDRESS_LEN},
        {read_cart_rom, write_cart_rom, ROM_CART_BASE_ADDRESS, ROM_CART_ADDRESS_LEN},
        {read_flashram, write_flashram, FLASHRAM_BASE_ADDRESS, FLASHRAM_ADDRESS_LEN},
        {read_dd_controller, write_dd_controller, DD_CONTROLLER_ADDRESS, DD_CONTROLLER_LEN},
        {read_dd_ipl_rom, write_dd_ipl_rom, DD_IPL_ROM_ADDRESS, DD_IPL_ROM_LEN},
        {read_pif_rom_and_ram, write_pif_rom_and_ram, PIF_BASE_ADDRESS, PIF_ADDRESS_LEN},
        {read_rdram_regs, write_rdram_regs, RDRAM_REGS_BASE_ADDRESS, RDRAM_REGS_ADDRESS_LEN},
        {read_sp_regs2, write_sp_regs2, SP_REGS2_BASE_ADDRESS, SP_REGS2_ADDRESS_LEN},
        {read_sp_mem, write_sp_mem, SP_MEM_BASE_ADDRESS, SP_MEM_ADDRESS_LEN},
        {read_sram, write_sram, SRAM_BASE_ADDRESS, SRAM_ADDRESS_LEN},
    };
    void *instances[NUM_MAPPINGS] = {
        bus->ai, bus->rdp, bus->vr4300, bus->pi, bus->ri, bus->si,  bus->rsp, bus->vi, bus->pi,
        bus->pi, bus->dd,  bus->dd,     bus->si, bus->ri, bus->rsp, bus->rsp, bus->pi,
    };
    create_memory_map(&bus->map);
    for (i = 0; i < NUM_MAPPINGS; i++) {
        memory_rd_function rd       = mappings[i].read;
        memory_wr_function wr       = mappings[i].write;
        void              *instance = instances[i];
        if (instance == bus->dd && !dd_present) {
            rd       = bus_open_read;
            wr       = bus_dead_write;
            instance = NULL;
        }
        if (map_address_range(&bus->map, mappings[i].address, mappings[i].length, instances[i], rd, wr))
            return 1;
    }
    return 0;
}
static int bus_open_read(void *opaque, uint32_t address, uint32_t *word)
{
    *word = (address >> 16) | (address & 0xFFFF0000);
    return 0;
}
static int bus_dead_write(void *opaque, uint32_t address, uint32_t word, uint32_t dqm)
{
    return 0;
}
int bus_read_word(const struct bus_controller *bus, uint32_t address, uint32_t *word)
{
    const struct memory_mapping *node;
    if (address < RDRAM_BASE_ADDRESS_LEN)
        return read_rdram(bus->ri, address, word);
    else if ((node = resolve_mapped_address(&bus->map, address)) == NULL) {
        *word = (address >> 16) | (address & 0xFFFF0000);
        return 0;
    }
    return node->on_read(node->instance, address, word);
}
int bus_write_word(struct bus_controller *bus, uint32_t address, uint32_t word, uint32_t dqm)
{
    const struct memory_mapping *node;
    if (address < RDRAM_BASE_ADDRESS_LEN)
        return write_rdram(bus->ri, address, word & dqm, dqm);
    else if ((node = resolve_mapped_address(&bus->map, address)) == NULL) {
        return 0;
    }
    return node->on_write(node->instance, address, word & dqm, dqm);
}
void create_memory_map(struct memory_map *map)
{
    memset(map->mappings, 0, sizeof(struct memory_map_node) * 19);
    map->next_map_index = 1;
    map->nil            = map->mappings;
    map->root           = map->nil;
}
void fixup(struct memory_map *map, struct memory_map_node *node)
{
    struct memory_map_node *cur;
    while (node->parent->color == MEMORY_MAP_RED) {
        if (node->parent == node->parent->parent->left) {
            cur = node->parent->parent->right;
            if (cur->color == MEMORY_MAP_RED) {
                node->parent->color         = MEMORY_MAP_BLACK;
                cur->color                  = MEMORY_MAP_BLACK;
                node->parent->parent->color = MEMORY_MAP_RED;
                node                        = node->parent->parent;
            } else {
                if (node == node->parent->right) {
                    node = node->parent;
                    rotate_left(map, node);
                }
                node->parent->color         = MEMORY_MAP_BLACK;
                node->parent->parent->color = MEMORY_MAP_RED;
                rotate_right(map, node->parent->parent);
            }
        } else {
            cur = node->parent->parent->left;
            if (cur->color == MEMORY_MAP_RED) {
                node->parent->color         = MEMORY_MAP_BLACK;
                cur->color                  = MEMORY_MAP_BLACK;
                node->parent->parent->color = MEMORY_MAP_RED;
                node                        = node->parent->parent;
            } else {
                if (node == node->parent->left) {
                    node = node->parent;
                    rotate_right(map, node);
                }
                node->parent->color         = MEMORY_MAP_BLACK;
                node->parent->parent->color = MEMORY_MAP_RED;
                rotate_left(map, node->parent->parent);
            }
        }
    }
    map->root->color = MEMORY_MAP_BLACK;
}
int map_address_range(struct memory_map *map, uint32_t start, uint32_t length, void *instance,
                      memory_rd_function on_read, memory_wr_function on_write)
{
    struct memory_map_node *check = map->root;
    struct memory_map_node *cur   = map->nil;
    uint32_t                end   = start + length - 1;
    struct memory_map_node *new_node;
    struct memory_mapping   mapping;
    const unsigned          num_mappings = sizeof(map->mappings) / sizeof(map->mappings[0]) - 1;
    if (unlikely(map->next_map_index >= num_mappings)) {
        return 1;
    }
    new_node = &map->mappings[map->next_map_index++];
    while (check != map->nil) {
        cur   = check;
        check = (start < cur->mapping.start) ? check->left : check->right;
    }
    if (cur == map->nil)
        map->root = new_node;
    else if (start < cur->mapping.start)
        cur->left = new_node;
    else
        cur->right = new_node;
    new_node->left    = map->nil;
    new_node->right   = map->nil;
    new_node->parent  = cur;
    mapping.instance  = instance;
    mapping.on_read   = on_read;
    mapping.on_write  = on_write;
    mapping.end       = end;
    mapping.length    = length;
    mapping.start     = start;
    new_node->mapping = mapping;
    new_node->color   = MEMORY_MAP_RED;
    fixup(map, new_node);
    return 0;
}
const struct memory_mapping *resolve_mapped_address(const struct memory_map *map, uint32_t address)
{
    const struct memory_map_node *cur = map->root;
    do {
        if (address < cur->mapping.start)
            cur = cur->left;
        else if (address > cur->mapping.end)
            cur = cur->right;
        else
            return &cur->mapping;
    } while (cur != map->nil);
    return NULL;
}
static void rotate_left(struct memory_map *map, struct memory_map_node *n)
{
    struct memory_map_node *y = n->right;
    n->right                  = y->left;
    if (y->left != map->nil)
        y->left->parent = n;
    y->parent = n->parent;
    if (n->parent == map->nil)
        map->root = y;
    else if (n == n->parent->left)
        n->parent->left = y;
    else
        n->parent->right = y;
    y->left   = n;
    n->parent = y;
}
static void rotate_right(struct memory_map *map, struct memory_map_node *n)
{
    struct memory_map_node *y = n->left;
    n->left                   = y->right;
    if (y->right != map->nil)
        y->right->parent = n;
    y->parent = n->parent;
    if (n->parent == map->nil)
        map->root = y;
    else if (n == n->parent->left)
        n->parent->left = y;
    else
        n->parent->right = y;
    y->right  = n;
    n->parent = y;
}
