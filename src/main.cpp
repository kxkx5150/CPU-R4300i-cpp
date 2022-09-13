#include "utils/common.h"
#include "utils/common/cart_db.h"
#include "utils/common/alloc.h"
#include "utils/common/rom_file.h"
#include "utils/common/save_file.h"
#include "utils/common/sha1.h"
#include "bus.h"
#include "device.h"
#include "pi.h"
#include "thread.h"
#include <cstdint>
#include <stdlib.h>

extern const struct n64_options default_n64_options;

enum cpu_extensions
{
    EXT_NONE = 0,
    EXT_SSE2,
    EXT_SSE3,
    EXT_SSSE3,
    EXT_SSE41,
    EXT_AVX
};
struct n64_options
{
    const char        *ddipl_path;
    const char        *ddrom_path;
    const char        *pifrom_path;
    const char        *cart_path;
    const char        *debugger_addr;
    const char        *eeprom_path;
    size_t             eeprom_size;
    const char        *sram_path;
    size_t             sram_size;
    const char        *flashram_path;
    int                is_viewer_output;
    struct controller *controller;
    bool               enable_debugger;
    bool               enable_profiling;
    bool               multithread;
    bool               no_audio;
    bool               no_video;
};

const uint8_t sha1_pifrom_ntsc[SHA1_SIZE] = {
    0x91, 0x74, 0xea, 0xdc, 0x0f, 0x0e, 0xa2, 0x65, 0x4c, 0x95,
    0xfd, 0x94, 0x14, 0x06, 0xab, 0x46, 0xb9, 0xdc, 0x9b, 0xdd,
};
const uint8_t sha1_pifrom_pal[SHA1_SIZE] = {
    0x46, 0xca, 0xe5, 0x9d, 0x31, 0xf9, 0x29, 0x8b, 0x93, 0xf3,
    0x38, 0x08, 0x79, 0x45, 0x4f, 0xce, 0xf5, 0x4e, 0xe6, 0xcc,
};
const uint8_t sha1_pifrom_ntsc_j[SHA1_SIZE] = {
    0X91, 0X74, 0Xea, 0Xdc, 0X0f, 0X0e, 0Xa2, 0X65, 0X4c, 0X95,
    0Xfd, 0X94, 0X14, 0X06, 0Xab, 0X46, 0Xb9, 0Xdc, 0X9b, 0Xdd,
};
const uint8_t sha1_dd_ipl_jp_v10[SHA1_SIZE] = {
    0x58, 0x67, 0x0c, 0x00, 0x63, 0x79, 0x3a, 0x8f, 0x3b, 0xe9,
    0x57, 0xd7, 0x1d, 0x93, 0x7b, 0x61, 0x88, 0x29, 0xba, 0x9e,
};
const uint8_t sha1_dd_ipl_jp_v11[SHA1_SIZE] = {0xb3, 0xe2, 0x6d, 0xbb, 0x4e, 0x94, 0x5f, 0x78, 0xc9, 0x18,
                                               0xfa, 0xbc, 0x5b, 0x9e, 0x60, 0xfc, 0xf2, 0x62, 0xc4, 0x7b};
const uint8_t sha1_dd_ipl_jp_v12[SHA1_SIZE] = {
    0xbf, 0x86, 0x19, 0x22, 0xdc, 0xb7, 0x8c, 0x31, 0x63, 0x60,
    0xe3, 0xe7, 0x42, 0xf4, 0xf7, 0x0f, 0xf6, 0x3c, 0x9b, 0xc3,
};
const uint8_t sha1_dd_ipl_us[SHA1_SIZE] = {0x3c, 0x5b, 0x93, 0xca, 0x23, 0x15, 0x50, 0xc6, 0x86, 0x93,
                                           0xa1, 0x4f, 0x03, 0xce, 0xa8, 0xd5, 0xdb, 0xd1, 0xbe, 0x9e};

const struct n64_options default_n64_options = {
    NULL, NULL, NULL, NULL, NULL, NULL, 0, NULL, 0, NULL, 0, NULL, false, false, false, true, false,
};

int parse_options(struct n64_options *, int argc, const char *argv[]);

static int   check_extensions(void);
static int   load_roms(const char *ddipl_path, const char *ddrom_path, const char *pifrom_path, const char *cart_path,
                       struct rom_file *ddipl, const struct dd_variant **dd_variant, struct rom_file *ddrom,
                       struct rom_file *pifrom, struct rom_file *cart);
static int   load_paks(struct controller *controller);
static int   validate_sha(struct rom_file *rom, const uint8_t *good_sum);
static int   run_device(struct n64_device *device, bool no_video);
static void *run_device_thread(void *opaque);
static int   parse_controller_options(const char *str, int *num, struct controller *opt);


int parse_options(struct n64_options *options, int argc, const char *argv[])
{
    int i;
    for (i = 0; i < argc - 1; i++) {
        if (!strcmp(argv[i], "-debug")) {
            options->enable_debugger = true;
            if ((i + 1) <= (argc - 1) && argv[i + 1][0] != '-')
                options->debugger_addr = argv[++i];
            else
                options->debugger_addr = "localhost:64646";
        } else if (!strcmp(argv[i], "-profile"))
            options->enable_profiling = true;
        else if (!strcmp(argv[i], "-multithread"))
            options->multithread = true;
        else if (!strcmp(argv[i], "-ddipl")) {
            if ((i + 1) >= (argc - 1)) {
                printf("-ddipl requires a path to the ROM file.\n\n");
                return 1;
            }
            options->ddipl_path = argv[++i];
        } else if (!strcmp(argv[i], "-ddrom")) {
            if ((i + 1) >= (argc - 1)) {
                printf("-ddrom requires a path to the ROM file.\n\n");
                return 1;
            }
            options->ddrom_path = argv[++i];
        } else if (!strcmp(argv[i], "-headless")) {
            options->no_audio = true;
            options->no_video = true;
        } else if (!strcmp(argv[i], "-noaudio"))
            options->no_audio = true;
        else if (!strcmp(argv[i], "-novideo"))
            options->no_video = true;
        else if (!strcmp(argv[i], "-eep4k")) {
            if ((i + 1) >= (argc - 1)) {
                printf("-eep4k requires a path to the save file.\n\n");
                return 1;
            }
            options->eeprom_path = argv[++i];
            options->eeprom_size = 0x200;
        } else if (!strcmp(argv[i], "-eep16k")) {
            if ((i + 1) >= (argc - 1)) {
                printf("-eep16k requires a path to the save file.\n\n");
                return 1;
            }
            options->eeprom_path = argv[++i];
            options->eeprom_size = 0x800;
        } else if (!strcmp(argv[i], "-sram")) {
            if ((i + 1) >= (argc - 1)) {
                printf("-sram requires a path to the save file.\n\n");
                return 1;
            }
            options->sram_path = argv[++i];
            options->sram_size = 0x8000;
        } else if (!strcmp(argv[i], "-sram256k")) {
            if ((i + 1) >= (argc - 1)) {
                printf("-sram256k requires a path to the save file.\n\n");
                return 1;
            }
            options->sram_path = argv[++i];
            options->sram_size = 0x8000;
        } else if (!strcmp(argv[i], "-sram768k")) {
            if ((i + 1) >= (argc - 1)) {
                printf("-sram768k requires a path to the save file.\n\n");
                return 1;
            }
            options->sram_path = argv[++i];
            options->sram_size = 0x18000;
        } else if (!strcmp(argv[i], "-sram1m")) {
            if ((i + 1) >= (argc - 1)) {
                printf("-sram1m requires a path to the save file.\n\n");
                return 1;
            }
            options->sram_path = argv[++i];
            options->sram_size = 0x20000;
        } else if (!strcmp(argv[i], "-flash")) {
            if ((i + 1) >= (argc - 1)) {
                printf("-flash requires a path to the save file.\n\n");
                return 1;
            }
            options->flashram_path = argv[++i];
        } else if (!strcmp(argv[i], "-is-viewer"))
            options->is_viewer_output = 1;
        else if (!strcmp(argv[i], "-controller")) {
            int               num;
            struct controller opt = {0};
            if ((i + 1) >= (argc - 1)) {
                printf("-controller requires a controller description.\n\n");
                return 1;
            }
            if (!parse_controller_options(argv[++i], &num, &opt)) {
                printf("Incorrect option format\n\n");
                return 1;
            }
            options->controller[num] = opt;
        } else
            break;
    }
    if (options->enable_debugger && options->multithread) {
        printf("Debugging not supported while using -multithread.\n");
        return 1;
    }
    options->pifrom_path = argv[i];
    if ((i + 1) < argc)
        options->cart_path = argv[i + 1];
    if (!options->ddipl_path && !options->ddrom_path && !options->cart_path)
        return 1;
    return 0;
}
int parse_controller_options(const char *str, int *num, struct controller *opt)
{
    char *token;
    char  mempak_path[4096];
    char  tpak_rom_path[4096] = {
         0,
    };
    char tpak_save_path[4096] = {
        0,
    };
    char *opt_string = strdup(str);
    if (opt_string == NULL) {
        printf("Unable to dup a string. You're gonna have trouble running games.\n");
        exit(1);
    }
    *num  = -1;
    token = strtok(opt_string, ",");
    while (token != NULL) {
        if (sscanf(token, "num=%d", num) == 1)
            ;
        else if (strcmp(token, "pak=rumble") == 0)
            opt->pak = PAK_RUMBLE;
        else if (strcmp(token, "pak=transfer") == 0)
            opt->pak = PAK_TRANSFER;
        else if (sscanf(token, "mempak=%4095s", mempak_path) == 1)
            opt->pak = PAK_MEM;
        else if (sscanf(token, "tpak_rom=%4095s", tpak_rom_path) == 1)
            opt->pak = PAK_TRANSFER;
        else if (sscanf(token, "tpak_save=%4095s", tpak_save_path) == 1)
            opt->pak = PAK_TRANSFER;
        else {
            printf("Unrecognized controller option: %s\n", token);
            free(opt_string);
            return 0;
        }
        token = strtok(NULL, ",");
    }
    if (*num < 1 || *num > 4) {
        printf("Controller number invalid or unspecified.\n");
        free(opt_string);
        return 0;
    }
    --*num;
    mempak_path[4095] = '\0';
    if (strlen(mempak_path) > 0)
        opt->mempak_path = strdup(mempak_path);
    tpak_rom_path[4095] = '\0';
    if (strlen(tpak_rom_path) > 0)
        opt->tpak_rom_path = strdup(tpak_rom_path);
    tpak_save_path[4095] = '\0';
    if (strlen(tpak_save_path) > 0)
        opt->tpak_save_path = strdup(tpak_save_path);
    opt->present = 1;
    free(opt_string);
    return 1;
}
static const char *_cpu_extensions_str(enum cpu_extensions ext)
{
    switch (ext) {
        case EXT_NONE:
            return "None";
        case EXT_SSE2:
            return "SSE2";
        case EXT_SSE3:
            return "SSE3";
        case EXT_SSSE3:
            return "SSSE3";
        case EXT_SSE41:
            return "SSE4.1";
        case EXT_AVX:
            return "AVX";
    }
    return "Unknown";
}
int check_extensions(void)
{
    return 0;
}
int load_roms(const char *ddipl_path, const char *ddrom_path, const char *pifrom_path, const char *cart_path,
              struct rom_file *ddipl, const struct dd_variant **dd_variant, struct rom_file *ddrom,
              struct rom_file *pifrom, struct rom_file *cart)
{
    memset(ddipl, 0, sizeof(*ddipl));
    if (ddipl_path && open_rom_file(ddipl_path, ddipl)) {
        printf("Failed to load DD IPL ROM: %s.\n", ddipl_path);
        return 1;
    }
    *dd_variant = NULL;
    if (ddipl_path != NULL) {
        *dd_variant = dd_identify_variant(ddipl);
        if (*dd_variant != NULL)
            printf("DD variant: %s\n", (*dd_variant)->description);
    }
    if (ddrom_path && open_rom_file(ddrom_path, ddrom)) {
        printf("Failed to load DD ROM: %s.\n", ddrom_path);
        if (ddipl_path)
            close_rom_file(ddipl);
        return 2;
    }
    if (open_rom_file(pifrom_path, pifrom)) {
        printf("Failed to load PIF ROM: %s.\n", pifrom_path);
        if (ddipl_path)
            close_rom_file(ddipl);
        if (ddrom_path)
            close_rom_file(ddrom);
        return 3;
    }
    if (validate_sha(pifrom, sha1_pifrom_ntsc))
        printf("Using NTSC-U PIFROM\n");
    else if (validate_sha(pifrom, sha1_pifrom_ntsc_j))
        printf("Using NTSC-J PIFROM\n");
    else if (validate_sha(pifrom, sha1_pifrom_pal))
        printf("Using PAL PIFROM\n");
    else {
        printf("Unknown or corrupted PIFROM: %s.\n", pifrom_path);
    }
    if (cart_path && open_rom_file(cart_path, cart)) {
        printf("Failed to load cart: %s.\n", cart_path);
        if (ddipl_path)
            close_rom_file(ddipl);
        if (ddrom_path)
            close_rom_file(ddrom);
        close_rom_file(pifrom);
        return 4;
    }
    return 0;
}
int load_paks(struct controller *controller)
{
    int i;
    for (i = 0; i < 4; ++i) {
        if (controller[i].pak == PAK_MEM && controller[i].mempak_path != NULL) {
            int created = 0;
            if (open_save_file(controller[i].mempak_path, MEMPAK_SIZE, &controller[i].mempak_save, &created) != 0) {
                printf("Can't open mempak file %s\n", controller[i].mempak_path);
                return -1;
            }
            if (created)
                controller_pak_format((uint8_t *)controller[i].mempak_save.ptr);
        } else if (controller[i].pak == PAK_TRANSFER) {
            if (controller[i].tpak_rom_path != NULL) {
                if (open_rom_file(controller[i].tpak_rom_path, &controller[i].tpak_rom)) {
                    printf("Can't open transfer pak ROM\n");
                    return -1;
                }
            } else {
                printf("No ROM supplied for transfer pak.\n");
                printf("The game will run but probably won't do anything interest\n");
            }
            if (controller[i].tpak_save_path != NULL) {
                if (open_gb_save(controller[i].tpak_save_path, &controller[i].tpak_save)) {
                    printf("Can't open transfer pak save\n");
                    return -1;
                }
            } else {
                printf("No save supplied for transfer pak. Just FYI.\n");
            }
            gb_init(&controller[i]);
        }
    }
    return 0;
}
int validate_sha(struct rom_file *rom, const uint8_t *good_sum)
{
    uint8_t sha1_calc[20];
    sha1((uint8_t *)rom->ptr, rom->size, sha1_calc);
    return memcmp(sha1_calc, good_sum, SHA1_SIZE) == 0;
}
int run_device(struct n64_device *device, bool no_video)
{
    n64_thread thread;
    device->running = true;
    bool cui_debug  = false;
    if (cui_debug) {
        device_run(device);
    } else {
        if (n64_thread_create(&thread, run_device_thread, device)) {
            printf("Failed to create the main emulation thread.\n");
            device_destroy(device, NULL);
            return 1;
        }
        n64_gl_window_thread(device);
    }
    device->running = false;
    n64_thread_join(&thread);
    return 0;
}
void *run_device_thread(void *opaque)
{
    struct n64_device *device = (struct n64_device *)opaque;
    device_run(device);
    return NULL;
}
int main(int argc, const char **argv)
{
    struct controller controller[4] = {
        {
            0,
        },
    };
    struct n64_options options = default_n64_options;
    options.controller         = controller;
    struct rom_file             ddipl, ddrom, pifrom, cart;
    const struct dd_variant    *dd_variant;
    struct n64_mem              n64_device_mem;
    struct n64_device          *device;
    int                         status;
    const struct cart_db_entry *cart_info;
    struct save_file            eeprom;
    struct save_file            sram;
    struct save_file            flashram;
    struct is_viewer            is, *is_in = NULL;

    if (!cart_db_is_well_formed()) {
        printf("Internal cart detection database is not well-formed.\n");
        return EXIT_FAILURE;
    }
    if (n64_alloc_init()) {
        printf("Failed to initialize the low-level allocators.\n");
        return EXIT_FAILURE;
    }
    if (check_extensions()) {
        return EXIT_FAILURE;
    }
    if (argc < 3) {
        n64_alloc_cleanup();
        return EXIT_SUCCESS;
    }
    if (parse_options(&options, argc - 1, argv + 1)) {
        printf("Invalid command line argument(s) specified.\n");
        n64_alloc_cleanup();
        return EXIT_FAILURE;
    }

    memset(&ddipl, 0, sizeof(ddipl));
    memset(&ddrom, 0, sizeof(ddrom));
    memset(&cart, 0, sizeof(cart));
    memset(&eeprom, 0, sizeof(eeprom));
    memset(&sram, 0, sizeof(sram));
    memset(&flashram, 0, sizeof(flashram));
    memset(&is, 0, sizeof(is));

    dd_variant = NULL;
    if (load_roms(options.ddipl_path, options.ddrom_path, options.pifrom_path, options.cart_path, &ddipl, &dd_variant,
                  &ddrom, &pifrom, &cart)) {
        n64_alloc_cleanup();
        return EXIT_FAILURE;
    }

    if (cart.size >= 0x40 && (cart_info = cart_db_get_entry((uint8_t *)cart.ptr)) != NULL) {
        printf("Detected cart: %s[%s] - %s\n", cart_info->rom_id, cart_info->regions, cart_info->description);
        enum cart_db_save_type save_type = cart_info->save_type;
        if (strcmp(cart_info->rom_id, "NK4") == 0) {
            uint8_t *rom = (uint8_t *)cart.ptr;
            if (rom[0x3e] == 'J' && rom[0x3f] < 2)
                save_type = CART_DB_SAVE_TYPE_SRAM_256KBIT;
        }
        switch (save_type) {
            case CART_DB_SAVE_TYPE_EEPROM_4KBIT:
                if (options.eeprom_path == NULL) {
                    printf("Warning: cart saves to 4kbit EEPROM, but none specified (see -eep4k)\n");
                    open_save_file(NULL, 0x200, &eeprom, NULL);
                } else {
                    if (options.eeprom_size != 0x200)
                        printf("Warning: cart saves to 4kbit EEPROM, but different size specified (see -eep4k)\n");
                }
                break;
            case CART_DB_SAVE_TYPE_EEPROM_16KBIT:
                if (options.eeprom_path == NULL) {
                    printf("Warning: cart saves to 16kbit EEPROM, but none specified (see -eep16k)\n");
                    open_save_file(NULL, 0x800, &eeprom, NULL);
                } else {
                    if (options.eeprom_size != 0x800)
                        printf("Warning: cart saves to 16kbit EEPROM, but different size specified (see -eep16k)\n");
                }
                break;
            case CART_DB_SAVE_TYPE_FLASH_1MBIT:
                if (options.flashram_path == NULL) {
                    int created;
                    printf("Warning: cart saves to Flash, but none specified (see -flash)\n");
                    open_save_file(NULL, FLASHRAM_SIZE, &flashram, &created);
                    if (created) {
                        memset(flashram.ptr, 0xFF, FLASHRAM_SIZE);
                    }
                }
                break;
            case CART_DB_SAVE_TYPE_SRAM_256KBIT:
                if (options.sram_path == NULL) {
                    printf("Warning: cart saves to 256kbit SRAM, but none specified (see -sram256k)\n");
                    open_save_file(NULL, 0x8000, &sram, NULL);
                } else if (options.sram_size != 0x8000) {
                    printf("Warning: cart saves to 256kbit SRAM, but different size specified (see -sram256k)\n");
                }
                break;
            case CART_DB_SAVE_TYPE_SRAM_768KBIT:
                if (options.sram_path == NULL) {
                    printf("Warning: cart saves to 768kbit SRAM, but none specified (see -sram768k)\n");
                    open_save_file(NULL, 0x18000, &sram, NULL);
                } else if (options.sram_size != 0x18000) {
                    printf("Warning: cart saves to 768kbit SRAM, but different size specified (see -sram768k)\n");
                }
                break;
            case CART_DB_SAVE_TYPE_SRAM_1MBIT:
                if (options.sram_path == NULL) {
                    printf("Warning: cart saves to 1mbit SRAM, but none specified (see -sram1m)\n");
                    open_save_file(NULL, 0x20000, &sram, NULL);
                } else if (options.sram_size != 0x20000) {
                    printf("Warning: cart saves to 1mbit SRAM, but different size specified (see -sram1m)\n");
                }
                break;
            case CART_DB_SAVE_TYPE_NONE:
                break;
        }
    }
    if (load_paks(controller)) {
        n64_alloc_cleanup();
        return EXIT_FAILURE;
    }
    if (options.eeprom_path != NULL && open_save_file(options.eeprom_path, options.eeprom_size, &eeprom, NULL)) {
        n64_alloc_cleanup();
        return EXIT_FAILURE;
    }
    if (options.sram_path != NULL && open_save_file(options.sram_path, options.sram_size, &sram, NULL)) {
        n64_alloc_cleanup();
        return EXIT_FAILURE;
    }
    if (options.flashram_path != NULL) {
        int created;
        if (open_save_file(options.flashram_path, FLASHRAM_SIZE, &flashram, &created)) {
            n64_alloc_cleanup();
            return EXIT_FAILURE;
        }
        if (created)
            memset(flashram.ptr, 0xFF, FLASHRAM_SIZE);
    }
    if (!is_viewer_init(&is, options.is_viewer_output)) {
        n64_alloc_cleanup();
        return EXIT_FAILURE;
    } else {
        is_in = &is;
    }
    if (n64_alloc(&n64_device_mem, sizeof(*device), false) == NULL) {
        printf("Failed to allocate enough memory for a device.\n");
        status = EXIT_FAILURE;
    } else {
        device = (struct n64_device *)n64_device_mem.ptr;
        if (device_create(device, &ddipl, dd_variant, &ddrom, &pifrom, &cart, &eeprom, &sram, &flashram, is_in,
                          controller, options.no_audio, options.no_video, options.enable_profiling) == NULL) {
            printf("Failed to create a device.\n");
            status = EXIT_FAILURE;
        } else {
            device->multithread = options.multithread;
            status              = run_device(device, options.no_video);
            device_destroy(device, options.cart_path);
        }
        n64_free(&n64_device_mem);
    }
    if (options.ddipl_path)
        close_rom_file(&ddipl);
    if (options.ddrom_path)
        close_rom_file(&ddrom);
    if (options.cart_path)
        close_rom_file(&cart);
    close_rom_file(&pifrom);
    n64_alloc_cleanup();
    return status;
}
