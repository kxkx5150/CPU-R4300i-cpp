#ifndef __common_h__
#define __common_h__
#define tostring(s)  #s
#define stringify(s) tostring(s)
#ifndef __cplusplus
#include <assert.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#else
#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <cstring>
#endif
#ifndef _MSC_VER
#ifndef __cplusplus
#include <stdbool.h>
#else
#include <cstdbool>
#endif
#else
typedef char bool;
#define false 0
#define true 1
#endif
#define CEN64_COMPILER  "gcc-" stringify(__GNUC__) "." stringify(__GNUC_MINOR__) "." stringify(__GNUC_PATCHLEVEL__)
#define CACHE_LINE_SIZE 64
#ifdef _MSC_VER
#define n64_align(decl, value) __declspec(align(value)) decl
#elif (defined __GNUC__)
#define n64_align(decl, value) decl __attribute__((aligned(value)))
#else
#define n64_align(decl, value) decl value
#endif
#ifdef __GNUC__
#else
#define n64_cold
#define n64_hot
#endif
#ifdef __GNUC__
#else
#define n64_flatten
#endif
#ifdef __GNUC__
#define likely(expr)   __builtin_expect((expr), !0)
#define unlikely(expr) __builtin_expect((expr), 0)
#else
#define likely(expr)   expr
#define unlikely(expr) expr
#endif
#ifdef __GNUC__
#define unused(decl) __attribute__((unused)) decl
#else
#define unused(decl) decl
#endif
#ifdef BIG_ENDIAN_HOST
#define WORD_ADDR_XOR 0
#else
#define WORD_ADDR_XOR 4
#endif
#ifndef _MSC_VER
#ifdef BIG_ENDIAN_HOST
#define htonll(x) (x)
#define ntohll(x) (x)
#else
#ifndef __APPLE__
#define htonll(x) __builtin_bswap64(x)
#define ntohll(x) __builtin_bswap64(x)
#endif
#endif
#endif
#ifdef __GNUC__
__attribute__((pure))
#endif
static inline uint32_t
byteswap_32(uint32_t word)
{
#ifdef BIG_ENDIAN_HOST
    return word;
#elif defined(_MSC_VER)
    return _byteswap_ulong(word);
#elif defined(__GNUC__)
    return __builtin_bswap32(word);
#else
  return
  (((((word) >> 24) & 0x000000FF) | \
    (((word) >>  8) & 0x0000FF00) | \
    (((word) <<  8) & 0x00FF0000) | \
    (((word) << 24) & 0xFF000000));
#endif
}
#ifdef __GNUC__
__attribute__((pure))
#endif
static inline uint16_t
byteswap_16(uint16_t hword)
{
#ifdef BIG_ENDIAN_HOST
    return hword;
#elif defined(_MSC_VER)
    return _byteswap_ushort(hword);
#elif defined(__GNUC__)
    return __builtin_bswap16(hword);
#else
    return ((((hword) >> 8) & 0x00FF) | (((hword) << 8) & 0xFF00));
#endif
}
struct bus_controller;
#ifdef __GNUC__
__attribute__ ((noreturn))
#endif
void n64_return(struct bus_controller *bus)
;
#ifdef DEBUG_MMIO_REGISTER_ACCESS
#ifndef __cplusplus
#include <stdio.h>
#else
#include <cstdio>
#endif
#define debug_mmio_read(what, mnemonic, val)       printf(#what ": READ [%s]: 0x%.8X\n", mnemonic, val)
#define debug_mmio_write(what, mnemonic, val, dqm) printf(#what ": WRITE [%s]: 0x%.8X/0x%.8X\n", mnemonic, val, dqm)
#else
#define debug_mmio_read(what, mnemonic, val)                                                                           \
    do {                                                                                                               \
    } while (0)
#define debug_mmio_write(what, mnemonic, val, dqm)                                                                     \
    do {                                                                                                               \
    } while (0)
#endif
#endif
