//*******************************************************
// Skypuff crash capture
//*******************************************************
// Tells where the module was when it got reset by a panic or a watchdog.
// - crumbs: bits set while in a sx isr, waiting on a sx BUSY pin or having a
//   sx selected on the SPI bus, plus the last SPI step and command byte
// - the panic handler is wrapped (-Wl,--wrap=esp_panic_handler) to keep the
//   PC and the callers of both cores, from the frames ESP-IDF saves per core,
//   and how far it got
// Both live in RTC memory, which survives a software reset but not a power loss.
// They are read once at the start, see skypuff_crash_capture(), and sent with the
// link telemetry. Enabled by SKYPUFF_CRASH_CAPTURE in platformio.ini.
//
// The ISR_SX crumbs also cover the handling of the sx event: on tx it is done in
// the main loop, by service_sx_dio(). skypuff_spi_reentries counts sx isrs which
// came in the middle of a SPI transfer. On rx such an isr does SPI itself and
// cuts into the transfer, on tx it only sets a flag.
//
// It's a debug tool and timing sensitive: the steps add a few RTC memory writes
// to every SPI transfer.
//*******************************************************
#ifndef SKYPUFF_CRASH_H
#define SKYPUFF_CRASH_H
#pragma once

#include <inttypes.h>


#define SKYPUFF_CRUMB_ISR_SX       0x01
#define SKYPUFF_CRUMB_ISR_SX2      0x02
#define SKYPUFF_CRUMB_BUSY_SX      0x04
#define SKYPUFF_CRUMB_BUSY_SX2     0x08
#define SKYPUFF_CRUMB_SEL_SX       0x10 // sx selected on the SPI bus
#define SKYPUFF_CRUMB_SEL_SX2      0x20
#define SKYPUFF_CRUMB_REENTRY      0x40 // the running sx isr interrupted a transfer of the main loop
#define SKYPUFF_CRUMB_VALID        0x80 // set in the report when the crumbs are from a crash

// SPI state: bit 7 chip (0: sx, 1: sx2), bit 6 in a sx isr, bits 0-2 step
#define SKYPUFF_SPI_SELECTED       1
#define SKYPUFF_SPI_TRANSFER       2 // inside the SPI driver
#define SKYPUFF_SPI_TRANSFERRED    3
#define SKYPUFF_SPI_DESELECTED     4
#define SKYPUFF_SPI_CMD_READ       0xFE // a read only transfer has no command byte

// How far the wrapped panic handler got
#define SKYPUFF_PANIC_ENTERED      1
#define SKYPUFF_PANIC_CORE0_DONE   2
#define SKYPUFF_PANIC_CORE1_DONE   3

#define SKYPUFF_CRASH_NO_CORE      0xFF
#define SKYPUFF_CRASH_CORE_NUM     2
#define SKYPUFF_CRASH_PC_NUM       6 // PC and 5 callers per core


typedef struct
{
    uint32_t pc[SKYPUFF_CRASH_CORE_NUM][SKYPUFF_CRASH_PC_NUM];  // 0 if unknown
    uint8_t core;          // core the panic is about, SKYPUFF_CRASH_NO_CORE if the panic handler didn't run
    uint8_t handler_core;  // core which ran the panic handler
    uint8_t crumbs;        // 0 if the last start wasn't a crash
    uint8_t panic_stage;   // 0 if the panic handler wasn't entered
    uint8_t spi_state;
    uint8_t spi_cmd;
} tSkypuffCrashReport;

tSkypuffCrashReport skypuff_crash_report = { {{0}}, SKYPUFF_CRASH_NO_CORE, SKYPUFF_CRASH_NO_CORE, 0, 0, 0, 0 };


#ifdef SKYPUFF_CRASH_CAPTURE

#include <esp_attr.h>
#include <esp_system.h>
#include <esp_debug_helpers.h>
#include <esp_private/panic_internal.h>
#include <xtensa/xtensa_context.h>
#include <freertos/FreeRTOS.h>

#define SKYPUFF_CRASH_MAGIC        0x5C7A5B03

typedef struct
{
    uint32_t magic;
    volatile uint8_t crumbs;
    volatile uint8_t spi_state;
    volatile uint8_t spi_cmd;
    uint8_t panic_stage;
    uint8_t core;
    uint8_t handler_core;
    uint32_t pc[SKYPUFF_CRASH_CORE_NUM][SKYPUFF_CRASH_PC_NUM];
} tSkypuffCrashRtc;

RTC_NOINIT_ATTR tSkypuffCrashRtc skypuff_crash_rtc;

volatile uint32_t skypuff_spi_reentries = 0;

#define SKYPUFF_CRUMB_SET(bits)    (skypuff_crash_rtc.crumbs |= (bits))
#define SKYPUFF_CRUMB_CLEAR(bits)  (skypuff_crash_rtc.crumbs &= (uint8_t)~(bits))

static inline IRAM_ATTR void skypuff_isr_enter(uint8_t isr_bit)
{
    if (skypuff_crash_rtc.crumbs & (SKYPUFF_CRUMB_SEL_SX | SKYPUFF_CRUMB_SEL_SX2)) {
        skypuff_spi_reentries++;
        isr_bit |= SKYPUFF_CRUMB_REENTRY;
    }
    SKYPUFF_CRUMB_SET(isr_bit);
}

static inline IRAM_ATTR void skypuff_isr_exit(uint8_t isr_bit)
{
    SKYPUFF_CRUMB_CLEAR(isr_bit | SKYPUFF_CRUMB_REENTRY);
}

static inline IRAM_ATTR void skypuff_spi_step(uint8_t chip, uint8_t step)
{
    bool in_isr = skypuff_crash_rtc.crumbs & (SKYPUFF_CRUMB_ISR_SX | SKYPUFF_CRUMB_ISR_SX2);
    skypuff_crash_rtc.spi_state = (chip ? 0x80 : 0) | (in_isr ? 0x40 : 0) | step;
}

static inline IRAM_ATTR void skypuff_spi_select(uint8_t chip)
{
    // Before the CS goes low, so an isr in between doesn't miss it
    SKYPUFF_CRUMB_SET(chip ? SKYPUFF_CRUMB_SEL_SX2 : SKYPUFF_CRUMB_SEL_SX);
    skypuff_spi_step(chip, SKYPUFF_SPI_SELECTED);
}

static inline IRAM_ATTR void skypuff_spi_deselect(uint8_t chip)
{
    SKYPUFF_CRUMB_CLEAR(chip ? SKYPUFF_CRUMB_SEL_SX2 : SKYPUFF_CRUMB_SEL_SX);
    skypuff_spi_step(chip, SKYPUFF_SPI_DESELECTED);
}

static inline IRAM_ATTR void skypuff_spi_transfer(uint8_t chip, uint8_t cmd)
{
    skypuff_crash_rtc.spi_cmd = cmd;
    skypuff_spi_step(chip, SKYPUFF_SPI_TRANSFER);
}

static inline IRAM_ATTR void skypuff_spi_transferred(uint8_t chip)
{
    skypuff_spi_step(chip, SKYPUFF_SPI_TRANSFERRED);
}


// Return addresses keep the call window size in the two top bits
static inline IRAM_ATTR uint32_t skypuff_crash_pc(uint32_t pc)
{
    return (pc & 0x80000000) ? ((pc & 0x3FFFFFFF) | 0x40000000) : pc;
}

static IRAM_ATTR void skypuff_crash_walk(const XtExcFrame* frame, uint32_t* pc)
{
    for (uint8_t i = 0; i < SKYPUFF_CRASH_PC_NUM; i++) pc[i] = 0;
    if (!frame) return;

    // The PC first, the walk below reads the stack and could fail on a broken one
    pc[0] = skypuff_crash_pc(frame->pc);

    esp_backtrace_frame_t bt;
    bt.pc = frame->pc;
    bt.sp = frame->a1;
    bt.next_pc = frame->a0;
    bt.exc_frame = frame;
    for (uint8_t i = 1; i < SKYPUFF_CRASH_PC_NUM; i++) {
        if (!bt.next_pc || !esp_backtrace_get_next_frame(&bt)) break;
        pc[i] = skypuff_crash_pc(bt.pc);
    }
}

extern "C" void __real_esp_panic_handler(panic_info_t* info);

extern "C" IRAM_ATTR void __wrap_esp_panic_handler(panic_info_t* info)
{
    uint8_t handler_core = (uint8_t)xPortGetCoreID();
    skypuff_crash_rtc.core = (uint8_t)info->core;
    skypuff_crash_rtc.handler_core = handler_core;
    skypuff_crash_rtc.panic_stage = SKYPUFF_PANIC_ENTERED;

    // Each core stores its frame when it enters the panic, a core which didn't leaves NULL
    for (uint8_t core = 0; core < SKYPUFF_CRASH_CORE_NUM; core++) {
        const void* frame = g_exc_frames[core];
        if (!frame && core == handler_core) frame = info->frame;
        skypuff_crash_walk((const XtExcFrame*)frame, skypuff_crash_rtc.pc[core]);
        skypuff_crash_rtc.panic_stage = (core == 0) ? SKYPUFF_PANIC_CORE0_DONE : SKYPUFF_PANIC_CORE1_DONE;
    }

    __real_esp_panic_handler(info);
}


// Must run before anything touches the sx chips, they overwrite the crumbs
void skypuff_crash_capture(void)
{
    esp_reset_reason_t reason = esp_reset_reason();
    bool crashed = (reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT ||
                    reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT);

    if (crashed && skypuff_crash_rtc.magic == SKYPUFF_CRASH_MAGIC) {
        skypuff_crash_report.crumbs = skypuff_crash_rtc.crumbs | SKYPUFF_CRUMB_VALID;
        skypuff_crash_report.spi_state = skypuff_crash_rtc.spi_state;
        skypuff_crash_report.spi_cmd = skypuff_crash_rtc.spi_cmd;
        skypuff_crash_report.panic_stage = skypuff_crash_rtc.panic_stage;
        if (skypuff_crash_rtc.panic_stage >= SKYPUFF_PANIC_ENTERED) {
            skypuff_crash_report.core = skypuff_crash_rtc.core;
            skypuff_crash_report.handler_core = skypuff_crash_rtc.handler_core;
            for (uint8_t core = 0; core < SKYPUFF_CRASH_CORE_NUM; core++) {
                for (uint8_t i = 0; i < SKYPUFF_CRASH_PC_NUM; i++) {
                    skypuff_crash_report.pc[core][i] = skypuff_crash_rtc.pc[core][i];
                }
            }
        }
    }

    skypuff_crash_rtc.magic = SKYPUFF_CRASH_MAGIC;
    skypuff_crash_rtc.crumbs = 0;
    skypuff_crash_rtc.spi_state = 0;
    skypuff_crash_rtc.spi_cmd = 0;
    skypuff_crash_rtc.panic_stage = 0;
    for (uint8_t core = 0; core < SKYPUFF_CRASH_CORE_NUM; core++) {
        for (uint8_t i = 0; i < SKYPUFF_CRASH_PC_NUM; i++) skypuff_crash_rtc.pc[core][i] = 0;
    }
}


#ifdef SKYPUFF_CRASH_SELFTEST_MS
// Test build only: hangs like a sx isr with the interrupts off, to check the capture.
// With SKYPUFF_CRASH_SELFTEST_LOOP the interrupts stay on, like a main loop stuck in plain
// code, to check the main loop watchdog.
void skypuff_crash_selftest(void)
{
    if (millis32() < SKYPUFF_CRASH_SELFTEST_MS) return;
    skypuff_isr_enter(SKYPUFF_CRUMB_ISR_SX2);
    skypuff_spi_select(1);
    skypuff_spi_transfer(1, 0x15);
#ifndef SKYPUFF_CRASH_SELFTEST_LOOP
    portDISABLE_INTERRUPTS();
#endif
    while (1) {}
}
#endif

#else

#define SKYPUFF_CRUMB_SET(bits)
#define SKYPUFF_CRUMB_CLEAR(bits)

const uint32_t skypuff_spi_reentries = 0;

static inline void skypuff_isr_enter(uint8_t isr_bit) {}
static inline void skypuff_isr_exit(uint8_t isr_bit) {}
static inline void skypuff_spi_select(uint8_t chip) {}
static inline void skypuff_spi_deselect(uint8_t chip) {}
static inline void skypuff_spi_transfer(uint8_t chip, uint8_t cmd) {}
static inline void skypuff_spi_transferred(uint8_t chip) {}

void skypuff_crash_capture(void) {}

#endif


// Tells which firmware build the crash PCs belong to, to pick the right .elf, and
// the board whether a release has another firmware. From the source for the
// Skypuff envs, see tools/skypuff_build_id.py: even for committed source. Else from
// the time of the build.
#if __has_include("skypuff_build_id.h")
#include "skypuff_build_id.h"

// In the image for scripts/publish_release.py of skypuff_rc
const char skypuff_build_marker[] = "SKYPUFF-BUILD-ID " SKYPUFF_BUILD_ID_STR;

inline uint32_t skypuff_build_id(void)
{
    // nothing reads the marker, this keeps the linker from dropping it
    __asm__ volatile("" : : "r"(skypuff_build_marker));
    return SKYPUFF_BUILD_ID;
}
#else
inline uint32_t skypuff_build_id(void)
{
    const char* s = __DATE__ " " __TIME__;
    uint32_t h = 2166136261u; // FNV-1a
    while (*s) {
        h ^= (uint8_t)*s++;
        h *= 16777619u;
    }
    return h;
}
#endif


#endif // SKYPUFF_CRASH_H
