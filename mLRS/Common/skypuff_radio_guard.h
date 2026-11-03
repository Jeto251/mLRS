//*******************************************************
// Skypuff radio guard
//*******************************************************
// The sx chips are waited on their BUSY pin before every SPI command, originally
// without a limit. On tx all the SPI work is done in the main loop, nothing would
// get a sx stuck on BUSY out of there, so the wait is limited by SKYPUFF_BUSY_TMO_US.
// Repeated timeouts restart the radio part: init_hw() resets both chips by their
// reset pin and sets them up again. A restart which doesn't help reboots the module.
// The counters are sent with the link telemetry.
//*******************************************************
#ifndef SKYPUFF_RADIO_GUARD_H
#define SKYPUFF_RADIO_GUARD_H
#pragma once

#include <inttypes.h>
#include "skypuff_trace.h"


#if defined SKYPUFF_BUSY_TMO_US && defined DEVICE_IS_TRANSMITTER
#define SKYPUFF_BUSY_BOUNDED
#endif

#define SKYPUFF_RADIO_KEEP          0
#define SKYPUFF_RADIO_RESTART       1 // restart the radio part
#define SKYPUFF_RADIO_REBOOT        2 // the last restart didn't help, reboot the module

#define SKYPUFF_BUSY_TMO_RESTART_CNT    3    // timeouts within the window which restart the radio
#define SKYPUFF_BUSY_TMO_WINDOW_MS      1000
#define SKYPUFF_RADIO_RESTART_HELD_MS   10000 // a restart is taken as failed if needed again so soon

// Since the module started
volatile uint16_t skypuff_busy_timeouts = 0;
volatile uint8_t skypuff_radio_restarts = 0;

#ifdef SKYPUFF_BUSY_BOUNDED

static uint32_t skypuff_busy_tmo_window_ms = 0;
static uint8_t skypuff_busy_tmo_window_cnt = 0;
static bool skypuff_radio_restart_requested = false;
static bool skypuff_radio_restarted_once = false;
static uint32_t skypuff_radio_restart_ms = 0;

void skypuff_busy_timeout(void)
{
    if (skypuff_busy_timeouts < UINT16_MAX) skypuff_busy_timeouts++;
    if (skypuff_busy_timeouts <= 20) SKYPUFF_TRACE("busy timeout %u", skypuff_busy_timeouts);

    uint32_t tnow_ms = millis32();
    if (tnow_ms - skypuff_busy_tmo_window_ms > SKYPUFF_BUSY_TMO_WINDOW_MS) {
        skypuff_busy_tmo_window_ms = tnow_ms;
        skypuff_busy_tmo_window_cnt = 0;
    }
    if (++skypuff_busy_tmo_window_cnt >= SKYPUFF_BUSY_TMO_RESTART_CNT) skypuff_radio_restart_requested = true;
}

// Called by the main loop, outside of the inits
uint8_t skypuff_radio_guard_action(void)
{
    if (!skypuff_radio_restart_requested) return SKYPUFF_RADIO_KEEP;
    skypuff_radio_restart_requested = false;
    skypuff_busy_tmo_window_cnt = 0;

    uint32_t tnow_ms = millis32();
    if (skypuff_radio_restarted_once && tnow_ms - skypuff_radio_restart_ms < SKYPUFF_RADIO_RESTART_HELD_MS) {
        SKYPUFF_TRACE("radio guard: reboot");
        return SKYPUFF_RADIO_REBOOT;
    }
    skypuff_radio_restarted_once = true;
    skypuff_radio_restart_ms = tnow_ms;
    if (skypuff_radio_restarts < UINT8_MAX) skypuff_radio_restarts++;
    SKYPUFF_TRACE("radio guard: restart");
    return SKYPUFF_RADIO_RESTART;
}

#else

uint8_t skypuff_radio_guard_action(void) { return SKYPUFF_RADIO_KEEP; }

#endif


#endif // SKYPUFF_RADIO_GUARD_H
