//*******************************************************
// Skypuff start trace
//*******************************************************
// Bench builds only (SKYPUFF_BOOT_TRACE): prints how far the module gets after its
// start as "E (ms) trace: ..." lines on the serial port, mixed into the telemetry.
// Goes straight to the ESP-IDF log like the core's own errors: Arduino's ESP_LOGE goes
// silent once the serial port was ended. Not for interrupts.
//*******************************************************
#ifndef SKYPUFF_TRACE_H
#define SKYPUFF_TRACE_H
#pragma once


#ifdef SKYPUFF_BOOT_TRACE

#include <esp_log.h>
#include <esp_system.h>
#include <rom/rtc.h>

#define SKYPUFF_TRACE(fmt, ...)  esp_log_write(ESP_LOG_ERROR, "trace", "E (%u) trace: " fmt "\r\n", esp_log_timestamp(), ##__VA_ARGS__)

static void skypuff_trace_shutdown(void)
{
    SKYPUFF_TRACE("esp_restart() called");
}

// Call once at the start
static void skypuff_trace_start(void)
{
    esp_register_shutdown_handler(skypuff_trace_shutdown);
    SKYPUFF_TRACE("start, reset reason %d, rom reasons %d/%d", (int)esp_reset_reason(),
                  (int)rtc_get_reset_reason(0), (int)rtc_get_reset_reason(1));
}

#else

#define SKYPUFF_TRACE(fmt, ...)
static inline void skypuff_trace_start(void) {}

#endif


#endif // SKYPUFF_TRACE_H
