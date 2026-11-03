//*******************************************************
// Skypuff link telemetry, from the module to its ESP board on the serial port
//*******************************************************
// Goes only over the wire, it doesn't use the radio link.
// The serial stream carries COBS frames coming from the air, so our frame must
// not be put inside of one. It is postponed until the stream is at a frame
// boundary. If the stream is stuck mid-frame, e.g. when the link is lost, the
// partial frame is terminated, the ESP rejects it by its crc.
//*******************************************************
#ifndef SKYPUFF_TELEMETRY_H
#define SKYPUFF_TELEMETRY_H
#pragma once


#include "skypuff_frame.h"
#include "skypuff_crash.h"
#include "skypuff_radio_guard.h"
#include "skypuff_bind.h"
#ifdef ESP32
#include <esp_system.h>
#endif
#if defined ESP32 && defined CONFIG_IDF_TARGET_ESP32
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/syscon_reg.h"
#endif


// trng_init() switches the SAR ADC into its entropy mode, and bootloader_random_disable()
// doesn't restore all of it (checked by disassembly of the esp32 SDK). On the tx module
// the chip temperature sensor then always reads 128 F = 53 C. This keeps the registers
// bootloader_random_enable() writes, and puts them back afterwards.
class tSkypuffAdcState
{
  public:
#if defined ESP32 && defined CONFIG_IDF_TARGET_ESP32
    void Save(void) { for (uint8_t i = 0; i < REG_NUM; i++) values[i] = REG_READ(reg(i)); }
    void Restore(void) { for (uint8_t i = 0; i < REG_NUM; i++) REG_WRITE(reg(i), values[i]); }

  private:
    static constexpr uint8_t REG_NUM = 11;

    static uint32_t reg(uint8_t i)
    {
        static const uint32_t regs[REG_NUM] = {
            RTC_CNTL_TEST_MUX_REG,
            SENS_SAR_READ_CTRL_REG,
            SENS_SAR_READ_CTRL2_REG,
            SENS_SAR_MEAS_WAIT2_REG,
            SENS_SAR_START_FORCE_REG,
            SYSCON_SARADC_CTRL_REG,
            SYSCON_SARADC_FSM_REG,
            SYSCON_SARADC_SAR2_PATT_TAB1_REG,
            SYSCON_SARADC_SAR2_PATT_TAB2_REG,
            SYSCON_SARADC_SAR2_PATT_TAB3_REG,
            SYSCON_SARADC_SAR2_PATT_TAB4_REG,
        };
        return regs[i];
    }

    uint32_t values[REG_NUM];
#else
    void Save(void) {}
    void Restore(void) {}
#endif
};


// The board restarted without the module and asks again, see skypuff_updater_sniff()
volatile bool skypuff_module_info_requested = false;


class tSkypuffTelemetry
{
  public:
    // _ser can be nullptr, then no telemetry is sent
    void Init(tSerialBase* const _ser, uint8_t _src)
    {
        ser = _ser;
        src = _src;
        mid_frame = false;
        seq = 0;
        info_left = SKYPUFF_MODULE_INFO_FRAMES;
        tlast_send_ms = millis32();
        tlast_byte_ms = tlast_send_ms;
#ifdef ESP32
        reset_reason = (uint8_t)esp_reset_reason();
#else
        reset_reason = 0;
#endif
    }

    void OnSerialOut(const uint8_t* const buf, uint16_t len)
    {
        if (len == 0) return;
        mid_frame = (buf[len - 1] != 0x00);
        tlast_byte_ms = millis32();
    }

    void Do(bool is_connected)
    {
#ifdef SKYPUFF_CRASH_SELFTEST_MS
        skypuff_crash_selftest();
#endif
        if (!ser) return;
        if (Setup.Rx.SerialLinkMode != SERIAL_LINK_MODE_TRANSPARENT) return;

        uint32_t tnow_ms = millis32();
        if ((tnow_ms - tlast_send_ms) < SKYPUFF_TELEMETRY_PERIOD_MS) return;

        if (mid_frame) {
            if ((tnow_ms - tlast_byte_ms) < SKYPUFF_TELEMETRY_MID_FRAME_TMO_MS) return; // postpone
            uint8_t terminator = 0x00;
            ser->putbuf(&terminator, 1);
            mid_frame = false;
        }

        tlast_send_ms = tnow_ms;

        tSkypuffLinkStatus status;
        status.uptime_ms = tnow_ms;
        status.bytes_per_sec_received = clip_u16(stats.bytes_received.GetBytesPerSec());
        status.bytes_per_sec_transmitted = clip_u16(stats.bytes_transmitted.GetBytesPerSec());
        status.connected = (is_connected) ? 1 : 0;
#ifdef DEVICE_IS_RECEIVER
        status.LQ_rc = stats.GetLQ_rc();
#else
        status.LQ_rc = stats.valid_frames_received.GetLQ(); // tx has no crc1 statistics
#endif
        status.LQ_serial = stats.GetLQ_serial();
        status.rssi1 = stats.last_rssi1;
        status.rssi2 = stats.last_rssi2;
        status.snr1 = stats.last_snr1;
        status.snr2 = stats.last_snr2;
        status.antenna = stats.last_antenna;
        status.transmit_antenna = stats.last_transmit_antenna;
        status.received_LQ_serial = stats.received_LQ_serial;
        status.received_rssi = stats.received_rssi;
        status.power_dbm = Config.Sx.Power_dbm;
        status.frame_rate_hz = (uint8_t)Config.frame_rate_hz;
        status.mcu_temperature_c = read_temperature();
        status.reset_reason = reset_reason;
        for (uint8_t core = 0; core < SKYPUFF_CRASH_CORE_NUM; core++) {
            for (uint8_t i = 0; i < SKYPUFF_CRASH_PC_NUM; i++) status.crash_pc[core][i] = skypuff_crash_report.pc[core][i];
        }
        status.build_id = skypuff_build_id();
        status.crash_core = skypuff_crash_report.core;
        status.crash_handler_core = skypuff_crash_report.handler_core;
        status.crash_crumbs = skypuff_crash_report.crumbs;
        status.crash_panic_stage = skypuff_crash_report.panic_stage;
        status.crash_spi_state = skypuff_crash_report.spi_state;
        status.crash_spi_cmd = skypuff_crash_report.spi_cmd;
        status.spi_reentries = clip_u16(skypuff_spi_reentries);
        status.busy_timeouts = skypuff_busy_timeouts;
        status.radio_restarts = skypuff_radio_restarts;

        uint8_t encoded[SKYPUFF_FRAME_MAX_ENCODED_LEN];
        uint16_t len = skypuff_frame_encode(SKYPUFF_FRAME_TYPE_LINK_STATUS, src, seq++,
                                            &status, sizeof(status), encoded);
        ser->putbuf(encoded, len);

        // What doesn't change, only at the start and on request
        if (skypuff_module_info_requested) {
            skypuff_module_info_requested = false;
            if (!info_left) info_left = 1;
        }
        if (info_left) {
            info_left--;
            send_module_info();
        }
    }

  private:
    void send_module_info(void)
    {
        tSkypuffModuleInfo info = {};
        info.build_id = skypuff_build_id();
        strncpy(info.version, VERSIONONLYSTR, sizeof(info.version));
#ifdef ESP32
        memcpy(info.bind_phrase, skypuff_bind_phrase, strlen(skypuff_bind_phrase));
#endif
#ifdef SKYPUFF_BIND_UNIVERSAL
        info.universal = 1;
#endif
        uint8_t encoded[SKYPUFF_FRAME_MAX_ENCODED_LEN];
        uint16_t len = skypuff_frame_encode(SKYPUFF_FRAME_TYPE_MODULE_INFO, src, seq++,
                                            &info, sizeof(info), encoded);
        ser->putbuf(encoded, len);
    }

    int8_t read_temperature(void)
    {
#ifdef ESP32
        float t = temperatureRead();
        return (t < -127.0f) ? -127 : (t > 127.0f) ? 127 : (int8_t)lroundf(t);
#else
        return SKYPUFF_TEMPERATURE_UNKNOWN;
#endif
    }

    static uint16_t clip_u16(uint32_t v) { return (v > UINT16_MAX) ? UINT16_MAX : (uint16_t)v; }

    tSerialBase* ser;
    uint8_t src;
    bool mid_frame;
    uint8_t seq;
    uint8_t info_left;
    uint32_t tlast_send_ms;
    uint32_t tlast_byte_ms;
    uint8_t reset_reason;
};


#endif // SKYPUFF_TELEMETRY_H
