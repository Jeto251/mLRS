//*******************************************************
// Skypuff service frames, sent by the module to the ESP board on its serial port
//*******************************************************
// Must match components/skypuff/radio_frame.h in skypuff_rc.
//
//   COBS( ver | type:4 src:4 | seq | payload... | crc16 (big-endian) ) + 0x00
//
// The stored crc is inverted, see radio_frame.cpp for why.
//*******************************************************
#ifndef SKYPUFF_FRAME_H
#define SKYPUFF_FRAME_H
#pragma once

#include <inttypes.h>
#include <stddef.h>


#define SKYPUFF_FRAME_VERSION           1
#define SKYPUFF_FRAME_TYPE_LINK_STATUS  1
#define SKYPUFF_FRAME_TYPE_MODULE_INFO  2 // tSkypuffModuleInfo, with the first SKYPUFF_MODULE_INFO_FRAMES link status frames, and on request
#define SKYPUFF_FRAME_SRC_TX_MODULE     1
#define SKYPUFF_FRAME_SRC_RX_MODULE     2

#define SKYPUFF_TEMPERATURE_UNKNOWN     INT8_MIN

#define SKYPUFF_FRAME_HEADER_LEN        3
#define SKYPUFF_FRAME_CRC_LEN           2
#define SKYPUFF_FRAME_MAX_PAYLOAD_LEN   96 // keeps frames below the 254 bytes of a COBS block
// worst case: COBS code byte + header + payload + crc + terminator
#define SKYPUFF_FRAME_MAX_ENCODED_LEN   (1 + SKYPUFF_FRAME_HEADER_LEN + SKYPUFF_FRAME_MAX_PAYLOAD_LEN + SKYPUFF_FRAME_CRC_LEN + 1)


typedef struct __attribute__((packed))
{
    uint32_t uptime_ms;
    uint16_t bytes_per_sec_received;
    uint16_t bytes_per_sec_transmitted;
    uint8_t connected;
    uint8_t LQ_rc;
    uint8_t LQ_serial;
    int8_t rssi1;
    int8_t rssi2;
    int8_t snr1;
    int8_t snr2;
    uint8_t antenna;
    uint8_t transmit_antenna;
    uint8_t received_LQ_serial; // reported by the other module
    int8_t received_rssi;       // reported by the other module
    int8_t power_dbm;
    uint8_t frame_rate_hz;
    int8_t mcu_temperature_c;   // module's ESP32, rough value, SKYPUFF_TEMPERATURE_UNKNOWN if not available
    uint8_t reset_reason;       // why the module last started, esp_reset_reason_t, 0 if not available
    uint32_t crash_pc[2][6];    // where both cores were when the module crashed before the last start, see skypuff_crash.h
    uint32_t build_id;          // firmware build the crash PCs belong to
    uint8_t crash_core;         // core the panic is about, 0xFF if unknown
    uint8_t crash_handler_core; // core which ran the panic handler
    uint8_t crash_crumbs;       // 0 if the last start wasn't a crash
    uint8_t crash_panic_stage;  // how far the panic handler got, 0 if it wasn't entered
    uint8_t crash_spi_state;    // last SPI step before the crash
    uint8_t crash_spi_cmd;      // last SPI command byte before the crash
    uint16_t spi_reentries;     // sx isr started a SPI transfer in the middle of one of the main loop
    uint16_t busy_timeouts;     // waits on a sx BUSY pin which ran out, see skypuff_radio_guard.h
    uint8_t radio_restarts;     // radio part restarts because of them
} tSkypuffLinkStatus; // 86 bytes


// What the module is, for the board's settings
typedef struct __attribute__((packed))
{
    uint32_t build_id;
    char version[12];         // VERSIONONLYSTR
    char bind_phrase[6];      // the module's, all 0 if it has none, see skypuff_bind.h
    uint8_t universal;        // 1: a universal firmware, it keeps the module's phrase
} tSkypuffModuleInfo; // 23 bytes


// same as crc16() in the VESC firmware: CCITT, poly 0x1021, init 0
inline uint16_t skypuff_crc16(const uint8_t* buf, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}


// Writes ver | type:4 src:4 | seq | payload | crc16 to out, which must hold
// payload_len + SKYPUFF_FRAME_HEADER_LEN + SKYPUFF_FRAME_CRC_LEN bytes. Returns the frame length.
inline uint16_t skypuff_frame_build(uint8_t type, uint8_t src, uint8_t seq, const void* payload, uint16_t payload_len, uint8_t* out)
{
    out[0] = SKYPUFF_FRAME_VERSION;
    out[1] = (uint8_t)((type << 4) | (src & 0x0F));
    out[2] = seq;
    for (uint16_t i = 0; i < payload_len; i++) out[SKYPUFF_FRAME_HEADER_LEN + i] = ((const uint8_t*)payload)[i];

    uint16_t crc_pos = SKYPUFF_FRAME_HEADER_LEN + payload_len;
    uint16_t crc = skypuff_crc16(out, crc_pos) ^ 0xFFFF;
    out[crc_pos] = (uint8_t)(crc >> 8);
    out[crc_pos + 1] = (uint8_t)(crc & 0xFF);
    return crc_pos + SKYPUFF_FRAME_CRC_LEN;
}

// Checks the crc of a decoded frame. On success points payload into it.
inline bool skypuff_frame_parse(const uint8_t* frame, uint16_t len, uint8_t* type, uint8_t* src, const uint8_t** payload, uint16_t* payload_len)
{
    if (len < SKYPUFF_FRAME_HEADER_LEN + SKYPUFF_FRAME_CRC_LEN) return false;
    uint16_t crc_pos = len - SKYPUFF_FRAME_CRC_LEN;
    uint16_t crc = (uint16_t)((frame[crc_pos] << 8) | frame[crc_pos + 1]);
    if ((uint16_t)(skypuff_crc16(frame, crc_pos) ^ 0xFFFF) != crc) return false;
    if (frame[0] != SKYPUFF_FRAME_VERSION) return false;
    *type = frame[1] >> 4;
    *src = frame[1] & 0x0F;
    *payload = frame + SKYPUFF_FRAME_HEADER_LEN;
    *payload_len = crc_pos - SKYPUFF_FRAME_HEADER_LEN;
    return true;
}

// out must hold len + len / 254 + 2 bytes. Returns the length including the 0x00 terminator.
inline uint16_t skypuff_cobs_encode(const uint8_t* in, uint16_t len, uint8_t* out)
{
    uint16_t code_pos = 0;
    uint16_t pos = 1;
    uint8_t code = 1;
    for (uint16_t i = 0; i < len; i++) {
        if (in[i] == 0) {
            out[code_pos] = code;
            code_pos = pos++;
            code = 1;
        } else {
            out[pos++] = in[i];
            if (++code == 0xFF) { // a block holds 254 data bytes at most
                out[code_pos] = code;
                code_pos = pos++;
                code = 1;
            }
        }
    }
    out[code_pos] = code;
    out[pos++] = 0x00;
    return pos;
}

// in is the frame without its 0x00 terminator. Returns the decoded length, 0 if malformed.
inline uint16_t skypuff_cobs_decode(const uint8_t* in, uint16_t len, uint8_t* out)
{
    uint16_t i = 0;
    uint16_t pos = 0;
    while (i < len) {
        uint8_t code = in[i];
        if (code == 0 || i + code > len) return 0;
        i++;
        for (uint8_t k = 1; k < code; k++) out[pos++] = in[i++];
        if (code != 0xFF && i < len) out[pos++] = 0;
    }
    return pos;
}

// Standard CRC-32 (zlib, Python zlib.crc32), start with crc = 0
inline uint32_t skypuff_crc32(uint32_t crc, const uint8_t* buf, uint32_t len)
{
    crc = ~crc;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : (crc >> 1);
    }
    return ~crc;
}

// out must hold SKYPUFF_FRAME_MAX_ENCODED_LEN bytes, payload_len must not exceed
// SKYPUFF_FRAME_MAX_PAYLOAD_LEN. Returns the length including the terminator.
inline uint16_t skypuff_frame_encode(uint8_t type, uint8_t src, uint8_t seq, const void* payload, uint8_t payload_len, uint8_t* out)
{
    uint8_t frame[SKYPUFF_FRAME_HEADER_LEN + SKYPUFF_FRAME_MAX_PAYLOAD_LEN + SKYPUFF_FRAME_CRC_LEN];
    uint16_t frame_len = skypuff_frame_build(type, src, seq, payload, payload_len, frame);
    return skypuff_cobs_encode(frame, frame_len, out);
}


//-------------------------------------------------------
// Module update over the serial port, see skypuff_updater.h
//-------------------------------------------------------
// Payload byte 0 is the command, the rest little-endian.

#define SKYPUFF_FRAME_TYPE_UPDATE       5
#define SKYPUFF_FRAME_SRC_ESP           0

#define SKYPUFF_UPDATE_PROTO_VERSION    1
#define SKYPUFF_UPDATE_CHUNK_LEN        1024 // DATA bytes per frame

// board -> module
#define SKYPUFF_UPDATE_PING             0x01 // -> READY
#define SKYPUFF_UPDATE_BEGIN            0x02 // u32 size, u32 crc32 -> ACK(0)
#define SKYPUFF_UPDATE_DATA             0x03 // u32 offset, data -> ACK(next offset)
#define SKYPUFF_UPDATE_END              0x04 // -> DONE, the module restarts into the new firmware
#define SKYPUFF_UPDATE_CONFIRM          0x05 // -> CONFIRMED, the new firmware stays
#define SKYPUFF_UPDATE_ABORT            0x06 // -> the module boots on
#define SKYPUFF_UPDATE_RESTART          0x07 // to the running firmware: restart into a long update window
#define SKYPUFF_UPDATE_SET_BIND         0x08 // char[6] bind phrase -> BIND_SET, see skypuff_bind.h
#define SKYPUFF_UPDATE_INFO_REQUEST     0x09 // to the running firmware: send tSkypuffModuleInfo again

// module -> board
#define SKYPUFF_UPDATE_READY            0x81
#define SKYPUFF_UPDATE_ACK              0x82 // u32 next offset
#define SKYPUFF_UPDATE_ERROR            0x83 // u8 code, i32 esp_err_t
#define SKYPUFF_UPDATE_DONE             0x84
#define SKYPUFF_UPDATE_CONFIRMED        0x85
#define SKYPUFF_UPDATE_BIND_SET         0x86

#define SKYPUFF_UPDATE_ERR_PENDING      1 // the running firmware isn't confirmed yet
#define SKYPUFF_UPDATE_ERR_TOO_BIG      2
#define SKYPUFF_UPDATE_ERR_BEGIN        3
#define SKYPUFF_UPDATE_ERR_WRITE        4
#define SKYPUFF_UPDATE_ERR_NOT_STARTED  5
#define SKYPUFF_UPDATE_ERR_SIZE         6
#define SKYPUFF_UPDATE_ERR_CRC          7
#define SKYPUFF_UPDATE_ERR_IMAGE        8 // esp_ota_end() didn't take the image
#define SKYPUFF_UPDATE_ERR_SET_BOOT     9
#define SKYPUFF_UPDATE_ERR_COMMAND      10
#define SKYPUFF_UPDATE_ERR_BIND         11 // not a valid phrase, can't store it, or the firmware has its own

typedef struct __attribute__((packed))
{
    uint8_t cmd;              // SKYPUFF_UPDATE_READY
    uint8_t proto_version;
    uint8_t src;              // SKYPUFF_FRAME_SRC_TX_MODULE or _RX_MODULE
    uint8_t pending;          // 1: running a new firmware which waits for CONFIRM
    uint8_t running_slot;     // 0: app0, 1: app1
    uint32_t build_id;
    uint16_t chunk_len;
    char version[12];         // VERSIONONLYSTR
    char bind_phrase[6];      // the module's, all 0 if it has none, see skypuff_bind.h
} tSkypuffUpdateReady;


#endif // SKYPUFF_FRAME_H
