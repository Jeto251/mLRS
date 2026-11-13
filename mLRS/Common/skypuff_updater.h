//*******************************************************
// Skypuff module updater
//*******************************************************
// Updates the module firmware over its serial port, from the ESP board it sits on.
// No extra wires: the module listens for a ping during the first
// SKYPUFF_UPDATER_WINDOW_MS of the boot, before the radio starts. Without a ping it
// boots on as usual. The board can't switch the module off and on, so it sends the
// running firmware a RESTART frame instead, which restarts the module into a window
// of SKYPUFF_UPDATER_REQUESTED_WINDOW_MS.
//
// The new firmware goes to the other app partition. Started for the first time it
// is pending: it waits SKYPUFF_UPDATER_CONFIRM_WINDOW_MS for the board to ping it and
// confirm it. That proves the new firmware can be updated again. Unconfirmed, or if
// it crashes before, the bootloader goes back to the previous firmware.
//
// Frames: skypuff_frame.h, type SKYPUFF_FRAME_TYPE_UPDATE. The board checks a firmware
// file for SKYPUFF_UPDATER_MARKER before sending it, a firmware without this updater
// couldn't be updated over the serial port again.
//*******************************************************
#ifndef SKYPUFF_UPDATER_H
#define SKYPUFF_UPDATER_H
#pragma once

#include "skypuff_frame.h"
#include "skypuff_crash.h"
#include "skypuff_bind.h"
#include "skypuff_temp.h"
#include "skypuff_telemetry.h"


#if defined ESP32 && defined SKYPUFF_UPDATER_WINDOW_MS

#include <esp_ota_ops.h>

// For the board's check of the firmware file, see above. Carries the role too, a tx
// firmware must not end up on a rx module and vice versa.
#ifdef DEVICE_IS_TRANSMITTER
#define SKYPUFF_UPDATER_MARKER          "SKYPUFF-MODULE-UPDATER-1 TX"
#else
#define SKYPUFF_UPDATER_MARKER          "SKYPUFF-MODULE-UPDATER-1 RX"
#endif
const char skypuff_updater_marker[] = SKYPUFF_UPDATER_MARKER;

// Arduino confirms a new firmware by itself at the start, unless asked not to
extern "C" bool verifyRollbackLater(void) { return true; }

#define SKYPUFF_UPDATER_REQUEST_MAGIC   0x5C7A0D17

// Set before the restart the board asked for, survives it
RTC_NOINIT_ATTR uint32_t skypuff_updater_request;

// Every byte the board sends passes here, the dropped ones too, see tUartBPort in
// common.h. A RESTART frame from the board restarts the module into a long update
// window, an INFO_REQUEST has the module info sent again. Only a frame with a one
// byte payload has this encoded length, COBS adds a byte to the frame.
static uint8_t skypuff_sniff_buf[SKYPUFF_FRAME_HEADER_LEN + 1 + SKYPUFF_FRAME_CRC_LEN + 1];
static uint8_t skypuff_sniff_len = 0;

void skypuff_updater_sniff(char c)
{
    if (c != 0) {
        if (skypuff_sniff_len < sizeof(skypuff_sniff_buf)) skypuff_sniff_buf[skypuff_sniff_len] = c;
        if (skypuff_sniff_len < UINT8_MAX) skypuff_sniff_len++;
        return;
    }

    uint8_t n = skypuff_sniff_len;
    skypuff_sniff_len = 0;
    if (n != sizeof(skypuff_sniff_buf)) return;

    uint8_t frame[sizeof(skypuff_sniff_buf)];
    uint16_t frame_len = skypuff_cobs_decode(skypuff_sniff_buf, n, frame);
    uint8_t type, src;
    const uint8_t* payload;
    uint16_t len;
    if (!skypuff_frame_parse(frame, frame_len, &type, &src, &payload, &len)) return;
    if (type != SKYPUFF_FRAME_TYPE_UPDATE || src != SKYPUFF_FRAME_SRC_ESP || len != 1) return;

    if (payload[0] == SKYPUFF_UPDATE_INFO_REQUEST) {
        skypuff_module_info_requested = true;
        return;
    }
    if (payload[0] != SKYPUFF_UPDATE_RESTART) return;

    skypuff_updater_request = SKYPUFF_UPDATER_REQUEST_MAGIC;
    esp_restart();
}


class tSkypuffUpdater
{
  public:
    void Boot(uint8_t _src);

  private:
    bool receive(uint32_t timeout_ms, const uint8_t** payload, uint16_t* len);
    void send(const void* payload, uint16_t len);
    void send_ready(void);
    void send_ack(uint32_t offset);
    void send_error(uint8_t code, esp_err_t err);
    bool session(void); // false: the board went silent

    void abort_update(void);

    uint8_t src;
    uint8_t seq;
    bool pending;
    bool confirmed;

    esp_ota_handle_t ota_handle;
    const esp_partition_t* ota_partition;
    uint32_t image_size;
    uint32_t image_crc;
    uint32_t next_offset;
    uint32_t crc;

    // decoded frame: header + cmd + offset + chunk + crc
    static const uint16_t FRAME_MAX = SKYPUFF_FRAME_HEADER_LEN + 1 + 4 + SKYPUFF_UPDATE_CHUNK_LEN + SKYPUFF_FRAME_CRC_LEN;
    uint8_t rx_buf[FRAME_MAX + FRAME_MAX / 254 + 2];
    uint16_t rx_len;
    uint8_t frame[FRAME_MAX];
    uint8_t tx_frame[64];
    uint8_t tx_buf[64 + 64 / 254 + 2];
};


bool tSkypuffUpdater::receive(uint32_t timeout_ms, const uint8_t** payload, uint16_t* len)
{
    uint32_t t0_ms = millis();
    while (millis() - t0_ms < timeout_ms) {
        int c = UARTB_SERIAL_NO.read();
        if (c < 0) { delay(1); continue; }

        if (c != 0) {
            if (rx_len < sizeof(rx_buf)) rx_buf[rx_len] = c;
            rx_len++; // an oversized frame keeps counting and gets dropped at its end
            continue;
        }

        uint16_t n = rx_len;
        rx_len = 0;
        if (n == 0 || n > sizeof(rx_buf)) continue;

        uint16_t frame_len = skypuff_cobs_decode(rx_buf, n, frame);
        uint8_t type, frame_src;
        // anything else is the board's normal traffic or noise
        if (!skypuff_frame_parse(frame, frame_len, &type, &frame_src, payload, len)) continue;
        if (type != SKYPUFF_FRAME_TYPE_UPDATE || *len == 0) continue;
        return true;
    }
    return false;
}


void tSkypuffUpdater::send(const void* payload, uint16_t len)
{
    uint16_t frame_len = skypuff_frame_build(SKYPUFF_FRAME_TYPE_UPDATE, src, seq++, payload, len, tx_frame);
    uint16_t n = skypuff_cobs_encode(tx_frame, frame_len, tx_buf);
    UARTB_SERIAL_NO.write(tx_buf, n);
}


void tSkypuffUpdater::send_ready(void)
{
    tSkypuffUpdateReady ready = {};
    ready.cmd = SKYPUFF_UPDATE_READY;
    ready.proto_version = SKYPUFF_UPDATE_PROTO_VERSION;
    ready.src = src;
    ready.pending = pending ? 1 : 0;
    const esp_partition_t* running = esp_ota_get_running_partition();
    ready.running_slot = (running && running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1) ? 1 : 0;
    ready.build_id = skypuff_build_id();
    ready.chunk_len = SKYPUFF_UPDATE_CHUNK_LEN;
    strncpy(ready.version, VERSIONONLYSTR, sizeof(ready.version));
    memcpy(ready.bind_phrase, skypuff_bind_phrase, strlen(skypuff_bind_phrase));
    ready.temp_offset_c10 = skypuff_temp_offset_c10;
    ready.temp_c10 = skypuff_temp_raw_c10() + skypuff_temp_offset_c10;
    send(&ready, sizeof(ready));
}


void tSkypuffUpdater::send_ack(uint32_t offset)
{
    uint8_t p[5] = { SKYPUFF_UPDATE_ACK };
    memcpy(&p[1], &offset, 4);
    send(p, sizeof(p));
}


void tSkypuffUpdater::send_error(uint8_t code, esp_err_t err)
{
    uint8_t p[6] = { SKYPUFF_UPDATE_ERROR, code };
    int32_t e = err;
    memcpy(&p[2], &e, 4);
    send(p, sizeof(p));
}


void tSkypuffUpdater::abort_update(void)
{
    if (ota_partition) esp_ota_abort(ota_handle);
    ota_partition = nullptr;
}


bool tSkypuffUpdater::session(void)
{
    const uint8_t* p;
    uint16_t len;

    send_ready();

    while (true) {
        if (!receive(SKYPUFF_UPDATER_IDLE_MS, &p, &len)) {
            abort_update();
            return false;
        }

        switch (p[0]) {
        case SKYPUFF_UPDATE_PING: // the board may ping on until it gets READY
            send_ready();
            break;

        case SKYPUFF_UPDATE_RESTART: // late ones of the board, it restarted already
            break;

        case SKYPUFF_UPDATE_SET_TEMP: {
            int16_t actual, raw;
            if (len != 3) { send_error(SKYPUFF_UPDATE_ERR_COMMAND, ESP_OK); break; }
            memcpy(&actual, &p[1], 2);
            if (!skypuff_temp_calibrate(actual, &raw)) { send_error(SKYPUFF_UPDATE_ERR_TEMP, ESP_OK); break; }
            uint8_t r[5] = { SKYPUFF_UPDATE_TEMP_SET };
            memcpy(&r[1], &skypuff_temp_offset_c10, 2);
            memcpy(&r[3], &raw, 2);
            send(r, sizeof(r));
            break; }

        case SKYPUFF_UPDATE_SET_BIND: {
            char phrase[SKYPUFF_BIND_LEN + 1] = "";
            if (len == 1 + SKYPUFF_BIND_LEN) memcpy(phrase, &p[1], SKYPUFF_BIND_LEN);
            if (!skypuff_bind_store(phrase)) { send_error(SKYPUFF_UPDATE_ERR_BIND, ESP_OK); break; }
            uint8_t c = SKYPUFF_UPDATE_BIND_SET;
            send(&c, 1);
            break; }

        case SKYPUFF_UPDATE_CONFIRM:
            if (pending) {
                esp_ota_mark_app_valid_cancel_rollback();
                pending = false;
            }
            confirmed = true;
            {
                uint8_t c = SKYPUFF_UPDATE_CONFIRMED;
                send(&c, 1);
            }
            return true;

        case SKYPUFF_UPDATE_ABORT:
            abort_update();
            return true;

        case SKYPUFF_UPDATE_BEGIN: {
            abort_update();
            if (pending) { send_error(SKYPUFF_UPDATE_ERR_PENDING, ESP_OK); break; }
            if (len < 9) { send_error(SKYPUFF_UPDATE_ERR_COMMAND, ESP_OK); break; }
            memcpy(&image_size, &p[1], 4);
            memcpy(&image_crc, &p[5], 4);
            const esp_partition_t* part = esp_ota_get_next_update_partition(nullptr);
            if (!part || image_size == 0 || image_size > part->size) { send_error(SKYPUFF_UPDATE_ERR_TOO_BIG, ESP_OK); break; }
            // erases as the writes go, so there is no long wait here
            esp_err_t err = esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
            if (err != ESP_OK) { send_error(SKYPUFF_UPDATE_ERR_BEGIN, err); break; }
            ota_partition = part;
            next_offset = 0;
            crc = 0;
            send_ack(0);
            break; }

        case SKYPUFF_UPDATE_DATA: {
            if (!ota_partition) { send_error(SKYPUFF_UPDATE_ERR_NOT_STARTED, ESP_OK); break; }
            if (len < 5) { send_error(SKYPUFF_UPDATE_ERR_COMMAND, ESP_OK); break; }
            uint32_t offset;
            memcpy(&offset, &p[1], 4);
            uint16_t n = len - 5;
            // a repeated or lost chunk: the ack tells the board where to go on
            if (offset != next_offset || n == 0 || next_offset + n > image_size) { send_ack(next_offset); break; }
            esp_err_t err = esp_ota_write(ota_handle, &p[5], n);
            if (err != ESP_OK) { abort_update(); send_error(SKYPUFF_UPDATE_ERR_WRITE, err); break; }
            crc = skypuff_crc32(crc, &p[5], n);
            next_offset += n;
            send_ack(next_offset);
            break; }

        case SKYPUFF_UPDATE_END: {
            if (!ota_partition) { send_error(SKYPUFF_UPDATE_ERR_NOT_STARTED, ESP_OK); break; }
            if (next_offset != image_size) { send_error(SKYPUFF_UPDATE_ERR_SIZE, ESP_OK); break; }
            if (crc != image_crc) { abort_update(); send_error(SKYPUFF_UPDATE_ERR_CRC, ESP_OK); break; }
            esp_err_t err = esp_ota_end(ota_handle); // checks the image
            const esp_partition_t* part = ota_partition;
            ota_partition = nullptr;
            if (err != ESP_OK) { send_error(SKYPUFF_UPDATE_ERR_IMAGE, err); break; }
            err = esp_ota_set_boot_partition(part);
            if (err != ESP_OK) { send_error(SKYPUFF_UPDATE_ERR_SET_BOOT, err); break; }
            uint8_t c = SKYPUFF_UPDATE_DONE;
            send(&c, 1);
            UARTB_SERIAL_NO.flush();
            delay(20);
            esp_restart(); // into the new firmware, which then waits for CONFIRM
            break; }

        default:
            send_error(SKYPUFF_UPDATE_ERR_COMMAND, ESP_OK);
            break;
        }
    }
}


void tSkypuffUpdater::Boot(uint8_t _src)
{
    // nothing reads the marker, this keeps the linker from dropping it
    __asm__ volatile("" : : "r"(skypuff_updater_marker));

    src = _src;
    seq = 0;
    rx_len = 0;
    ota_partition = nullptr;

    esp_ota_img_states_t state;
    const esp_partition_t* running = esp_ota_get_running_partition();
    pending = (esp_ota_get_state_partition(running, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY);
    confirmed = !pending;

    // mLRS sets the buffer sizes before its begin(), so this ends the port again below
    UARTB_SERIAL_NO.setRxBufferSize(SKYPUFF_UPDATER_RXBUF);
#if defined UARTB_USE_TX_IO && defined UARTB_USE_RX_IO
    UARTB_SERIAL_NO.begin(SKYPUFF_UPDATER_BAUD, SERIAL_8N1, UARTB_USE_RX_IO, UARTB_USE_TX_IO);
#else
    UARTB_SERIAL_NO.begin(SKYPUFF_UPDATER_BAUD, SERIAL_8N1);
#endif

    const uint8_t* p;
    uint16_t len;
    uint32_t window_ms = pending ? SKYPUFF_UPDATER_CONFIRM_WINDOW_MS : SKYPUFF_UPDATER_WINDOW_MS;
    if (skypuff_updater_request == SKYPUFF_UPDATER_REQUEST_MAGIC && window_ms < SKYPUFF_UPDATER_REQUESTED_WINDOW_MS) {
        window_ms = SKYPUFF_UPDATER_REQUESTED_WINDOW_MS;
    }
    skypuff_updater_request = 0;
#ifdef SKYPUFF_UPDATER_TEST_NO_CONFIRM
    // Test build only: a new firmware which can't be confirmed, checks the rollback
    if (pending) window_ms = 0;
#endif
    uint32_t t0_ms = millis();
    while (millis() - t0_ms < window_ms) {
        if (!receive(window_ms - (millis() - t0_ms), &p, &len)) break;
        if (p[0] != SKYPUFF_UPDATE_PING) continue;
        session();
        break;
    }

    if (!confirmed) {
        // The board didn't confirm this new firmware, it may not be able to update it again
        esp_ota_mark_app_invalid_rollback_and_reboot();
    }

    UARTB_SERIAL_NO.flush(); // end() drops what is not sent yet, like the CONFIRMED reply
    UARTB_SERIAL_NO.end();
}

tSkypuffUpdater skypuff_updater;

#define SKYPUFF_UPDATER_BOOT(src)  skypuff_updater.Boot(src)

#else

#define SKYPUFF_UPDATER_BOOT(src)

#endif


#endif // SKYPUFF_UPDATER_H
