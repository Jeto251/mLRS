//*******************************************************
// Skypuff bind phrase store
//*******************************************************
// The module keeps its bind phrase in its NVS, apart from the firmware, so an update
// keeps it. The firmware has none built in, one firmware fits all modules. The
// updater reports the phrase and sets it on request, from the board's screen or
// from the PC.
//
// A module from before this store (firmwares before 3.11.2026) has none here, but its
// firmware had the phrase built in and wrote it to the mLRS setup at every start
// (SETUP_FORCE_COMMON_CONF). The firmware takes it from there once.
//
// SKYPUFF_BIND_MARKER in the image tells the board and the update script that the
// firmware keeps the module's phrase. Customer firmwares until 13.11.2026 had their
// phrase there instead of "keep" and wrote it to the module at every start.
//*******************************************************
#ifndef SKYPUFF_BIND_H
#define SKYPUFF_BIND_H
#pragma once

#include <inttypes.h>
#include <string.h>


#define SKYPUFF_BIND_LEN    6

#define SKYPUFF_BIND_MARKER "SKYPUFF-BIND-PHRASE keep"


// Same rule as in setup.h, which would quietly replace other characters
inline bool skypuff_bind_valid(const char* s)
{
    if (strlen(s) != SKYPUFF_BIND_LEN) return false;
    for (uint8_t i = 0; i < SKYPUFF_BIND_LEN; i++) {
        char c = s[i];
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '#' || c == '.')) return false;
    }
    return true;
}


#ifdef ESP32

#include <stddef.h>
#include <stdlib.h>
#include <nvs.h>
#include "setup_types.h"

const char skypuff_bind_marker[] = SKYPUFF_BIND_MARKER;

// The module's phrase, "" if it has none
char skypuff_bind_phrase[SKYPUFF_BIND_LEN + 1] = "";


// The mLRS setup is in the Arduino EEPROM emulation, which is the blob "eeprom" in the
// NVS namespace "eeprom": two copies of 1 KB (esp-eeprom.h), each a status word, up to
// 16 bytes, then tSetup. Common[] is at the same place since mLRS v1.4, the oldest
// firmware of the modules; the asserts catch a change of the layout.
#define SKYPUFF_EE_PAGE_SIZE   0x400
#define SKYPUFF_EE_DATA_OFS    16
#define SKYPUFF_EE_VALID_PAGE  0x11111111
#define SKYPUFF_EE_PHRASE_OFS  (SKYPUFF_EE_DATA_OFS + offsetof(tSetup, Common) + offsetof(tCommonSetup, BindPhrase))

static_assert(offsetof(tSetup, MarkerStr) == 0, "the setup layout has changed, check the phrase of old modules");
static_assert(offsetof(tSetup, Common) == 274, "the setup layout has changed, check the phrase of old modules");
static_assert(SKYPUFF_EE_PHRASE_OFS + SKYPUFF_BIND_LEN <= SKYPUFF_EE_PAGE_SIZE, "the phrase is out of the page");

// The phrase in the stored mLRS setup, "" if there is none or it's the mLRS default
void skypuff_bind_from_mlrs_setup(char* s)
{
    s[0] = '\0';
    nvs_handle_t h;
    if (nvs_open("eeprom", NVS_READONLY, &h) != ESP_OK) return;
    uint8_t* blob = NULL;
    size_t len = 0;
    if (nvs_get_blob(h, "eeprom", NULL, &len) == ESP_OK && len >= 2 * SKYPUFF_EE_PAGE_SIZE) {
        blob = (uint8_t*)malloc(len);
        if (blob != NULL && nvs_get_blob(h, "eeprom", blob, &len) != ESP_OK) { free(blob); blob = NULL; }
    }
    nvs_close(h);
    if (blob == NULL) return;

    // the second copy, if a write of the first one was cut
    for (uint8_t page = 0; page < 2 && s[0] == '\0'; page++) {
        const uint8_t* p = blob + page * SKYPUFF_EE_PAGE_SIZE;
        uint32_t status;
        memcpy(&status, p, sizeof(status));
        if (status != SKYPUFF_EE_VALID_PAGE) continue;
        const char* marker = (const char*)(p + SKYPUFF_EE_DATA_OFS + offsetof(tSetup, MarkerStr));
        if (strncmp(marker, SETUP_MARKER_TX_STR, 16) != 0 && strncmp(marker, SETUP_MARKER_RX_STR, 16) != 0 &&
            strncmp(marker, SETUP_MARKER_OLD_STR, 16) != 0) continue;
        char phrase[SKYPUFF_BIND_LEN + 1];
        memcpy(phrase, p + SKYPUFF_EE_PHRASE_OFS, SKYPUFF_BIND_LEN);
        phrase[SKYPUFF_BIND_LEN] = '\0';
        if (skypuff_bind_valid(phrase) && strcmp(phrase, "mlrs.0") != 0) strcpy(s, phrase);
    }
    free(blob);
}


// Before the updater and setup_init()
void skypuff_bind_load(void)
{
    // nothing reads the marker, this keeps the linker from dropping it
    __asm__ volatile("" : : "r"(skypuff_bind_marker));

    char s[SKYPUFF_BIND_LEN + 1] = "";
    nvs_handle_t h;
    if (nvs_open("skypuff", NVS_READWRITE, &h) == ESP_OK) {
#ifdef SKYPUFF_BIND_TEST_FORGET
        nvs_erase_key(h, "bind"); // test build: as a module from before this store
        nvs_commit(h);
#endif
        size_t len = sizeof(s);
        if (nvs_get_str(h, "bind", s, &len) != ESP_OK || !skypuff_bind_valid(s)) s[0] = '\0';
        if (s[0] == '\0') {
            skypuff_bind_from_mlrs_setup(s);
            if (s[0] != '\0') {
                nvs_set_str(h, "bind", s);
                nvs_commit(h);
            }
        }
        nvs_close(h);
    }
    strcpy(skypuff_bind_phrase, s);
}

// False if the phrase isn't valid or couldn't be stored
bool skypuff_bind_store(const char* s)
{
    if (!skypuff_bind_valid(s)) return false;
    nvs_handle_t h;
    if (nvs_open("skypuff", NVS_READWRITE, &h) != ESP_OK) return false;
    bool ok = (nvs_set_str(h, "bind", s) == ESP_OK && nvs_commit(h) == ESP_OK);
    nvs_close(h);
    if (ok) strcpy(skypuff_bind_phrase, s);
    return ok;
}

// For setup_default(): the module's phrase, BIND_PHRASE if it has none
#define SKYPUFF_BIND_PHRASE()  ((skypuff_bind_phrase[0] != '\0') ? skypuff_bind_phrase : BIND_PHRASE)

#else

inline void skypuff_bind_load(void) {}
#define SKYPUFF_BIND_PHRASE()  BIND_PHRASE

#endif


#endif // SKYPUFF_BIND_H
