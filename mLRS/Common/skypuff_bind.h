//*******************************************************
// Skypuff bind phrase store
//*******************************************************
// The module keeps its bind phrase in its NVS, apart from the firmware, so an update
// keeps it. A customer firmware, built with the customer's BIND_PHRASE, writes it
// there at the start. The universal firmware (SKYPUFF_BIND_UNIVERSAL) is built
// without one and takes the module's. The updater reports it and sets it on request.
//
// SKYPUFF_BIND_MARKER in the image tells the update script which kind of firmware it
// sends: "SKYPUFF-BIND-PHRASE " followed by the phrase, or by "keep".
//*******************************************************
#ifndef SKYPUFF_BIND_H
#define SKYPUFF_BIND_H
#pragma once

#include <inttypes.h>
#include <string.h>


#define SKYPUFF_BIND_LEN    6

#ifdef SKYPUFF_BIND_UNIVERSAL
#define SKYPUFF_BIND_MARKER "SKYPUFF-BIND-PHRASE keep"
#else
#define SKYPUFF_BIND_MARKER "SKYPUFF-BIND-PHRASE " BIND_PHRASE
#endif


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

#include <nvs.h>

const char skypuff_bind_marker[] = SKYPUFF_BIND_MARKER;

// The module's phrase, "" if it has none
char skypuff_bind_phrase[SKYPUFF_BIND_LEN + 1] = "";

// Before the updater and setup_init()
void skypuff_bind_load(void)
{
    // nothing reads the marker, this keeps the linker from dropping it
    __asm__ volatile("" : : "r"(skypuff_bind_marker));

    char s[SKYPUFF_BIND_LEN + 1] = "";
    nvs_handle_t h;
    if (nvs_open("skypuff", NVS_READWRITE, &h) == ESP_OK) {
        size_t len = sizeof(s);
        if (nvs_get_str(h, "bind", s, &len) != ESP_OK || !skypuff_bind_valid(s)) s[0] = '\0';
#ifndef SKYPUFF_BIND_UNIVERSAL
        if (strcmp(s, BIND_PHRASE) != 0 && skypuff_bind_valid(BIND_PHRASE)) {
            strcpy(s, BIND_PHRASE);
            nvs_set_str(h, "bind", s);
            nvs_commit(h);
        }
#endif
        nvs_close(h);
    }
    strcpy(skypuff_bind_phrase, s);
}

// False if the phrase isn't valid or couldn't be stored. A customer firmware writes
// its own again at the next start.
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
