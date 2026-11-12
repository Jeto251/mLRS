//*******************************************************
// Skypuff temperature calibration
//*******************************************************
// The temperature sensor of the ESP32 isn't calibrated, chips differ by tens of
// degrees at the same temperature. The update script calibrates it at one point:
// it sends the actual temperature, the module keeps the offset to its own reading
// in its NVS, apart from the firmware, and adds it to every reading. Best done on
// a cold module, the chip is at the air temperature then.
//*******************************************************
#ifndef SKYPUFF_TEMP_H
#define SKYPUFF_TEMP_H
#pragma once

#include <inttypes.h>
#include <math.h>


#define SKYPUFF_TEMP_OFFSET_MAX_C10   600 // a larger offset is taken as a mistake
#define SKYPUFF_TEMP_RESET            INT16_MIN // as the actual temperature: back to no calibration


#ifdef ESP32

#include <nvs.h>

// Added to the readings, in 0.1 C
int16_t skypuff_temp_offset_c10 = 0;

// Before the updater and the telemetry
void skypuff_temp_load(void)
{
    nvs_handle_t h;
    if (nvs_open("skypuff", NVS_READONLY, &h) != ESP_OK) return; // no calibration yet
    int16_t v;
    if (nvs_get_i16(h, "temp_c10", &v) == ESP_OK && v >= -SKYPUFF_TEMP_OFFSET_MAX_C10 && v <= SKYPUFF_TEMP_OFFSET_MAX_C10) {
        skypuff_temp_offset_c10 = v;
    }
    nvs_close(h);
}

// The sensor itself, in 0.1 C. Averaged, a single reading steps by 0.56 C and is noisy.
int16_t skypuff_temp_raw_c10(void)
{
    float sum = 0.0f;
    for (uint8_t i = 0; i < 8; i++) sum += temperatureRead();
    return (int16_t)lroundf(sum / 8 * 10);
}

// Calibrated, in C
float skypuff_temp_read(void)
{
    return temperatureRead() + skypuff_temp_offset_c10 / 10.0f;
}

// actual_c10 is the temperature the chip is at now, SKYPUFF_TEMP_RESET drops the
// calibration. False if the offset is out of range or can't be stored.
bool skypuff_temp_calibrate(int16_t actual_c10, int16_t* raw_c10)
{
    int16_t raw = skypuff_temp_raw_c10();
    if (raw_c10) *raw_c10 = raw;
    int32_t offset = (actual_c10 == SKYPUFF_TEMP_RESET) ? 0 : (int32_t)actual_c10 - raw;
    if (offset < -SKYPUFF_TEMP_OFFSET_MAX_C10 || offset > SKYPUFF_TEMP_OFFSET_MAX_C10) return false;

    nvs_handle_t h;
    if (nvs_open("skypuff", NVS_READWRITE, &h) != ESP_OK) return false;
    bool ok = (nvs_set_i16(h, "temp_c10", (int16_t)offset) == ESP_OK && nvs_commit(h) == ESP_OK);
    nvs_close(h);
    if (ok) skypuff_temp_offset_c10 = (int16_t)offset;
    return ok;
}

#else

inline void skypuff_temp_load(void) {}

#endif


#endif // SKYPUFF_TEMP_H
