#pragma once

#include <stdint.h>

/**
 *
 * HX711 library for Arduino
 * https://github.com/bogde/HX711
 *
 * MIT License
 * (c) 2018 Bogdan Necula
 *
**/
#ifndef HX711_h
#define HX711_h

#include <stdbool.h>

typedef uint8_t byte;
// Initialize library with data output pin, clock input pin and gain factor.
// Channel selection is made by passing the appropriate gain:
// - With a gain factor of 64 or 128, channel A is selected
// - With a gain factor of 32, channel B is selected
// The library default is "128" (Channel A).
void hx711_begin(byte dout, byte pd_sck, byte gain/* = 128*/);

// Check if HX711 is ready
// from the datasheet: When output data is not ready for retrieval, digital output pin DOUT is high. Serial clock
// input PD_SCK should be low. When DOUT goes to low, it indicates data is ready for retrieval.
bool hx711_is_ready();

// Wait for the HX711 to become ready
void hx711_wait_ready(unsigned long delay_ms/* = 0*/);
bool hx711_wait_ready_retry(int retries/* = 3*/, unsigned long delay_ms/* = 0*/);
bool hx711_wait_ready_timeout(unsigned long timeout/* = 1000*/, unsigned long delay_ms/* = 0*/);

// set the gain factor; takes effect only after a call to read()
// channel A can be set for a 128 or 64 gain; channel B has a fixed 32 gain
// depending on the parameter, the channel is also set to either A or B
void hx711_set_gain(byte gain/* = 128*/);

// waits for the chip to be ready and returns a reading
long hx711_read();

// returns an average reading; times = how many times to read
long hx711_read_average(byte times/* = 10*/);

// returns (read_average() - OFFSET), that is the current value without the tare weight; times = how many readings to do
double hx711_get_value(byte times/* = 1*/);

// returns get_value() divided by SCALE, that is the raw value divided by a value obtained via calibration
// times = how many readings to do
float hx711_get_units(byte times/* = 1*/);

// set the OFFSET value for tare weight; times = how many times to read the tare value
void hx711_tare(byte times/* = 10*/);

// set the SCALE value; this value is used to convert the raw data to "human readable" data (measure units)
void hx711_set_scale(float scale/* = 1.f*/);

// get the current SCALE
float hx711_get_scale();

// set OFFSET, the value that's subtracted from the actual reading (tare weight)
void hx711_set_offset(long offset/* = 0*/);

// get the current OFFSET
long hx711_get_offset();

// puts the chip into power down mode
void hx711_power_down();

// wakes up the chip after power down mode
void hx711_power_up();

#endif /* HX711_h */