/**
 *
 * HX711 library for Arduino
 * https://github.com/bogde/HX711
 *
 * MIT License
 * (c) 2018 Bogdan Necula
 *
**/

#include "hx711.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <applibs/gpio.h>

#include <time.h>

typedef uint8_t byte;

#define LSBFIRST (1)
#define MSBFIRST (2)

#define LOW (false)
#define HIGH (true)

#define MAKE8(var, offset) ((unsigned int) var >> (offset * 8)) & 0x00FF
#define MAKE16(varhigh, varlow) (((unsigned int) varhigh & 0xFF) * 0x100) + ((unsigned int) varhigh & 0xFF)

void digitalWrite(int gpioFd, bool value) {
	//printf("Digital write pin=%d value=%d.\n", gpioFd, value);
	GPIO_SetValue(gpioFd, (GPIO_Value)value);
}

bool digitalRead(int gpioFd) {
	GPIO_Value result = 0;
	GPIO_GetValue(gpioFd, &result);
	//printf("Digital read pin=%d value=%d.\n", gpioFd, result);
	return result != 0;
}

void delayMicroseconds(long micros) {	
	if (micros == 0) {
		return;
	}
	const struct timespec sleepTime = { 0, 1000 * micros };	
	if (nanosleep(&sleepTime, NULL) != 0) {
		printf("Sleep fail");
	};
}

void delay(long millis) {
	if (millis == 0) {
		return;
	}
	long seconds = millis / 1000;
	millis = millis % 1000;
	const struct timespec sleepTime = { seconds, 1000000 * millis };
	if (nanosleep(&sleepTime, NULL) != 0) {
		printf("Sleep fail");
	};
}

const long CLOCKS_PER_MSEC = CLOCKS_PER_SEC / 1000;

long millis() {
	clock_t time = clock();
	return time / CLOCKS_PER_MSEC;
}

void noInterrupts() {
	// TODO
}

void interrupts() {
	// TODO
}

// TEENSYDUINO has a port of Dean Camera's ATOMIC_BLOCK macros for AVR to ARM Cortex M3.
//#define HAS_ATOMIC_BLOCK (defined(ARDUINO_ARCH_AVR) || defined(TEENSYDUINO))
//#define HAS_ATOMIC_BLOCK

// Whether we are running on either the ESP8266 or the ESP32.
#define ARCH_ESPRESSIF (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))
//#define ARCH_ESPRESSIF

// Whether we are actually running on FreeRTOS.
#define IS_FREE_RTOS defined(ARDUINO_ARCH_ESP32)

// Define macro designating whether we're running on a reasonable
// fast CPU and so should slow down sampling from GPIO.
#define FAST_CPU \
    ( \
    ARCH_ESPRESSIF || \
    defined(ARDUINO_ARCH_SAM)     || defined(ARDUINO_ARCH_SAMD) || \
    defined(ARDUINO_ARCH_STM32)   || defined(TEENSYDUINO) \
    )

#ifdef HAS_ATOMIC_BLOCK
// Acquire AVR-specific ATOMIC_BLOCK(ATOMIC_RESTORESTATE) macro.
#include <util/atomic.h>
#endif

uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
	uint8_t value = 0;
	uint8_t i;
	for (i = 0; i < 8; ++i) {
		digitalWrite(clockPin, HIGH);
		if (bitOrder == LSBFIRST)
			value |= digitalRead(dataPin) << i;
		else
			value |= digitalRead(dataPin) << (7 - i);
		digitalWrite(clockPin, LOW);
	}
	return value;
}

#undef FAST_CPU
#ifdef FAST_CPU
// Make shiftIn() be aware of clockspeed for
// faster CPUs like ESP32, Teensy 3.x and friends.
// See also:
// - https://github.com/bogde/HX711/issues/75
// - https://github.com/arduino/Arduino/issues/6561
// - https://community.hiveeyes.org/t/using-bogdans-canonical-hx711-library-on-the-esp32/539
uint8_t shiftInSlow(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
	uint8_t value = 0;
	uint8_t i;

	for (i = 0; i < 8; ++i) {
		digitalWrite(clockPin, HIGH);
		//delay(1);
		delayMicroseconds(1);
		if (bitOrder == LSBFIRST)
			value |= digitalRead(dataPin) << i;
		else
			value |= digitalRead(dataPin) << (7 - i);
		digitalWrite(clockPin, LOW);
		delayMicroseconds(1);
		//delay(1);
	}
	return value;
}
#define SHIFTIN_WITH_SPEED_SUPPORT(data,clock,order) shiftInSlow(data,clock,order)
#else
#define SHIFTIN_WITH_SPEED_SUPPORT(data,clock,order) shiftIn(data,clock,order)
#endif

byte hx711_pd_sck;	// Power Down and Serial Clock Input Pin
byte hx711_dout;// Serial Data Output Pin
byte hx711_gain;// amplification factor
long hx711_offset = 0;	// used for tare weight
float hx711_scale = 1;	// used to return weight in grams, kg, ounces, whatever

void hx711_begin(byte dout, byte pd_sck, byte gain) {
	hx711_pd_sck = pd_sck;
	hx711_dout = dout;

	hx711_set_gain(gain);
}

bool hx711_is_ready() {
	if (digitalRead(hx711_dout) == LOW) {
		delayMicroseconds(50);
		if (digitalRead(hx711_dout) == LOW) {
			return true;
		}
	}
	return false;
}

void hx711_set_gain(byte gain) {
	switch (gain) {
	case 128:		// channel A, gain factor 128
		hx711_gain = 1;
		break;
	case 64:		// channel A, gain factor 64
		hx711_gain = 3;
		break;
	case 32:		// channel B, gain factor 32
		hx711_gain = 2;
		break;
	}

	digitalWrite(hx711_pd_sck, LOW);
	hx711_read();
}

long hx711_read() {

	// Wait for the chip to become ready.
	hx711_wait_ready(0);

	// Define structures for reading data into.
	unsigned long value = 0;
	uint8_t data[3] = { 0 };
	uint8_t filler = 0x00;

	// Protect the read sequence from system interrupts.  If an interrupt occurs during
	// the time the hx711_pd_sck signal is high it will stretch the length of the clock pulse.
	// If the total pulse time exceeds 60 uSec this will cause the HX711 to enter
	// power down mode during the middle of the read sequence.  While the device will
	// wake up when hx711_pd_sck goes low again, the reset starts a new conversion cycle which
	// forces hx711_dout high until that cycle is completed.
	//
	// The result is that all subsequent bits read by shiftIn() will read back as 1,
	// corrupting the value returned by read().  The ATOMIC_BLOCK macro disables
	// interrupts during the sequence and then restores the interrupt mask to its previous
	// state after the sequence completes, insuring that the entire read-and-gain-set
	// sequence is not interrupted.  The macro has a few minor advantages over bracketing
	// the sequence between `noInterrupts()` and `interrupts()` calls.
#if HAS_ATOMIC_BLOCK
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

#elif IS_FREE_RTOS
// Begin of critical section.
// Critical sections are used as a valid protection method
// against simultaneous access in vanilla FreeRTOS.
// Disable the scheduler and call portDISABLE_INTERRUPTS. This prevents
// context switches and servicing of ISRs during a critical section.
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
	portENTER_CRITICAL(&mux);

#else
// Disable interrupts.
	noInterrupts();
#endif

	// Pulse the clock pin 24 times to read the data.
	data[2] = SHIFTIN_WITH_SPEED_SUPPORT(hx711_dout, hx711_pd_sck, MSBFIRST);
	data[1] = SHIFTIN_WITH_SPEED_SUPPORT(hx711_dout, hx711_pd_sck, MSBFIRST);
	data[0] = SHIFTIN_WITH_SPEED_SUPPORT(hx711_dout, hx711_pd_sck, MSBFIRST);

	// Set the channel and the gain factor for the next reading using the clock pin.
	for (unsigned int i = 0; i < hx711_gain; i++) {
		digitalWrite(hx711_pd_sck, HIGH);
#ifdef ARCH_ESPRESSIF
		delayMicroseconds(1);
#endif
		digitalWrite(hx711_pd_sck, LOW);
#ifdef ARCH_ESPRESSIF
		delayMicroseconds(1);
#endif
	}

#if IS_FREE_RTOS
	// End of critical section.
	portEXIT_CRITICAL(&mux);

#elif HAS_ATOMIC_BLOCK
	}

#else
	// Enable interrupts again.
	interrupts();
#endif

	// Replicate the most significant bit to pad out a 32-bit signed integer
	if (data[2] & 0x80) {
		filler = 0xFF;
	}
	else {
		filler = 0x00;
	}

	// Construct a 32-bit signed integer
	value = ((unsigned long) (filler)) << 24
		| ((unsigned long) (data[2])) << 16
		| ((unsigned long) (data[1])) << 8
		| ((unsigned long) (data[0]));

	return (long) (value);
}

void hx711_wait_ready(unsigned long delay_ms) {
	// Wait for the chip to become ready.
	// This is a blocking implementation and will
	// halt the sketch until a load cell is connected.
	while (!hx711_is_ready()) {
		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
		// https://github.com/bogde/HX711/issues/73
		delay(delay_ms);
	}
}

bool hx711_wait_ready_retry(int retries, unsigned long delay_ms) {
	// Wait for the chip to become ready by
	// retrying for a specified amount of attempts.
	// https://github.com/bogde/HX711/issues/76
	int count = 0;
	while (count < retries) {
		if (hx711_is_ready()) {
			return true;
		}
		delay(delay_ms);
		count++;
	}
	return false;
}

bool hx711_wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {
	// Wait for the chip to become ready until timeout.
	// https://github.com/bogde/HX711/pull/96
	unsigned long millisStarted = millis();
	while (millis() - millisStarted < timeout) {
		if (hx711_is_ready()) {
			return true;
		}
		delay(delay_ms);
	}
	return false;
}

long hx711_read_average(byte times) {
	long sum = 0;
	for (byte i = 0; i < times; i++) {
		sum += hx711_read();
		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
		// https://github.com/bogde/HX711/issues/73
		delay(0);
	}
	return sum / times;
}

double hx711_get_value(byte times) {
	return hx711_read_average(times) - hx711_offset;
}

float hx711_get_units(byte times) {
	return hx711_get_value(times) / hx711_scale;
}

void hx711_tare(byte times) {
	double sum = hx711_read_average(times);
	hx711_set_offset(sum);
}

void hx711_set_scale(float scale) {
	hx711_scale = scale;
}

float hx711_get_scale() {
	return hx711_scale;
}

void hx711_set_offset(long offset) {
	hx711_offset = offset;
}

long hx711_get_offset() {
	return hx711_offset;
}

void hx711_power_down() {
	digitalWrite(hx711_pd_sck, LOW);
	digitalWrite(hx711_pd_sck, HIGH);
}

void hx711_power_up() {
	digitalWrite(hx711_pd_sck, LOW);
}