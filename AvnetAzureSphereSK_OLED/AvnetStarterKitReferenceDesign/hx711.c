/** 
 * Based on:
 * https://www.ccsinfo.com/forum/viewtopic.php?p=195617#195617
 */
#include "hx711.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <applibs/gpio.h>

#include <time.h>

static int hx711_do_fd;
static int hx711_clk_fd;

#define HX711_DO  hx711_do_fd
#define HX711_CLK hx711_clk_fd

#define MAKE8(var, offset) ((unsigned int) var >> (offset * 8)) & 0x00FF
#define MAKE16(varhigh, varlow) (((unsigned int) varhigh & 0xFF) * 0x100) + ((unsigned int) varhigh & 0xFF)

void output_bit(int gpioFd, bool value) {
	GPIO_SetValue(gpioFd, (GPIO_Value) value);
}

bool input(int gpioFd) {
	GPIO_Value result = 0;
	GPIO_GetValue(gpioFd, &result);
	return result != 0;
}

void hx711_init(int do_fd, int clk_fd) {
	hx711_do_fd = do_fd;
	hx711_clk_fd = clk_fd;
}

void sleepC() {
	const struct timespec sleepTime = { 0, 10000000 };
	nanosleep(&sleepTime, NULL);
}

void hx711_main()
{
	bool test = false;
	uint32_t Count, BUFFER[30], offseeet = 0, mied = 0;
	uint8_t i = 0;

	while (true) {

		while (i < 10) {
			Count = hx711_measurement();
			BUFFER[i] = Count;
			mied += BUFFER[i];
			i++;
		}
		i = 0;
		mied /= 11;

		if (test == 0) {
			offseeet = mied;
			test = 1;
		}

		//Count=measurement();
		mied -= offseeet;
		//printf("data=%lu \n\r",Count);
		printf("mied=%lu \n\r", mied);
		//delay_ms(1);
		//printf("\f");
	}
}
//********************************************
int32_t hx711_measurement(void) {
	uint32_t Count;
	uint8_t i, A_1, A_2, A_3;

	//output_bit(HX711_DO, 1);
	output_bit(HX711_CLK, 0);
	sleepC();
	Count = 0;

	while (input(HX711_DO));

	for (i = 0; i < 24; i++) {// gain 128
		output_bit(HX711_CLK, 1);
		sleepC();
		Count = Count << 1;
		output_bit(HX711_CLK, 0);
		sleepC();
		if (input(HX711_DO)) Count++;
	}

	output_bit(HX711_CLK, 1);
	sleepC();
	Count = Count ^ 0x800000;
	output_bit(HX711_CLK, 0);
	sleepC();

	//************************
	A_1 = MAKE8(Count, 0);
	A_2 = MAKE8(Count, 1);
	A_3 = MAKE8(Count, 2);
	A_2 = (A_2 & 0b11111000);
	Count = MAKE16(A_3, A_2);
	return(Count);
}

