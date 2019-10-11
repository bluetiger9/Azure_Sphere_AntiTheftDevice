#pragma once

#include <stdint.h>

void hx711_main();

void hx711_init(int do_fd, int clk_fd);

int32_t hx711_measurement(void);

