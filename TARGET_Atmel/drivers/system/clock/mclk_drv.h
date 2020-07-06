#ifndef SYSTEM_CLOCK_MCLK_H_INCLUDED
#define SYSTEM_CLOCK_MCLK_H_INCLUDED

#include <cmsis.h>
#include <compiler.h>
#include <status_codes.h>
#include "clock_feature.h"

void system_ahb_clock_set_mask(
		const uint32_t ahb_mask);
void system_ahb_clock_clear_mask(
		const uint32_t ahb_mask);
enum status_code system_apb_clock_set_mask(
		const enum system_clock_apb_bus bus,
		const uint32_t mask);
enum status_code system_apb_clock_clear_mask(
		const enum system_clock_apb_bus bus,
		const uint32_t mask);

#endif
