#include <stdint.h>
#include "LEDHelper.h"
#include "core_pins.h"

#define STAT_DIN 2
#define STAT_CLK 12
#define STAT_LOAD 11

uint8_t statusLED = 0;

void setRGBLED(int mode) {
	statusLED = (statusLED & ~7) | (1 << mode);
}

void setLED(int lednum, int level) {
	if (level) {
		statusLED |= (1 << (lednum + 2));
	}
	else {
		statusLED &= ~(1 << (lednum + 2));
	}
}

void updateLEDs() {
	//Set the initial pin state
	digitalWriteFast(STAT_CLK, 0);
	digitalWriteFast(STAT_LOAD, 0);

	//Update all the pins based off the status state
	for (int i = 0; i < 8; i++) {
		digitalWriteFast(STAT_DIN, (1 & (statusLED >> i)));
		digitalWriteFast(STAT_CLK, 1);
		delayMicroseconds(1);
		digitalWriteFast(STAT_CLK, 0);
		delayMicroseconds(1);
	}

	//Load the register
	digitalWriteFast(STAT_LOAD, 1);
	delayMicroseconds(1);

	//Set the final pin state
	digitalWriteFast(STAT_LOAD, 0);
	digitalWriteFast(STAT_DIN, 0);
}