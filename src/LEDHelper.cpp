#include <stdint.h>
#include "LEDHelper.h"



extern uint8_t statusLED;

void setRGBLED(int mode) {
	statusLED = statusLED & ~7 | (1 << mode);
}

void setLED(int lednum, int level) {
	if (level) {
		statusLED |= (1 << (lednum + 2));
	}
	else {
		statusLED &= ~(1 << (lednum + 2));
	}
}