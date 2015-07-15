#include <stdint.h>
#include "LEDHelper.h"
#include "core_pins.h"

#define STAT_DIN 2
#define STAT_CLK 12
#define STAT_LOAD 11

uint8_t statusLED = 0;

//Sets the RGB LED to one of 3 states. 
void setRGBLED(uint8_t mode) {
	statusLED = (statusLED & ~0xE0) | (1 << mode);
}

//Enables or disables one of the LEDs
void setLED(uint8_t lednum, uint8_t level) {
	if (level) {
		statusLED |= (1 << (lednum));
	}
	else {
		statusLED &= ~(1 << (lednum));
	}
}

//Sets up the outputs for the Shift Register
void setupLEDs() {
	//Set Pin Modes
	pinMode(STAT_CLK, OUTPUT);
	pinMode(STAT_DIN, OUTPUT);
	pinMode(STAT_LOAD, OUTPUT);
	updateLEDs();
	//Do a small animation, to ensure they're working
	for (int i = 0; i < 5; i++) {
		setLED(i, 1);
		setRGBLED(BAD);
		updateLEDs();
		delay(100);
		setRGBLED(EH);
		updateLEDs();
		delay(100);
		setLED(i, 0);
		setRGBLED(GOOD);
		updateLEDs();
		delay(100);
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

//Rests everything to a base state (status led green, everything else off)
void resetLEDs() {
	statusLED = 0;
	setRGBLED(GOOD);
}