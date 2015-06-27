#include "Battery.h"
#include "core_pins.h"
#include <stdint.h>

#define BATT_SENSE A11

//A voltage divider reduces the battery voltage to a factor of 0.245
//Suitable battery voltage ranges from 12.6V to 9.5V
//Therefore max voltage will be 3.09V and min will be 2.33V
//On a 16-Bit ADC these numbers correspond to values of 61365 and 46272
//To allow for slight overcharging we set the max to 61464(3.095V)
//Subtract min from max to get 15192

bool checkBattery() {

	uint16_t rawLevel;
	rawLevel = analogRead(BATT_SENSE);
	//Need some math to compensate for battery curve, derive empirically
	int adjLevel = 100 * ((rawLevel - 46272) / 15192);
	return adjLevel < 0;
}