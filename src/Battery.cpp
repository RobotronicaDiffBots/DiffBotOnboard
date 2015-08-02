#include "Battery.h"

#define BAT_CRIT 39600
#define BAT_LOW 42600

//Returns 1 if the battery is critical
uint8_t checkBatteryCritical() {
	return (analogRead(A11) < BAT_CRIT);
}

//Returns 1 if the battery is low
uint8_t checkBatteryLow() {
	return (analogRead(A11) < BAT_LOW);
}