//NOTE: Always call these with the defined constants in main
//Do not go off-trail, soldier!
#pragma once

//LED Statuses
#define BAD 0
#define GOOD 1
#define EH 2

#define NO_MSGS 1
#define LOW_BATT 2
#define NO_BATT 3
#define GTO_MODE 4
#define MTR_MODE 5

extern uint8_t statusLED;

void setRGBLED(int mode);

void setLED(int lednum, int level);

void updateLEDs();