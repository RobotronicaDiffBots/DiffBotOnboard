//NOTE: Always call these with the defined constants in main
//Do not go off-trail, soldier!
#pragma once

//LED Statuses
#define BAD 7
#define GOOD 6
#define EH 5

#define NO_MSGS 4
#define LOW_BATT 2
#define NO_BATT 3
#define GTO_MODE 1
#define MTR_MODE 0

extern uint8_t statusLED;

void setRGBLED(uint8_t mode);

void setLED(uint8_t lednum, uint8_t level);

void updateLEDs();

void setupLEDs();

void resetLEDs();