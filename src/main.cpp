#include "Networking.h"
#include "Encoder.h"
#include "MotorControl.h"
#include "LEDHelper.h"
#include "Battery.h"

#include <Math.h>


void setup() {
	setupNetworking();
	setupLEDs();
	updateLEDs();
	//setupCompass();		//requires actual compass connected.
	setupMotors();
}

void mainloop() {
	//Read the serials
	readSerial();
	resetLEDs();

	//Check the timeout
	if (timeoutCheck()) {
		stop();
		setLED(NO_MSGS, 1);
		setRGBLED(BAD);
	}
	else {
		setLED(NO_MSGS, 0);
	}

	//Update physical inputs
	//checkCompass();	//Won't work without compass connected
	readEncoders();

	/*
	//Check battery, finally, before moving
	if (!checkBattery()) {
		stop();
		setLED(LOW_BATT, 1);
	}
	else {
		setLED(LOW_BATT, 0);
	}
	*/

	//Estimate location
	estimateLocation();

	//Update outputs
	updateMotors();
	updateLEDs();	
}

extern "C" int main() {
	setup();
	while (1) {
		mainloop();
	}
}