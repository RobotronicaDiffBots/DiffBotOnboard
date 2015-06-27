#include "Networking.h"
#include "Encoder.h"
#include "MotorControl.h"
#include "LEDHelper.h"
#include "Battery.h"

#include <Math.h>


void setup() {
	setupNetworking();
	setupCompass();
	setupMotors();
}

void loop() {
	//Read the serials
	readSerial();
	setRGBLED(GOOD);

	//Check the timeout
	if (timeoutCheck()) {
		stop();
		setLED(NO_MSGS, 1);
	}
	else {
		setLED(NO_MSGS, 0);
	}


	//Update physical inputs
	checkCompass();
	readEncoders();

	//Check battery, finally, before moving
	if (!checkBattery()) {
		stop();
		setLED(LOW_BATT, 1);
	}
	else {

		setLED(LOW_BATT, 0);
	}

	//Update outputs
	updateMotors();
	updateLEDs();
}

extern "C" int main() {
	setup();
	while (1)
		loop();
}