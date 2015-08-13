#include "Networking.h"
#include "Control.h"
#include "util.h"
#include "LEDHelper.h"
#include "Battery.h"


//The two serial objects
serialReader xbReader = serialReader(&xbSerial);
serialReader btReader = serialReader(&btSerial);

uint32_t previousMillis;

void setup() {
	//Set analog read res
	analogReadResolution(16);

	//Enable the LED outptus
	setupLEDs();

	//begin the three serial ports
	xbSerial.begin(xbBaud);
	btSerial.begin(btBaud);
	topSerial.begin(topBaud);

	//Set up motor outputs (but don't turn them on)
	setupMotors();
	//setIdle();
	updateLoopOnce();
}

void mainloop() {
	//Check the two radios, process the packets recieved from them
	//(and update a status output
	if (xbReader.checkRadio()) {
		processPacket(xbReader.radioMessage, &xbSerial);
		setRGBLED(GOOD);
		setLED(NO_MSGS, 0);
	}
	if (btReader.checkRadio()) {
		processPacket(btReader.radioMessage, &btSerial);
		setRGBLED(GOOD);
		setLED(NO_MSGS, 0);
	}

	//If either of these are being flooded, set an output LED
	if (btReader.flood() || xbReader.flood()) {
		setRGBLED(EH);
	}

	//If the battery is low, set some output LEDs
	if (checkBatteryLow()) {
		setRGBLED(EH);
		setLED(LOW_BATT, 1);
	}

	//Execute the main loop if it's been enough time
	if ((previousMillis - millis()) > DT_CONTROL_MS) {
		previousMillis = millis();
		//Check when we last got a valid packet 
		//If too long, stop the robot and set a status output
		uint32_t lastValid = max(btReader.lastValid(), xbReader.lastValid());
		if ((previousMillis - lastValid) >= 800) {
			setLED(NO_MSGS, 1);
			setRGBLED(BAD);
			setIdle();
		}
		/* This is where all the main action/tasks take place. DO NOT use while loops or delays */
		updateLoopOnce();
		updateLEDs();
	}

}

extern "C" int main() {
	setup();
	while (!checkBatteryCritical()) {
		mainloop();
	}
	setLED(NO_BATT, 1);
	setRGBLED(BAD);
	setIdle();
	updateLoopOnce();
	updateLEDs();
	
}