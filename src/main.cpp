#include "Networking.h"
#include "Control.h"
#include "util.h"
#include "LEDHelper.h"

#define DT_CONTROL_MS       100             // Main sample loop timer (ms)


serialReader xbReader = serialReader(&xbSerial);
serialReader btReader = serialReader(&btSerial);

uint32_t previousMillis;

void setup() {
	setupLEDs();

	xbSerial.begin(xbBaud);
	btSerial.begin(btBaud);

	setupMotors();
	setIdle();
}

void mainloop() {

	if (xbReader.checkRadio()) {
		processPacket(xbReader.radioMessage, &xbSerial);
	}
	if (btReader.checkRadio()) {
		processPacket(btReader.radioMessage, &btSerial);
	}

	
	//If either of these are being flooded, set an output LED
	if (btReader.flood() || xbReader.flood()) {
		//setLEDMode(WARNING);
	}


	if (previousMillis - millis() > DT_CONTROL_MS) {
		previousMillis = millis();
		//Check when we last got a valid packet
		uint32_t lastValid = max(btReader.lastValid(), xbReader.lastValid());
		if ((previousMillis - lastValid) >= 2000) {
			//setLEDMode(ERROR);
			setIdle();
		}
		/* This is where all the main action/tasks take place. DO NOT use while loops or delays */
		updateLoopOnce();
	}
}

extern "C" int main() {
	setup();
	while (1) {
		mainloop();
	}
}