#include "main.h"
#include "core_pins.h"
#include "usb_serial.h"
#include "HardwareSerial.h"
#include "NMEASerial.h"


void btNMEACallback(char *msg) {
	//Confirm it's a NMEA string (5 chars then ',')
	for (int i = 0; i < 5; i++) {
		//Should be all caps first field
		char c = msg[i];
		if (c == '\0' || c < 'A' || c > 'Z') {
			return;	//Nothing we can do
		}
	}

	//Confirm that the UID exists, extract it, and store it
	char uid[UIDLEN + 1];
	for (int i = 0; i < UIDLEN; i++) {
		char c = msg[UIDSTART + i];
		if (c == '\0' || c < '0' || c > '9') {
			return;
		}
		else {
			uid[i] = c;
		}
	}
	uid[UIDLEN] = '\0';

	//Check the message type (check substr(2, 5), call appropriate funcn)
	//Messy if chain, nothing for it

	//Ping (should just ACK back)
	if (strncmp(msg + CODESTART, "PNG", 3) == 0) {
		btRespond(uid);	
	}
	//Motor commands (most commonly used in rc controller
	else if (strncmp(msg + CODESTART, "MTR", 3) == 0) {
		setMotorDemands(msg + CONTENTSTART);
		btRespond(uid);
	}
	//Go to (should drive to a point)
	else if (strncmp(msg + CODESTART, "GTO", 3) == 0) {
		setTargetLocation(msg + CONTENTSTART);
		btRespond(uid);
	}
	//Brake the motors
	else if (strncmp(msg + CODESTART, "STP", 3) == 0) {
		stop();
		btRespond(uid);
	}
	//tell the robot where it is and where it is facing
	else if (strncmp(msg + CODESTART, "LOC", 3) == 0) {
		setLocation(msg + CONTENTSTART);
		btRespond(uid);
	}
	//The code wasn't recognised, but at least let the controller know we got the msg
	else {
		bterr = true;
		btRespond(uid);
	}
}

void updateMotors() {
}
void updateLEDs() {
}




NMEAReader btReader(&btSerial, btNMEACallback);

void setup() {
	btSerial.begin(115200);
}

void loop() {
	btReader.read();
	updateMotors();
	updateLEDs();
}

extern "C" int main() {
	setup();
	while (1)
		loop();
}