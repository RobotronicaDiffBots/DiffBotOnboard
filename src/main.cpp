#include "core_pins.h"
#include "usb_serial.h"
#include "NMEASerial.h"
#include "NMEAParse.h"
#include "Encoder.h"

#include <Math.h>

//Serial ports
#define xbSerial Serial1
#define btSerial Serial2
#define topSerial Serial3

//LED Statuses
#define BAD 0
#define EH 1
#define GOOD 2

#define NO_MSGS 1
#define LOW_BATT 2
#define NO_BATT 3
#define GTO_MODE 4
#define MTR_MODE 5

//PINS
#define LENC1 16
#define LENC2 17
#define RENC1 3
#define RENC2 4

#define STAT_DIN 2
#define STAT_CLK 12
#define STAT_LOAD 11

//Physical Dimensions (SI Units)
#define WHEEL_RAD 0.05F
#define WHEELBASE 0.3F
#define WHEELBASE_CIRC 2 * PI * WHEELBASE

#define ENCODER_CPR 1920
#define ENC_READ_TIME 0.2F 
#define ERT_MS ENC_READ_TIME * 1000

#define M_PER_REV 2 * PI * WHEEL_RAD
#define RPC 1/ENCODER_CPR
#define VEL_CONST M_PER_REV * RPC / ENC_READ_TIME
#define DIST_CONST M_PER_REV * RPC




char robotID[] = "AA";


int mode = 0;
int brake = 0;
bool gtoflag = false;
bool motflag = false;
float gto[] = { 0, 0 };
float mot[] = { 0, 0 };
float loc[] = { 0, 0 };
float estloc[] = { 0, 0 };
float heading = 0; //in degrees, 0 is down stage, -ve is stage left, +ve right   
float estHeading = 0;

uint8_t statusLED = 0;

elapsedMillis timeout;
elapsedMillis encTimer;

void updateMotors() {
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

void timeoutCheck() {
	/* Commented for manual sending of messages
	//if we haven't gotten a message in 2 seconds, probably stop
	if (timeout > 2000) {
		motflag = false;
		gtoflag = false;
		setRGBLED(BAD);
		setLED(NO_MSGS, 1);
	}
	*/
}



void checkCompass() {
}
void readEncoders()	{
	if (encTimer > ERT_MS) {
		encTimer = 0;
		long lenc = lEncoder.read();
		long renc = rEncoder.read();
		float lvel = VEL_CONST * lenc;
		float rvel = VEL_CONST * renc;

		//Special cases 
		//lvel == rvel, straight line
		if (lvel == rvel) {
			float theta = -(heading + 90);
			float dist = lenc * DIST_CONST;
			estloc[0] += cos(theta) * dist;
			estloc[1] += sin(theta) * dist;
		}
		else if (lvel == -rvel) {
			float dist = lenc * DIST_CONST;
			estHeading = dist / 
		}

	}
}
void estimateLocation() {
}

Encoder lEncoder(LENC1, LENC2);
Encoder rEncoder(RENC1, RENC2);


NMEAReader btReader(&btSerial, NMEACallback);
NMEAReader xbReader(&xbSerial, NMEACallback);

void setup() {
	btSerial.begin(115200);
	xbSerial.begin(115200);
}

void loop() {
	//Read the serials
	btReader.read();
	xbReader.read();

	//Check the timeout
	timeoutCheck();

	//Update physical inputs
	checkCompass();
	readEncoders();

	estimateLocation();

	//Update outputs
	updateMotors();
	updateLEDs();
}

extern "C" int main() {
	setup();
	while (1)
		loop();
}