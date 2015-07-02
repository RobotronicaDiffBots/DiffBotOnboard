#include <stdlib.h>
#include <string.h>
#include "Networking.h"
#include "elapsedMillis.h"
#include "HardwareSerial.h"
#include "LEDHelper.h"


#define MSGLENGTH 32

char robotID[] = "AA";

extern bool gtoflag;
extern bool motflag;
extern float gto[];
extern float mot[];
extern float loc[];
extern float heading; //in degrees, 0 is down stage, -ve is stage left, +ve right   

elapsedMillis timeout;

NMEAReader btReader(&btSerial, NMEACallback);
//NMEAReader xbReader(&xbSerial, NMEACallback);

bool msgerr = false;
void readSerial() {
	btReader.read();
	//xbReader.read();
}

void setupNetworking() {
	btSerial.begin(115200);
	//xbSerial.begin(115200);
	timeout = 0;
}

void NMEACallback(char *msg, Stream *stream) {

	//Extract the messenger and the code
	char farg[6];
	farg[5] = '\0';

	//Confirm it's a NMEA string (5 chars then ',')
	for (int i = 0; i < 5; i++) {
		//Should be all caps first field
		char c = *msg;
		if (c == '\0' || c < 'A' || c > 'Z') {
			return;	//Nothing we can do
		}
		farg[i] = c;
		msg++;
	}

	//Set the comma to null, and then skip it
	*msg = '\0';
	msg++;

	//Confirm that the UID exists, extract it, and store it
	uint16_t uid;
	if (*msg < '0' || *msg > '9') {
		return;	//has no UID, abort!
	}
	uid = atoi(msg);

	//Skip the UID
	while (*msg != '\0' && *msg != ',') {
		msg++;
	}

	//NOTE: The initial comma stays with the message. This is for ease of use 
	//of strtok(msg, ",");
	
	//Check the message type (check substr(2, 5), call appropriate funcn)
	//Messy if chain, nothing for it

	char *code = farg + 2;
	//Ping (should just ACK back)
	if (strcmp(code, "PNG") == 0) {
		Respond(uid, stream);
	}
	//Motor commands (most commonly used in rc controller)
	else if (strcmp(code, "MTR") == 0) {
		setMotorDemands(msg);
		Respond(uid, stream);
	}
	//Go to (should drive to a point)
	else if (strcmp(code, "GTO") == 0) {
		setTargetLocation(msg);
		Respond(uid, stream);
	}
	//Brake the motors
	else if (strcmp(code, "STP") == 0) {
		stop();
		Respond(uid, stream);
	}
	//tell the robot where it is and where it is facing
	else if (strcmp(code, "LOC") == 0) {
		setLocation(msg);
		Respond(uid, stream);
	}
	//Tell the robot to go a distance
	else if (strcmp(code, "DRV") == 0) {
		setDrive(msg);
		Respond(uid, stream);
	}
	//Tell the robot to rotate 
	else if (strcmp(code, "ROT") == 0) {
		setRotate(msg);
		Respond(uid, stream);
	}
	//The code wasn't recognised, but at least let the controller know we got the msg
	else {
		setErr();
		Respond(uid, stream);
	}
	timeout = 0;
}

void Respond(int uid, Stream *stream) {
	//Assemble NMEA string (known length)
	char msg[MSGLENGTH];
	*msg = '\0';
	char struid[6];	//the UID max is 65535, plus a
	itoa(uid, struid, 10);

	strcat(msg, "$");
	strcat(msg, robotID);

	if (msgerr) {
		strcat(msg, "ERR");
	}
	else {
		strcat(msg, "ACK");
	}

	strcat(msg, ",");
	strcat(msg, struid);

	uint8_t chk = 0;
	for (uint8_t i = 1; i < strlen(msg); i++) {
		chk ^= msg[i];
	}
	strcat(msg, "*");

	char checksum[3];
	sprintf(checksum, "%x", chk);
	checksum[2] = '\0';

	strcat(msg, checksum);

	strcat(msg, "\r\n");

	stream->print(msg);
	msgerr = false;
}

void setMotorDemands(char *msg) {
	//First, extract the demands
	char* mdem[2];

	//Check that the msg actually contains two substrings
	if (!(mdem[0] = strtok(msg, ",")) || !(mdem[1] = strtok(NULL, ","))) {
		msgerr = true;
		return;
	}

	float motor_demands[2];
	//Check that the substrings are floats
	for (int i = 0; i < 2; i++) {
		//First, check that the first thing is actually an int
		//atof returns 0 for both 0.0 and an invalid, so not a good test
		if (mdem[i][0] != '-' && (mdem[i][0] < '0' || mdem[i][0] > '9')) {
			msgerr = true;
			return;
		}
		motor_demands[i] = atof(mdem[i]);
		//bounds check
		if (motor_demands[i] < -1 || motor_demands[i] > 1) {
			return;
		}
		btSerial.print("Motor: ");
		btSerial.print(motor_demands[i]);
		btSerial.write("\r\n");
	}

	//If we got here, we have a legitimate demand. Do it!
	//We do not want to goto any points, so disable gto
	gtoflag = false;
	//We do, however, want to set some motor demands.
	motflag = true;

	mot[0] = motor_demands[0];
	mot[1] = motor_demands[1];

	
}

void setTargetLocation(char *msg) {
	//First, extract the demands
	char* gtoloc[2];

	//Check that the msg actually contains two substrings
	if (!(gtoloc[0] = strtok(msg, ",")) || !(gtoloc[1] = strtok(NULL, ","))) {
		msgerr = true;
		return;
	}

	float coords[2];
	//Check that the substrings are floats
	for (int i = 0; i < 2; i++) {
		//First, check that the first thing is actually an int
		//atof returns 0 for both 0.0 and an invalid, so not a good test
		if (gtoloc[i][0] != '-' && (gtoloc[i][0] < '0' || gtoloc[i][0] > '9')) {
			msgerr = true;
			return;
		}
		coords[i] = atof(gtoloc[i]);
	}

	//If we got here, we have a legitimate xy point. Do it!
	//We do not want to set motor demands, so disable that
	motflag = false;
	//We do, however, want to go somewhere
	gtoflag = true;

	gto[0] = coords[0];
	gto[1] = coords[1];
}

void setLocation(char *msg) {
	//First, extract the demands
	char* location[2];
	char* direction;

	//Check that the msg actually contains two substrings
	if (!(location[0] = strtok(msg, ",")) || !(location[1] = strtok(NULL, ","))
		|| !(direction = strtok(NULL, ","))) {
		msgerr = true;
		return;
	}

	float coords[2];
	float dir;
	//Check that the substrings are floats
	for (int i = 0; i < 2; i++) {
		//First, check that the first thing is actually an int
		//atof returns 0 for both 0.0 and an invalid, so not a good test
		if (location[i][0] != '-' && (location[i][0] < '0' || location[i][0] > '9')) {
			msgerr = true;
			return;
		}
		coords[i] = atof(location[i]);
	}
	if (direction[0] != '-' && (direction[0] < '0' || direction[0] > '9')) {
		msgerr = true;
		return;
	}
	dir = atof(direction);

	//bounds check
	if (dir < -180 || dir >= 180) {
		msgerr = true;
		return;
	}

	loc[0] = coords[0];
	loc[1] = coords[1];
	heading = dir;
}

void setDrive() {
	//TODO
}

void setRotate() {
	//TODO
}


void setErr() {
	msgerr = true;
}

void stop() {
	motflag = true;
	mot[0] = 0;
	mot[1] = 0;
	gtoflag = false;
	setRGBLED(BAD);
}

bool timeoutCheck() {
	//if we haven't gotten a message in 2 seconds, probably stop
	return (timeout > 2000);

}
