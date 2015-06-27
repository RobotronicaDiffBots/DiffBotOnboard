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
NMEAReader xbReader(&xbSerial, NMEACallback);

bool msgerr = false;

void readSerial() {
	btReader.read();
	xbReader.read();
}

void setupNetworking() {
	btSerial.begin(115200);
	xbSerial.begin(115200);
}

void NMEACallback(char *msg, Stream *stream) {
	//Extract the messenger and the code
	char farg[6];
	farg[5] = 0;
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

	//Skip the ','
	msg++;

	//Confirm that the UID exists, extract it, and store it
	uint8_t uid;
	if (*msg < '0' || *msg > '9') {
		return;	//has no UID, abort!
	}
	atoi(msg);

	while (*msg != ',' || *msg != '\0') {
		msg++;
	}

	//Skip the ','
	msg++;

	//Check the message type (check substr(2, 5), call appropriate funcn)
	//Messy if chain, nothing for it

	char *code = farg + 2;
	//Ping (should just ACK back)
	if (strncmp(code, "PNG", 3) == 0) {
		Respond(uid, stream);
	}
	//Motor commands (most commonly used in rc controller)
	else if (strncmp(code, "MTR", 3) == 0) {
		setMotorDemands(msg);
		Respond(uid, stream);
	}
	//Go to (should drive to a point)
	else if (strncmp(code, "GTO", 3) == 0) {
		setTargetLocation(msg);
		Respond(uid, stream);
	}
	//Brake the motors
	else if (strncmp(code, "STP", 3) == 0) {
		stop();
		Respond(uid, stream);
	}
	//tell the robot where it is and where it is facing
	else if (strncmp(code, "LOC", 3) == 0) {
		setLocation(msg);
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
	char struid[6];	//the UID max is 65535, plus a null term
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
	strcat(msg, "\r\n");

	stream->write(msg);
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
			msgerr = true;
			return;
		}
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

void stop() {
	//If we got here, we have a legitimate demand. Do it!
	//We do not want to goto any points, so disable gto
	gtoflag = false;
	//We do, however, want to set some motor demands.
	motflag = true;

	mot[0] = 0;
	mot[1] = 0;
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
bool; timeoutCheck() {
	/* Commented for manual sending of messages
	//if we haven't gotten a message in 2 seconds, probably stop
	if (timeout > 2000) {
		return true;
	}	
	else {
		timeout = 0;
	}
	*/
	return false;
}
