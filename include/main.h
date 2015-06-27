#ifndef MAIN_H

#include <stdlib.h>
#include <string.h>

#define xbSerial Serial1
#define btSerial Serial2
#define topSerial Serial3

#define CODESTART 2
#define UIDSTART 6
#define CONTENTSTART 13

#define UIDLEN 8
#define ACKLENGTH 21

char robotID[] = "AA";

int mode = 0;
int brake = 0;
bool gtoflag = false;
bool motflag = false;
bool bterr = false;
float gto[] = { 0, 0 };
float mot[] = { 0, 0 };
float loc[] = { 0, 0 };
float heading = 0; //in degrees, 0 is down stage, -ve is stage left, +ve right   

void btRespond(char *uid) {
	//Assemble NMEA string (known length)
	char *msg = (char *)malloc(ACKLENGTH);
	strcat(msg, "$");
	strcat(msg, robotID);

	if (bterr) {
		strcat(msg, "ERR");
	}
	else {
		strcat(msg, "ACK");
	}

	strcat(msg, ",");
	strcat(msg, uid);
	strcat(msg, "\r\n");

	bterr = false;
}

void setMotorDemands(char *msg) {	
	//First, extract the demands
	char* mdem[2];

	//Check that the msg actually contains two substrings
	if (!(mdem[0] = strtok(msg, ",")) || !(mdem[1] = strtok(NULL, ","))) {
		bterr = true;
		return;
	}
	
	float motor_demands[2];
	//Check that the substrings are floats
	for (int i = 0; i < 2; i++) {
		//First, check that the first thing is actually an int
		//atof returns 0 for both 0.0 and an invalid, so not a good test
		if (mdem[i][0] != '-' && (mdem[i][0] < '0' || mdem[i][0] > '9')) {
			bterr = true;
			return;
		}
		motor_demands[i] = atof(mdem[i]);
		//bounds check
		if (motor_demands[i] < -1 || motor_demands[i] > 1) {
			bterr = true;
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
		bterr = true;
		return;
	}

	float coords[2];
	//Check that the substrings are floats
	for (int i = 0; i < 2; i++) {
		//First, check that the first thing is actually an int
		//atof returns 0 for both 0.0 and an invalid, so not a good test
		if (gtoloc[i][0] != '-' && (gtoloc[i][0] < '0' ||gtoloc[i][0] > '9')) {
			bterr = true;
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
		bterr = true;
		return;
	}

	float coords[2];
	float dir;
	//Check that the substrings are floats
	for (int i = 0; i < 2; i++) {
		//First, check that the first thing is actually an int
		//atof returns 0 for both 0.0 and an invalid, so not a good test
		if (location[i][0] != '-' && (location[i][0] < '0' || location[i][0] > '9')) {
			bterr = true;
			return;
		}
		coords[i] = atof(location[i]);
	}
	if (direction[0] != '-' && (direction[0] < '0' || direction[0] > '9')) {
		bterr = true;
		return;
	}
	dir = atof(direction);

	//bounds check
	if (dir < -180 || dir >= 180) {
		bterr = true;
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

#define MAIN_H
#endif