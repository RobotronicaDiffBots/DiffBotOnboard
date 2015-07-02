
#include "MotorControl.h"
#include "LSM303.h"
#include "IntervalTimer.h"
#include "LEDHelper.h"
#include "Networking.h"

//TODO: ADD MOTOR PIN DEFINES

bool gtoflag = false;
bool motflag = false;
float gto[] = { 0, 0 };
float mot[] = { 0, 0 };
float loc[] = { 0, 0 };
float estloc[] = { 0, 0 };
float heading = 0; //in degrees, 0 is down stage, -ve is stage left, +ve right   
float estHeading = 0;
float compassHeading = 0;

long lenc = 0;
long renc = 0;

uint8_t motorOut[] = { 0, 0 };
bool dir[] = { true, true };

float initHeading = 0;

Encoder lEncoder(LENC1, LENC2);
Encoder rEncoder(RENC1, RENC2);

LSM303 compass;

elapsedMillis encTimer;

void readEncoders()	{
	
	if (encTimer > ERT_MS) {
		
		encTimer = 0;

		//Read the encoders and reset them asap
		long lenc = lEncoder.read();
		long renc = rEncoder.read();
		lEncoder.write(0);
		rEncoder.write(0);	
	}
}

void estimateLocation() {

	if (lenc == 0 && renc == 0) {
		return;	//we haven't moved, don't bother
	}

	//Find the velocities.
	float lvel = VEL_CONST * lenc;
	float rvel = VEL_CONST * renc;

	//Special cases:
	//lvel == rvel, straight line
	if (lvel == rvel) {
		float theta = -(estHeading + 90);
		float dist = lenc * DIST_CONST;
		estloc[0] += cos(theta) * dist;
		estloc[1] += sin(theta) * dist;
	}
	//lvel == -rvel, rotating on spot
	else if (lvel == -rvel) {
		float omega = (rvel - lvel) / WHEELBASE;
		estHeading -= omega * ENC_READ_TIME;
		normaliseEstHeading();
	}
	//Normal case:
	//just the normal circle arc case
	else {
		float theta = -(estHeading + 90);
		//Using info from a Berkley paper
		float R = (WHEELBASE / 2) * ((lvel + rvel) / (rvel - lvel));
		float ICC[] = { estloc[0] - R*cos(theta), estloc[1] - R*sin(theta) };
		float omega = (rvel - lvel) / WHEELBASE;
		float oconst = omega * ENC_READ_TIME;
		estloc[0] = cos(oconst) * (estloc[0] - ICC[0]) - sin(oconst) * (estloc[1] - ICC[1]) + ICC[0];
		estloc[1] = sin(oconst) * (estloc[0] - ICC[0]) + cos(oconst) * (estloc[1] - ICC[1]) + ICC[1];
		estHeading -= oconst;
		normaliseEstHeading();
	}
}

void normaliseHeading() {
	if (heading < -180) {
		while (heading < -180) {
			heading += 360;
		}
	}
	else if (heading > 180) {
		while (heading > 180) {
			heading -= 360;
		}
	}
}

void setupCompass() {
	compass.init();
	compass.enableDefault();
	//Defualt values, may need to change later.
	LSM303::vector<int16_t> mmin = { -32767, -32767, -32767 };
	compass.m_min = mmin;
	LSM303::vector<int16_t> mmax = { +32767, +32767, +32767 };
	compass.m_max = mmax;
	compass.read();
	initHeading = compass.heading();
}

void checkCompass() {
	compass.read();
	//Plus instead of minus because reversed compass direction.
	//May need changing depending on how compass works
	estHeading = compass.heading() + initHeading;
	normaliseCompassHeading();
}

void normaliseCompassHeading() {
	if (compassHeading < -180) {
		while (compassHeading < -180) {
			compassHeading += 360;
		}
	}
	else if (compassHeading > 180) {
		while (compassHeading > 180) {
			compassHeading -= 360;
		}
	}
}

void normaliseEstHeading() {
	if (estHeading < -180) {
		while (estHeading < -180) {
			estHeading += 360;
		}
	}
	else if (estHeading > 180) {
		while (estHeading > 180) {
			estHeading -= 360;
		}
	}
}

void setupMotors() {
	pinMode(M1D1, OUTPUT);
	pinMode(M2D1, OUTPUT);
	pinMode(M1IN1, OUTPUT);
	pinMode(M1IN2, OUTPUT);
	pinMode(M2IN1, OUTPUT);
	pinMode(M2IN2, OUTPUT);
	analogWriteFrequency(M1D1, 20000);
	analogWriteFrequency(M2D1, 20000);

}

void updateMotors() {
	setLED(MTR_MODE, motflag);
	if (motflag) {
		//The magnitudes
		analogWrite(M1D1, (uint8_t) (255 - abs(mot[0]) * 255));
		analogWrite(M2D1, (uint8_t) (255 - abs(mot[1]) * 255));
		//The directions
		dir[0] = !signbit(mot[0]);
		dir[1] = !signbit(mot[1]);
		digitalWriteFast(M1IN1, (dir[0]));
		digitalWriteFast(M1IN2, (!dir[0]));
		digitalWriteFast(M2IN1, (dir[1]));
		digitalWriteFast(M2IN2, (!dir[1]));
		
		motflag = false;
	}
	
}