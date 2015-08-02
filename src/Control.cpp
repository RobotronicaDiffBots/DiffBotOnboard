#include "Networking.h"
#include "Control.h"
#include "core_pins.h"
#include "LEDHelper.h"
#include "Encoder.h"
#include "Math.h"
#include "util.h"
//Pins
#define M1D1 21
#define M1IN1 22
#define M1IN2 23

#define M2D1 5
#define M2IN1 15
#define M2IN2 6

//Physical constants
#define WB 0.285							//Wheelbase length
#define WD 0.1
#define DELTAT 0.04
#define IDELTAT 25

#define VCONST PI * 0.1 * 25 /1920.0		//25 = 1/DELTAT

#define MAX_TICKS_PER_SEC 60				//May need tweaking

enum {
	TASK_IDLE = 0,	//Robot stops (0 params)
	TASK_MANUAL,	//Controls the motors directly (2 params, lmotor and rmotor (0 to 200, 100 is idle))
	TASK_SPIN,		//Spins on the spot (4 params, direction (0 or 1), speed (0 to 100), time (ms, split into two parts))
	TASK_LED,		//Sets the LED status of the top LEDs. (4 params, R, G, B, ledno (0, 1, 255))
	TASK_STOP,
	TASK_DRIVE,
	TASK_BRAKE = 250,	//Stops. (0 params)
	PING = 255		//A 'keep alive' type message, just to avoid timeout (0 params)
};

radio_message_t latestMessage;

uint8_t mode;

uint8_t ldem = 100;
uint8_t rdem = 100;

uint16_t counter;

//Location stuff
Encoder lenc = Encoder(17, 16);
Encoder renc = Encoder(4, 3);
long lcount;
long rcount;
uint8_t loccount = 0;
float pose[3] = { 0 };

//BT stuff
char output[50];

//PID
int16_t e_t;
int16_t e_tl;
int16_t e_tr;

//Spin stuff
int16_t demandedLoc;
uint8_t sdem;
float sk_p = 0.5;
float sk_i = 0.7;
float sk_d = 0.9;

//Motor stuff
int8_t dl;
int8_t dr;
float mk_p = 1;
float mk_i = 1;
float mk_d = 1;



/**
* Process the valid incoming radio packet based on task and set the overall robot mode.
* Note that the mapping between radio sent modes may be different to onboard robot modes.
*/
void processPacket(radio_message_t radioMessage, Stream* stream) {
	if (VERBOSE) {
		stream->write("Valid message received\r\n");
	}

	//Pass the packet on to the top
	uint8_t *p = (uint8_t *)&radioMessage;
	for (uint8_t i = 0; i < 10; i++) {
		topSerial.write(p[i]);
	}

	//Cache the message that we got. 
	latestMessage = radioMessage;
	switch (latestMessage.type) {
	case TASK_MANUAL:
		mode = TASK_MANUAL;
		dl = (latestMessage.d1 - 100) * 0.01 * MAX_TICKS_PER_SEC;	//changes (-100, 100) to (-MAX, MAX)
		dr = (latestMessage.d2 - 100) * 0.01 * MAX_TICKS_PER_SEC;	//				"
		e_tl = dl - lcount;
		e_tr = dr - rcount;
		break;
	case TASK_SPIN:
		mode = TASK_SPIN;
		demandedLoc = pose[2] * 1000 + (int16_t)((latestMessage.d1 << 8) | (latestMessage.d2 & 0xFF));
		sdem = latestMessage.d3;
		e_t = demandedLoc - pose[2] * 1000;
		break;
	case TASK_DRIVE:
		mode = TASK_DRIVE;

		break;
	default:
		break;
	}
}

/**
* Execute one time step of the action/task state machine. Do not use while loops or delays here.
* Trying to keep the system semi-concurrent.
*/
void updateLoopOnce() {
	//First, update the motor outputs based on the mode
	switch (mode) {
		case TASK_IDLE:
			ldem = 100;
			rdem = 100;
			break;

		case TASK_MANUAL:
		{
			int16_t e_tl1 = dl - lcount;
			int16_t e_tr1 = dr - rcount;

			int16_t ldiff = e_tl1 - e_tl;
			int16_t rdiff = e_tr1 - e_tr;

			int8_t lvel = clamp(-100, 100, e_tl1 * mk_p + ldiff * DELTAT * mk_i + ldiff * IDELTAT * mk_d);
			int8_t rvel = clamp(-100, 100, e_tl1 * mk_p + rdiff * DELTAT * mk_i + rdiff * IDELTAT * mk_d);

			e_tl = e_tl1;
			e_tr = e_tr1;
			break;
		}
		case TASK_SPIN:
		{
			int16_t mr = pose[2] * 1000;
			int16_t e_t1 = demandedLoc - mr;

			int16_t diff = (e_t1 - e_t);
			int8_t vel = clamp(-100, 100, e_t1 * sk_p + diff * DELTAT * sk_i + diff * IDELTAT * sk_d) * (sdem * 0.01);
			ldem = 100 + vel;
			rdem = 100 + -vel;

			e_t = e_t1;
			break;
		}
		case TASK_DRIVE:

		default: //Any commands we don't care about, like TASK_LED or PNG			
			break;
	}
	if (loccount % 4 == 0)
		calculateLocation();
	
	loccount++;

	//Decrement our timer counter (each count is 10ms)
	if (counter > 0) {
		counter--;
		if (!counter) {
			ldem = 100;
			rdem = 100;
			mode = TASK_IDLE;
		}
	}
		
	setMotors(ldem, rdem);
}


// Basic setup function, sets all the correct pin modes and the
// output frequency of the PWM. This freq should not exceed 20kHz
void setupMotors() {
	pinMode(M1D1, OUTPUT);
	pinMode(M2D1, OUTPUT);
	pinMode(M1IN1, OUTPUT);
	pinMode(M1IN2, OUTPUT);
	pinMode(M2IN1, OUTPUT);
	pinMode(M2IN2, OUTPUT);
	analogWriteFrequency(M1D1, 20000);
	analogWriteFrequency(M2D1, 20000);

	lenc.write(0);
	renc.write(0);
}

// Sets the actual motor outputs based upon the demands passed
// Expects values between 0 and 200 for each motor, 100 is float
void setMotors(uint8_t ldem, uint8_t rdem) {
	//Check bounds
	if (ldem > 200 || rdem > 200) {
		return;	//bad values, nothing to do
	}

	//The magnitudes
	int lmag = abs(ldem-100) * 2.55;
	int rmag = abs(rdem-100) * 2.55;


	//Set the pwm outputs
	analogWrite(M1D1, 255 - lmag);
	analogWrite(M2D1, 255 - rmag);

	//The directions
	if (1) {//mode != TASK_BRAKE) {
		digitalWriteFast(M1IN1, ldem < 100);
		digitalWriteFast(M1IN2, !(ldem < 100));
		digitalWriteFast(M2IN1, rdem < 100);
		digitalWriteFast(M2IN2, !(rdem < 100));
	}
	else {
		digitalWriteFast(M1IN1, HIGH);
		digitalWriteFast(M1IN2, HIGH);
		digitalWriteFast(M2IN1, HIGH);
		digitalWriteFast(M2IN2, HIGH);
	}
}

void calculateLocation() {
	//Next, let's see how far we've come
	lcount = lenc.read();
	rcount = renc.read();

	float lvel = lcount * VCONST;
	float rvel = -1* rcount * VCONST;
	

	lenc.write(0);
	renc.write(0);

	float omega = (rvel - lvel) / WB;

	if (omega == 0) {
		//Straight line
		float cosv = cos(pose[2]);
		float sinv = sin(pose[2]);
		pose[0] += cosv * lvel;
		pose[1] += sinv * lvel;
	}
	else {
		//Curvelinear
		float R = (WB * 0.5) * ((lvel + rvel) / (rvel - lvel));
		float ICC[] = { pose[0] - R * sin(pose[2]), pose[1] + R * cos(pose[2]) };

		float cost = cos(omega*DELTAT);
		float sint = sin(omega*DELTAT);

		float newpose[3];
		newpose[0] = cost * (pose[0] - ICC[0]) - sint * (pose[1] - ICC[1]) + ICC[0];
		newpose[1] = sint * (pose[0] - ICC[0]) + cost * (pose[1] - ICC[1]) + ICC[1];
		newpose[2] = pose[2] + omega*DELTAT;

		pose[0] = newpose[0];
		pose[1] = newpose[1];
		pose[2] = newpose[2];

	}
	if (loccount % 100 == 0) {
		/*
		output[0] = '\0';
		snprintf(output, 50, "%d", (int)pose[0]);
		btSerial.print("x: ");
		btSerial.println(output);
		output[0] = '\0';
		snprintf(output, 50, "%d", (int)pose[1]);
		btSerial.print("y: ");
		btSerial.println(output);
		*/
		output[0] = '\0';
		snprintf(output, 50, "%d", (int)(pose[2] * 1000));
		btSerial.print("millirads: ");
		btSerial.println(output);

	}
	
}