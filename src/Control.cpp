#include "Networking.h"
#include "Control.h"
#include "core_pins.h"
#include "LEDHelper.h"
#include "Encoder.h"
#include "math.h"
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
#define IDELTAT (1 / DELTAT)
//#define IDELTAT 25

#define VCONST (PI * WD * IDELTAT / 480.0)

#define MAX_TICKS_PER_SEC 50				//May need tweaking

//#define COMPASS_CONSTANT 5.7433e-04
#define COMPASS_CONSTANT 5.7423e-04


enum {
    COUNT_L6 = 0,
    COUNT_L5,
    COUNT_L4,
    COUNT_LT,
    COUNT_R3,
    COUNT_R2,
    COUNT_R1,
    COUNT_RT
};

// Robot modes
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

// Robot actions
enum {
    ONE_EIGHTY_SPIN = 0,
    NINETY_SPIN,
    FORTY_FIVE_SPIN
};

#define ONE_EIGHTY_DEGREES  ((PI))
#define NINETY_DEGREES      ((PI / 2))
// Don't use FORTY_FIVE_DEGREES because it's a pain
// Has the most compounding error due to the small movement
//#define FORTY_FIVE_DEGREES  ((PI / 4))
#define ONE_HUNDRED_CM (100.f)
#define BACKWARDS_ERROR_OFFSET (3.f)
#define ONE_THOUSAND_CM (1000.f)

radio_message_t latestMessage;

uint8_t mode;
uint8_t oldMode;

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

// long lcountPers = 0;
// long rcountPers = 0;

int16_t spinCount1 = 0;
int16_t spinCount2 = 0;

float theta = 0.f;
float distance = 0.f;

//BT stuff
char output[50];

//PID
// Error at time t
int16_t e_t;
// Error at time t-1
int16_t e_t1;

int16_t e_tL;
int16_t e_tR;
int16_t e_tL1;
int16_t e_tR1;

float e_tTheta;
float e_tTheta1;

float e_tDistance;
float e_tDistance1;

//Motor stuff
float mk_p = 10;
float mk_i = 5;
float mk_d = 1;

float sk_p = 30;
float sk_i = 30;
float sk_d = 10;

float pk_p = 10;
float pk_i = 5;
float pk_d = 1;

// Need these for straight line control
// Pose has position but I don't want that
float lvel;
float rvel;
int16_t omega;

float p_errorL;
float i_errorL;
float d_errorL;

float p_errorR;
float i_errorR;
float d_errorR;

int16_t demandedVelL;
int16_t demandedVelR;

float p_errorTheta;
float i_errorTheta;
float d_errorTheta;

float p_errorDistance;
float i_errorDistance;
float d_errorDistance;

float desiredTheta = 0.f;
float desiredDistance = 0.f;

uint32_t controlTimer;
uint32_t controlTimeout;
uint32_t lastActionTime;

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
    if (mode != TASK_SPIN && mode != TASK_DRIVE) {
        switch (latestMessage.type) {
        case TASK_MANUAL:
            oldMode = mode;
            mode = TASK_MANUAL;
            
            demandedVelL = (latestMessage.d1 - 100) * 0.01 * MAX_TICKS_PER_SEC;
            demandedVelR = (latestMessage.d2 - 100) * 0.01 * MAX_TICKS_PER_SEC;
            
            break;
        case TASK_SPIN:
            /*oldMode = mode;
            mode = TASK_SPIN;
            spinCount2 = 0;
            
            // Reset theta to be in the range of -2PI <= theta <= 2PI
            theta = fmod(theta, PI);*/
            
    // 		mode = TASK_MANUAL;
    // 		demandedLoc = pose[2] * 1000 + (int16_t)((latestMessage.d1 << 8) | (latestMessage.d2 & 0xFF));
    // 		sdem = latestMessage.d3;
    // 		e_t = demandedLoc - pose[2] * 1000;
            
            break;
        case TASK_DRIVE:
            mode = TASK_DRIVE;

            break;
        default:
            break;
        }
        
        if (latestMessage.type <= 5) {
            for(int j = 0; j < 8; j++)
            {
                int k = latestMessage.d4 & (1 << j);
                if(k)
                {
                    switch(k) {
                    case (1 << COUNT_L6):
                        theta = 0.f;
                        distance = 0.f;
                        break;
                    case (1 << COUNT_L5):
                        
                        break;
                    case (1 << COUNT_L4):
                        
                        break;
                    case (1 << COUNT_R3):
                        // 45 degrees
    //                     sk_p = 35;
    //                     sk_i = 45;
    //                     sk_d = 30;
    //                     if(latestMessage.d4 & (1 << COUNT_RT))
    //                         desiredTheta = (theta - FORTY_FIVE_DEGREES);
    //                     else
    //                         desiredTheta = (theta + FORTY_FIVE_DEGREES);
                        oldMode = mode;
                        mode = TASK_DRIVE;
                        spinCount2 = 0;
                        controlTimer = millis();
                        controlTimeout = 3800;
                        
                        // Reset theta to be in the range of -2PI <= theta <= 2PI
                        distance = fmod(distance, ONE_THOUSAND_CM);
                        
                        // 100cm
                        sk_p = 60;
                        sk_i = 0.02;
                        sk_d = 100;
                        if(latestMessage.d4 & (1 << COUNT_RT))
                        {
                            // backwards
                            desiredDistance = (distance + ONE_HUNDRED_CM);
                        }
                        else
                        {
                            // forwards
                            desiredDistance = (distance - ONE_HUNDRED_CM);
                        }
                        break;
                    case (1 << COUNT_R2):
                        oldMode = mode;
                        mode = TASK_SPIN;
                        spinCount2 = 0;
                        controlTimer = millis();
                        controlTimeout = 1500;
                        
                        // Reset theta to be in the range of -2PI <= theta <= 2PI
                        theta = fmod(theta, PI);
                        
                        // 90 degrees
                        sk_p = 35;
                        sk_i = 45;
                        sk_d = 30;
                        if(latestMessage.d4 & (1 << COUNT_RT))
                        {
                            desiredTheta = (theta - NINETY_DEGREES);
                        }
                        else
                        {
                            desiredTheta = (theta + NINETY_DEGREES);
                        }
                        break;
                    case (1 << COUNT_R1):
                        oldMode = mode;
                        mode = TASK_SPIN;
                        spinCount2 = 0;
                        controlTimer = millis();
                        controlTimeout = 2000;
                        
                        // Reset theta to be in the range of -2PI <= theta <= 2PI
                        theta = fmod(theta, PI);
                        
                        // 180 degrees
                        sk_p = 30;
                        sk_i = 35;
                        sk_d = 30;
                        if(latestMessage.d4 & (1 << COUNT_RT))
                        {
                            desiredTheta = (theta - ONE_EIGHTY_DEGREES);
                        }
                        else
                        {
                            desiredTheta = (theta + ONE_EIGHTY_DEGREES);
                        }
                        break;
                    }
                    
                    break;
                }
            }
        }
    }
	// Reset error values if changing mode
    if(mode != oldMode) {
        p_errorTheta = 0.f;
        i_errorTheta = 0.f;
        d_errorTheta = 0.f;
        
        p_errorL = 0.f;
        i_errorL = 0.f;
        d_errorL = 0.f;
        
        p_errorR = 0.f;
        i_errorR = 0.f;
        d_errorR = 0.f;
        
        p_errorDistance = 0.f;
        i_errorDistance = 0.f;
        d_errorDistance = 0.f;
    }
}

/**
* Execute one time step of the action/task state machine. Do not use while loops or delays here.
* Trying to keep the system semi-concurrent.
*/

void PID()
{
    int velL = 0;
    int velR = 0;
    
    e_tL = demandedVelL - lcount;
    e_tR = demandedVelR - rcount;

    p_errorL = e_tL;
    p_errorR = e_tR;

    d_errorL = (e_tL - e_tL1) * IDELTAT;
    d_errorR = (e_tR - e_tR1) * IDELTAT;

    velL = clamp(-100, 100, p_errorL * pk_p + i_errorL * pk_i + d_errorL * pk_d);
    velR = clamp(-100, 100, p_errorR * pk_p + i_errorR * pk_i + d_errorR * pk_d);

    ldem = 100 + velL;
    rdem = 100 + velR;

    e_tL1 = e_tL;
    e_tR1 = e_tR;

    i_errorL += e_tL * DELTAT;
    i_errorR += e_tR * DELTAT;
}

void updateLoopOnce() {
	//First, update the motor outputs based on the mode
	switch (mode) {
		case TASK_IDLE:
			ldem = 100;
			rdem = 100;
			break;

		case TASK_MANUAL:
		{
            PID();
            
			break;
		}
		case TASK_SPIN:
		{
            e_tTheta = desiredTheta - theta;
            
            p_errorTheta = e_tTheta;
            
            d_errorTheta = (e_tTheta - e_tTheta1) * IDELTAT;
            
            omega = clamp(-10, 10, p_errorTheta * sk_p + i_errorTheta * sk_i + d_errorTheta * sk_d);
            demandedVelL = -omega;
            demandedVelR = omega;
            
            e_tTheta1 = e_tTheta;
            
            i_errorTheta += e_tTheta * DELTAT;
            
            PID();
            
            if (fabs(e_tTheta) < 0.05 && abs(omega) < 2 && (abs(ldem - 100) < 5) && (abs(rdem - 100) < 5))
            {
                spinCount1++;
            }
            else
            {
                spinCount1 = 0;
            }
            
            if (fabs(e_tTheta) < 0.05f && abs(omega) < 2 && (abs(ldem - 100) < 5) && (abs(rdem - 100) < 5))
            {
                spinCount2++;
            }
            
            if (spinCount2 > 20 || spinCount1 > 10 || (millis() - controlTimer) > controlTimeout)
            {
                lastActionTime = millis() - controlTimer;
                mode = TASK_MANUAL;
            }
            break;
		}
		case TASK_DRIVE:
        {
            e_tDistance = desiredDistance - distance;
            
            p_errorDistance = e_tDistance;
            
            d_errorDistance = (e_tDistance - e_tDistance1) * IDELTAT;
            
            omega = clamp(-10, 10, p_errorDistance * sk_p + i_errorDistance * sk_i + d_errorDistance * sk_d);
            demandedVelL = omega;
            demandedVelR = omega;
            
            e_tDistance1 = e_tDistance;
            
            i_errorDistance += e_tDistance * DELTAT;
            
            PID();
            
            if (fabs(e_tDistance) < 0.5f && abs(omega) < 2 && (abs(ldem - 100) < 5) && (abs(rdem - 100) < 5))
            {
                spinCount1++;
            }
            else
            {
                spinCount1 = 0;
            }
            
            if (fabs(e_tDistance) < 1.f && abs(omega) < 2 && (abs(ldem - 100) < 5) && (abs(rdem - 100) < 5))
            {
                spinCount2++;
            }
            
            if (spinCount2 > 20 || spinCount1 > 10 || (millis() - controlTimer) > controlTimeout)
            {
                lastActionTime = millis() - controlTimer;
                demandedVelL = 0;
                demandedVelR = 0;
                mode = TASK_MANUAL;
            }
            break;
        }
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
	if (!((abs(ldem - 100) < 2) && (abs(rdem - 100) < 2))) {
		digitalWriteFast(M1IN1, ldem < 100);
		digitalWriteFast(M1IN2, !(ldem < 100));
		digitalWriteFast(M2IN1, rdem < 100);
		digitalWriteFast(M2IN2, !(rdem < 100));
	}
	else {
        i_errorL = 0.f;
        i_errorR = 0.f;
        i_errorTheta = 0.f;
        i_errorDistance = 0.f;
		digitalWriteFast(M1IN1, HIGH);
		digitalWriteFast(M1IN2, HIGH);
		digitalWriteFast(M2IN1, HIGH);
		digitalWriteFast(M2IN2, HIGH);
	}
}

void calculateLocation() {
	//Next, let's see how far we've come
	lcount = -lenc.read();
	rcount = renc.read();

	lvel = lcount * VCONST;
	rvel = rcount * VCONST;

	lenc.write(0);
	renc.write(0);
    
//     lcountPers += lcount;
//     rcountPers += rcount;
    
    float deltaTheta = (rcount - lcount) * COMPASS_CONSTANT;
    float deltaDistance = (rvel + lvel) * 0.5;
    
    theta += deltaTheta;
    distance += deltaDistance;

// 	float omegaVel = (rvel - lvel) / WB;

	//if (omega == 0) {
		//Straight line
		//float cosv = cos(pose[2]);
		//float sinv = sin(pose[2]);
		//pose[0] += cosv * lvel;
		//pose[1] += sinv * lvel;
	//}
	//else {
		//Curvelinear
		//float R = (WB * 0.5) * ((lvel + rvel) / (rvel - lvel));
		//float ICC[] = { pose[0] - R * sinf(pose[2]), pose[1] + R * cosf(pose[2]) };

		//float cost = cos(omega*DELTAT);
		//float sint = sin(omega*DELTAT);

		//float newpose[3];
		//newpose[0] = cost * (pose[0] - ICC[0]) - sint * (pose[1] - ICC[1]) + ICC[0];
		//newpose[1] = sint * (pose[0] - ICC[0]) + cost * (pose[1] - ICC[1]) + ICC[1];

		//pose[0] = newpose[0];
		//pose[1] = newpose[1];
		//pose[2] = newpose[2];

	//}
    
//     pose[2] = pose[2] + omegaVel*DELTAT;

#ifdef DEBUG
	if (loccount % 100 == 0) {
        /* Debug stuff goes here */
        /*output[0] = 0;
        switch(mode)
        {
            case TASK_MANUAL:
                snprintf(output, 50, "TASK_MANUAL");
                break;
            case TASK_SPIN:
                snprintf(output, 50, "TASK_SPIN");
                break;
            case TASK_DRIVE:
                snprintf(output, 50, "TASK_DRIVE");
                break;
            default:
                snprintf(output, 50, "Unknown state");
                break;
        }
        xbSerial.print("mode: ");
        xbSerial.println(output);
        output[0] = 0;
        snprintf(output, 50, "%lu", controlTimer);
        xbSerial.print("controlTimer: ");
        xbSerial.println(output);
        output[0] = 0;
        snprintf(output, 50, "%lu", millis());
        xbSerial.print("millis(): ");
        xbSerial.println(output);*/
        output[0] = 0;
        snprintf(output, 50, "\n%lu", e_tDistance);
        xbSerial.println(output);
	}
#endif
}
