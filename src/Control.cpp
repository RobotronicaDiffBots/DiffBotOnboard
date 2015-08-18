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
#define DELTAT 0.01
#define IDELTAT (1 / DELTAT)
//#define IDELTAT 25

#define VCONST (PI * WD * IDELTAT / 480.0)

#define MAX_TICKS_PER_SEC 50				//May need tweaking

//#define COMPASS_CONSTANT 5.7433e-04
#define COMPASS_CONSTANT 5.7413e-04

enum {
    COUNT_L6 = 0, // 0x01
    COUNT_L5, // 1,  0x02
    COUNT_L4, // 2,  0x04
    COUNT_LT, // 3,  0x08
    COUNT_R3, // 4,  0x10
    COUNT_R2, // 5,  0x20
    COUNT_R1, // 6,  0x40
    COUNT_RT  // 7,  0x80
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

#define IDLE_TIMEOUT (2000)

#define UNDO_SCALING_FACTOR (1 / 0.4f)

radio_message_t latestMessage;

uint8_t mode;
uint8_t oldMode;

int16_t ldem = 100;
int16_t rdem = 100;

uint16_t counter;

//Location stuff
Encoder lenc = Encoder(17, 16);
Encoder renc = Encoder(4, 3);
long lcount;
long rcount;
uint8_t loccount = 0;
float pose[3] = { 0 };

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

float pk_p = 4.5;
float pk_i = 1.2;
float pk_d = 0.02;

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

int8_t idleFlag = 0;
uint32_t idleTimer = 0;

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
    if (mode != TASK_SPIN/* && mode != TASK_DRIVE*/) {
        switch (latestMessage.type) {
        case TASK_MANUAL:
            oldMode = mode;
            mode = TASK_MANUAL;
            
//             demandedVelL = (latestMessage.d1 - 100) * 0.01 * MAX_TICKS_PER_SEC;
//             demandedVelR = (latestMessage.d2 - 100) * 0.01 * MAX_TICKS_PER_SEC;
            // Multiply by UNDO_SCALING_FACTOR to undo scaling (to a degree) from controller
            demandedVelL = (latestMessage.d1 - 100) * UNDO_SCALING_FACTOR;
            demandedVelR = (latestMessage.d2 - 100) * UNDO_SCALING_FACTOR;
            
            break;
        case TASK_SPIN:
            oldMode = mode;
            mode = TASK_SPIN;
            spinCount2 = 0;
            
            // Reset theta to be in the range of -2PI <= theta <= 2PI
            theta = fmod(theta, PI);
            
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
//                         theta = 0.f;
//                         distance = 0.f;
                        break;
                    case (1 << COUNT_L5):
                        
                        break;
                    case (1 << COUNT_L4):
                        
                        break;
                    case (1 << COUNT_R3):
                        // 45 degrees
                        /*oldMode = mode;
                        controlTimer = millis();
                        controlTimeout = 3800;
                        
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
                        }*/
                        break;
                    case (1 << COUNT_R2):
                        controlTimer = millis();
                        controlTimeout = 1500;
                        
                        // 90 degrees
                        sk_p = 150;
                        sk_i = 100;
                        sk_d = 1;
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
                        controlTimer = millis();
                        controlTimeout = 2000;
                        
                        // 180 degrees
//                         sk_p = 100;
//                         sk_i = 2;
//                         sk_d = 0.02;
                        sk_p = 100;
                        sk_i = 30;
                        sk_d = 1;
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
    // If we've received a message recently, update the timeout for braking
    if (idleFlag && mode != TASK_SPIN)
    {
        oldMode = mode;
        mode = TASK_IDLE;
        demandedVelL = 0;
        demandedVelR = 0;
    }
    else {
        idleTimer = millis();
    }
    
	//First, update the motor outputs based on the mode
	switch (mode) {
		case TASK_IDLE:
        {
			demandedVelL = 0;
            demandedVelR = 0;
            PID();
			break;
        }
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
            
            omega = clamp(-50, 50, p_errorTheta * sk_p + i_errorTheta * sk_i + d_errorTheta * sk_d);
            demandedVelL = -omega;
            demandedVelR = omega;
            
            e_tTheta1 = e_tTheta;
            
            i_errorTheta += e_tTheta * DELTAT;
            
            if (fabs(e_tTheta) < 0.05f && abs(omega) < 2 && (abs(ldem - 100) < 5) && (abs(rdem - 100) < 5))
            {
                spinCount1++;
            }
            else
            {
                spinCount1 = 0;
            }
            
            if (fabs(e_tTheta) < 0.1f && abs(omega) < 2 && (abs(ldem - 100) < 5) && (abs(rdem - 100) < 5))
            {
                spinCount2++;
            }
            
            // If we're close enough for a period of time, or we've taken too long, stop the motors and revert to manual control
            if (spinCount1 > 10 || spinCount2 > 20 || (millis() - controlTimer) > controlTimeout)
            {
                lastActionTime = millis() - controlTimer;
                demandedVelL = 0;
                demandedVelR = 0;
                oldMode = mode;
                mode = TASK_MANUAL;
            }
            
            PID();
            break;
		}
		case TASK_DRIVE:
            break;
		default: //Any commands we don't care about, like TASK_LED or PNG
            break;
	}
	
    calculateLocation();
	
	loccount++;
	
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
void setMotors(int16_t lmot, int16_t rmot) {
	//Check bounds
	if (lmot > 200 || rmot > 200) {
		return;	//bad values, nothing to do
	}

	//The magnitudes
	int lmag = abs(lmot-100) * 2.55;
	int rmag = abs(rmot-100) * 2.55;
    /*
    xbSerial.println("setMotors()");
    output[0] = 0;
    snprintf(output, 50, "%d", lmot);
    xbSerial.print("lmot: ");
    xbSerial.println(output);
    output[0] = 0;
    snprintf(output, 50, "%d", rmot);
    xbSerial.print("rmot: ");
    xbSerial.println(output);
*/
	//The directions
    // If the motor does not have very small speed values, or IDLE_TIMEOUT ms have passed since we received a valid message and finished our action
/*	if ((millis() - idleTimer) > IDLE_TIMEOUT) {
        digitalWriteFast(M1IN1, LOW);
        digitalWriteFast(M1IN2, LOW);
        digitalWriteFast(M2IN1, LOW);
        digitalWriteFast(M2IN2, LOW);
        //Set the pwm outputs
        analogWrite(M1D1, 255);
        analogWrite(M2D1, 255);
	}
	else */if (!((abs(lmot - 100) < 2) && (abs(rmot - 100) < 2))) {
        digitalWriteFast(M1IN1, lmot < 100);
        digitalWriteFast(M1IN2, !(lmot < 100));
        digitalWriteFast(M2IN1, rmot < 100);
        digitalWriteFast(M2IN2, !(rmot < 100));
        //Set the pwm outputs
        analogWrite(M1D1, 255 - lmag);
        analogWrite(M2D1, 255 - rmag);
    }
	// Just finished our action, or have received no valid message and IDLE_TIMEOUT has not passed
	else {
        i_errorL = 0.f;
        i_errorR = 0.f;
        i_errorTheta = 0.f;
        i_errorDistance = 0.f;
        // Pretty sure this stuff doesn't work and it's just PID making it brake
		digitalWriteFast(M1IN1, HIGH);
		digitalWriteFast(M1IN2, HIGH);
		digitalWriteFast(M2IN1, HIGH);
		digitalWriteFast(M2IN2, HIGH);
        //Set the pwm outputs
        analogWrite(M1D1, 255);
        analogWrite(M2D1, 255);
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
    
    float deltaTheta = (rcount - lcount) * COMPASS_CONSTANT;
    float deltaDistance = (rvel + lvel) * 0.5;
    
    theta += deltaTheta;
    distance += deltaDistance;

#ifdef DEBUG
        /* Debug stuff goes here */
        /*output[0] = 0;
        switch(mode)
        {
            case TASK_IDLE:
                snprintf(output, 50, "TASK_IDLE");
                break;
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
        xbSerial.println(output);*/
//         output[0] = 0;
//         snprintf(output, 50, "%d", omega);
//         xbSerial.print("omega: ");
//         xbSerial.println(output);
//         output[0] = 0;
//         snprintf(output, 50, "%d", (int)(e_tTheta * 1000));
//         xbSerial.print("e_tTheta: ");
//         xbSerial.println(output);
//         output[0] = 0;
//         snprintf(output, 50, "%d", demandedVelR);
//         xbSerial.print("demandedVelR: ");
//         xbSerial.println(output);
        output[0] = 0;
        snprintf(output, 50, "%lu", lastActionTime);
        xbSerial.print("lastActionTime: ");
        xbSerial.println(output);
#endif
}

void setIdle() {
    idleFlag = 1;
}

void clearIdle() {
    idleFlag = 0;
}
