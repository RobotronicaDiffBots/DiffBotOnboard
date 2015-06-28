#pragma once

#include "Encoder.h"


//PINS
#define LENC1 16
#define LENC2 17
#define RENC1 3
#define RENC2 4

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

void normaliseHeading();
void readEncoders();
void setupCompass();
void checkCompass();
void normaliseCompassHeading();
void normaliseEstHeading();
void setupMotors();
void updateMotors();
