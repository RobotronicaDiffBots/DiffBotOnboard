#pragma once

#include "NMEASerial.h"

#include "Stream.h"

//Serial ports
#define xbSerial Serial1
#define btSerial Serial2
#define topSerial Serial3

void readSerial();

void setupNetworking();

void NMEACallback(char *msg, Stream *stream);

void Respond(int uid, Stream *stream);

void setMotorDemands(char *msg);

void setTargetLocation(char *msg);

void setLocation(char *msg);

void stop();

void setErr();

bool timeoutCheck();