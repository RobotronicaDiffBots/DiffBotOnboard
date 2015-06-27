#pragma once

#include "Stream.h"

void NMEACallback(char *msg, Stream *stream);

void Respond(int uid, Stream *stream);

void setMotorDemands(char *msg);

void setTargetLocation(char *msg);

void setLocation(char *msg);

void stop();

void setErr();
