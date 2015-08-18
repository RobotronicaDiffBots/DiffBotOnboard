#pragma once

void processPacket(radio_message_t radioMessage, Stream* stream);

void updateLoopOnce();

void setupMotors();

void setMotors(int16_t ldem, int16_t rdem);

void calculateLocation();

void setIdle();

void clearIdle();