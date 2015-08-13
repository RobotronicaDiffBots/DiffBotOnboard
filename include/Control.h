#pragma once

void processPacket(radio_message_t radioMessage, Stream* stream);

void updateLoopOnce();

void setupMotors();

void setMotors(uint8_t ldem, uint8_t rdem);

void calculateLocation();

void setIdle();