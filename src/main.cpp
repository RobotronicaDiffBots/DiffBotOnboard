#include "core_pins.h"
#include "usb_serial.h"
#include "HardwareSerial.h"
#include "Arduino.h"

void setup() {
	Serial.begin(115200);
	Serial2.begin(115200);
	pinMode(13, OUTPUT);
}

void loop() {
	// Keep reading from BT Module and send to Arduino Serial Monitor
	while (Serial2.available())
		Serial.write(Serial2.read());

	// Keep reading from Arduino Serial Monitor and send to BT Module
	while (Serial.available()) {
		char a = Serial.read();
		Serial.write(a);
		Serial2.write(a);
	}
}

extern "C" int main() {
	setup();
	while (1) {
		loop();
		yield();
	}
}