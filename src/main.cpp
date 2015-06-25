#include "core_pins.h"
#include "usb_serial.h"
#include "HardwareSerial.h"
#include "NMEASerial.h"

void btNMEACallback(const char *msg) {
	Serial2.write(msg);
}

NMEAReader btReader(&Serial2, btNMEACallback);

void setup() {
	Serial.begin(115200);
	Serial2.begin(115200);
}

void loop() {
	btReader.read();
}

extern "C" int main() {
	setup();
	while (1)
		loop();
}