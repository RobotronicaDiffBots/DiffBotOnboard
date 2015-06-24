void setup() {
	Serial.begin(115200);
	Serial2.begin(115200);
}
void loop() {
	// Keep reading from BT Module and send to Arduino Serial Monitor
	if (Serial2.available())
		Serial.write(Serial2.read());

	// Keep reading from Arduino Serial Monitor and send to BT Module
	if (Serial.available()) {
		char a = Serial.read();
		Serial.write(a);
		Serial2.write(a);
	}
}
	
int main() {
	setup();
	while(1) 
		loop();
}