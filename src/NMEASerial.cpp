#include "NMEASerial.h"
#include "util.h"

#include "HardwareSerial.h"
#include "usb_serial.h"

void NMEAReader::read() {
	while (stream->available()) {
		int ch = stream->read();
		
		if (ch == '$') {//start of new message
			pos = 0;
			reading = true;
		}
		if (!reading)
			continue;

		buf[pos++] = ch;

		if (pos >= 6 && buf[pos-5] == '*') {
			if (verify(buf, pos)) {
				buf[pos-5] = '\0';//terminate string before the *
				callback(buf + 1, stream);//skip the $
			}

			reading = false;
		}
		else if (pos == MAX_MESSAGE_LEN) {
			//error, message too long
			reading = false;
		}
	}
}

bool NMEAReader::verify(const char* msg, int len) {
	if (len < 6 || msg[0] != '$' || msg[len-2] != '\r' || msg[len-1] != '\n' || msg[len-5] != '*')
		return false;

	uint8_t chk = parseHex(msg[len-4]) << 4 | parseHex(msg[len-3]);
	uint8_t calc = 0;
	for (int i = 1; i < len - 5; i++)
		calc ^= msg[i];
	
	return chk == calc;
}