#pragma once

#include <stdint.h>
#include "Stream.h"
#include "HardwareSerial.h"
#include "core_pins.h"

#define VERBOSE             1               // Verbose mode flag for debug purposes, use with caution when other robots online

//Descriptive names for the serial objects
#define xbSerial Serial1
#define btSerial Serial2
#define topSerial Serial3

#define xbBaud 9600
#define btBaud 115200

typedef struct _radio_message {
	uint8_t     hdr0;
	uint8_t     hdr1;
	uint8_t     robotID;
	uint8_t     type;
	uint8_t     d1;
	uint8_t		d2;
	uint8_t		d3;
	uint8_t		d4;
	uint8_t     seqno;
	uint8_t     crc;
	uint8_t     nextByte;
	int         mode;
} radio_message_t;

/*
A wrapper for setup and monitoring of serial data over different interfaces
Usage: Create a serialReader object, call checkRadio(), if not null
then pass it to whatever handler function you care for
*/
class serialReader {
public:
	//Message parameters
	enum { MSG_SYNC_0 = 0, MSG_SYNC_1, MSG_QBOT_ID, MSG_TYPE, MSG_D1, MSG_D2,
		MSG_D3, MSG_D4, MSG_SEQNO, MSG_CHECKSUM };


	serialReader(Stream *stream)
		: stream(stream){
		radioMessage.seqno = 0;
		t_lastpacket = 0;
		
	};

	void setupReader();
	int checkRadio();
	uint8_t flood() { return isFlooded; }	//returns 1 if flooded with messages
	uint32_t lastValid() { return t_lastpacket; }
	radio_message_t radioMessage;

private:
	Stream *stream;

	uint32_t t_lastpacket;
	uint8_t isFlooded;
};

