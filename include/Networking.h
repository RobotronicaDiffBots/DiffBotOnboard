#pragma once

#include <stdint.h>
#include "Stream.h"
#include "HardwareSerial.h"
#include "core_pins.h"

#define VERBOSE             0               // Verbose mode flag for debug purposes, use with caution when other robots online

#define DT_CONTROL_MS       10             // Main sample loop timer (ms)

//Descriptive names for the serial objects
#define xbSerial Serial1
#define btSerial Serial2
#define topSerial Serial3

#define xbBaud 57600
#define btBaud 115200
#define topBaud 9600
/*
* Packet Structure (10 Bytes):
*      hdr0        - First sync byte 0xAA
*      hdr1        - Second sync byte 0x55
*      robotID     - Targeted robot ID number (0-255). Special case 250 = all robots
*      type        - Specific task tso be executed by the robot (needs master enum list)
*      d1		   - Data field 1
*      d2		   - Data field 2
*      d3		   - Data field 3
*      d4		   - Data field 4
*      seqno       - check that current packet is unique (one byte)
*      crc         - checksum for incoming packet from hdr0 to seqno (one byte)
*/
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

	//Setup some default values
	serialReader(Stream *stream) : stream(stream) {
		radioMessage.seqno = 255;	//The first message should be 0, don't want to collide
		t_lastpacket = 0;			//Time since last packet
		radioMessage.hdr0 = 0xAA;	//The first two parameters of the message are always the same
		radioMessage.hdr1 = 0x55;
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

char toHex(uint8_t i);