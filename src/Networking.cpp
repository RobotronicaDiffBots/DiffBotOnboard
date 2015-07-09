#include "Networking.h"

#define QBOT_ID             1               // The unique ID of the robot (0-255), 250 is global
#define MAX_DURATION		100				// The longest we'll try analyse incoming packets (ms)
/**
* Read the light weight serial packets, check validity and action if specified for the current robot.
* Currently set up for manual control of left motor and right motor. If want heading and speed, this can
* be done, but see Chris Dirkis and Matt Dunbabin for change of code base.
*
* Packet Structure (10 Bytes):
*      hdr0        - First sync byte 0xAA
*      hdr1        - Second sync byte 0x55
*      robotID     - Targeted robot ID number (0-255). Special case 250 = all robots
*      type        - Specific task to be executed by the robot (needs master enum list)
*      d1		   - Data field 1
*      d2		   - Data field 2
*      d3		   - Data field 3
*      d4		   - Data field 4
*      seqno       - check that current packet is unique (one byte)
*      crc         - checksum for incoming packet from hdr0 to seqno (one byte)
*/

int serialReader::checkRadio() {
	unsigned long ts;
	ts = micros();

	//Assume not flooded at the start (update later)
	isFlooded = 0;

	uint8_t crc = 0;
	uint8_t *p = (uint8_t *)&radioMessage;
	

	while (stream->available() > 0) {
		radioMessage.nextByte = (uint8_t)(stream->read());
		//TODO remove
		stream->write(radioMessage.nextByte);

		switch (radioMessage.mode) {
		case MSG_SYNC_0:
			if (radioMessage.nextByte == 0xAA) {
				radioMessage.mode = MSG_SYNC_1;
			}
			break;

		case MSG_SYNC_1:
			if (radioMessage.nextByte == 0x55) {
				radioMessage.mode = MSG_QBOT_ID;
			}
			else {
				radioMessage.mode = MSG_SYNC_0;
			}
			break;

		case MSG_QBOT_ID:
			radioMessage.robotID = radioMessage.nextByte;
			if ((radioMessage.robotID == QBOT_ID) || (radioMessage.robotID == 250)) {
				radioMessage.mode = MSG_TYPE;
			}
			else {
				radioMessage.mode = MSG_SYNC_0;
			}
			break;

		case MSG_TYPE:
			radioMessage.type = radioMessage.nextByte;
			radioMessage.mode = MSG_D1;
			break;

		case MSG_D1:
			radioMessage.d1 = radioMessage.nextByte;
			radioMessage.mode = MSG_D2;
			break;

		case MSG_D2:
			radioMessage.d2 = radioMessage.nextByte;
			radioMessage.mode = MSG_D3;
			break;

		case MSG_D3:
			radioMessage.d3 = radioMessage.nextByte;
			radioMessage.mode = MSG_D4;
			break;

		case MSG_D4:
			radioMessage.d4 = radioMessage.nextByte;
			radioMessage.mode = MSG_SEQNO;
			break;

		case MSG_SEQNO:
			if (radioMessage.nextByte != radioMessage.seqno) {
				radioMessage.seqno = radioMessage.nextByte;
				radioMessage.mode = MSG_CHECKSUM;
			}
			else {
				radioMessage.mode = MSG_SYNC_0;
			}
			break;

		case MSG_CHECKSUM:
			radioMessage.crc = radioMessage.nextByte;
			/* Calculate the CRC */
			for (int k = 0; k < 9; k++) {
				crc ^= p[k];
			}

			radioMessage.mode = MSG_SYNC_0;
			if (radioMessage.crc == crc) {
				return 1;
			}
			else {
				return 0;
			}
		default:
			break;
		}

		/* timeout hack in case getting swampped by irrelevant packets */
		if ((micros() - ts) > MAX_DURATION) {
			if (VERBOSE) {
				stream->write("Too many packets\r\n");
			}
			radioMessage.mode = MSG_SYNC_0;
			isFlooded = 1;
			return 0;
		}
	}
	return 0;
}

