#pragma once

#include "Stream.h"

#define MAX_MESSAGE_LEN 512

class NMEAReader
{
public:
	typedef void(*NMEACallback_t)(char *msg);

	NMEAReader(Stream *stream, NMEACallback_t callback) : stream(stream), callback(callback) {};
	void read();

	static bool verify(const char *msg, int len);

private:
	Stream *stream;
	NMEACallback_t callback;
	char buf[MAX_MESSAGE_LEN+1];
	uint16_t pos = 0;
	bool reading = false;
};