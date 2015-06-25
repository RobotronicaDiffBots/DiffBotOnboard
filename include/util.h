#pragma once

#include "stdint.h"

uint8_t parseHex(char c) {
	if (c >= 'a') return c - 'a' + 0xA;
	if (c >= 'A') return c - 'A' + 0xA;
	return c - '0';
}

char toHex(uint8_t i) {
	if (i >= 0xA) return 'A' + i - 0xA;
	return '0' + i;
}