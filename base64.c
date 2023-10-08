#include <stdlib.h>

#include "base.h"
#include "base64.h"

#define BASE64_DIGITS "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"
//                     0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF

static inline char encode_base64_digit(unsigned v)
{
	if (v >= 64) PANIC(PANIC_XXX);
	return (BASE64_DIGITS)[v];
}

// n_bytes MUST be a multiple of 3 for partial encodes (padding characters are
// inserted otherwise)
char* base64_encode(char* output, uint8_t* input, int n_bytes)
{
	uint8_t* rp = input;
	char* wp = output;
	int n_bytes_remaining = n_bytes;
	while (n_bytes_remaining >= 3) {
		uint8_t b0 = *(rp)++;
		uint8_t b1 = *(rp)++;
		uint8_t b2 = *(rp)++;
		*(wp++) = encode_base64_digit((b0 >> 2) & 0x3f);
		*(wp++) = encode_base64_digit(((b0 & 0x3) << 4) | ((b1 >> 4) & 0xf));
		*(wp++) = encode_base64_digit(((b1 & 0xf) << 2) | ((b2 >> 6) & 0x3));
		*(wp++) = encode_base64_digit(b2 & 0x3f);
		n_bytes_remaining -= 3;
	}
	switch (n_bytes_remaining) {
	case 0: break;
	case 1: {
		uint8_t b0 = *(rp)++;
		*(wp++) = encode_base64_digit((b0 >> 2) & 0x3f);
		*(wp++) = encode_base64_digit((b0 & 0x3) << 4);
		*(wp++) = '=';
		*(wp++) = '=';
	} break;
	case 2: {
		uint8_t b0 = *(rp)++;
		uint8_t b1 = *(rp)++;
		*(wp++) = encode_base64_digit((b0 >> 2) & 0x3f);
		*(wp++) = encode_base64_digit(((b0 & 0x3) << 4) | ((b1 >> 4) & 0xf));
		*(wp++) = encode_base64_digit((b1 & 0xf) << 2);
		*(wp++) = '=';
	} break;
	default: PANIC(PANIC_XXX);
	}
	return wp;
}

static inline int decode_base64_digit(char digit)
{
	if ('A' <= digit && digit <= 'Z') {
		return digit - 'A';
	} else if ('a' <= digit && digit <= 'z') {
		return (digit - 'a') + 26;
	} else if ('0' <= digit && digit <= '9') {
		return (digit - '0') + 52;
	} else if (digit == '+') {
		return 62;
	} else if (digit == '/') {
		return 63;
	}
	return -1;
}

uint8_t* base64_decode_line(uint8_t* output, char* line)
{
	char* rp = line;
	uint8_t* wp = output;
	for (;;) {
		int digits[4] = {0};
		int padding = 0;
		int eol = 0;
		for (int i = 0; i < 4; i++) {
			char c = *(rp++);
			int d = decode_base64_digit(c);
			if (padding == 0 && d != -1) {
				digits[i] = d;
			} else {
				if (padding == 0 && ((c == 0) || (c == '\n') || (c == '\r'))) {
					if (i == 0) {
						eol = 1;
						break;
					} else {
						return NULL;
					}
				} else if (c == '=') {
					// padding char can only occur in position 2,3
					if ((i == 0) || (i == 1)) return NULL;
					padding++;
				} else {
					return NULL;
				}
			}
		}
		if (eol) break;
		switch (padding) {
		case 0: {
			*(wp++) = (digits[0] << 2) | (digits[1] >> 4);
			*(wp++) = ((digits[1] & 0xf) << 4) | (digits[2] >> 2);
			*(wp++) = ((digits[2] & 0x3) << 6) | digits[3];
		} break;
		case 1: {
			*(wp++) = (digits[0] << 2) | (digits[1] >> 4);
			*(wp++) = ((digits[1] & 0xf) << 4) | (digits[2] >> 2);
		} break;
		case 2: {
			*(wp++) = (digits[0] << 2) | (digits[1] >> 4);
		} break;
		default: PANIC(PANIC_XXX);
		}
	}
	return wp;
}

// -----------------------------------------------------------------------------------------
// cc -DUNIT_TEST base64.c -o unittest_base64 && ./unittest_base64
#ifdef UNIT_TEST

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

void enc(char* output, uint8_t* input, int n_bytes)
{
	char* w = base64_encode(output, input, n_bytes);
	*(w++) = 0;
	#if 0
	printf("enc -> [%s]\n", output);
	#endif
}

int dec(uint8_t* output, char* input)
{
	uint8_t* p = base64_decode_line(output, input);
	assert((p != NULL) && "base64_decode_line() failed");
	const int n = p - output;
	#if 0
	printf("dec -> [");
	for (int i = 0; i < n; i++) {
		printf("%s%.2x", i>0?", ":"", output[i]);
	}
	printf("]\n");
	#endif
	return n;
}

static void test0(int d, uint8_t* xs, size_t nxs, const char* expected_base64)
{
	uint8_t ys[1<<10];
	char line[1<<10];
	const int n = nxs - d;
	enc(line, xs, n);
	assert(strcmp(line, expected_base64) == 0);
	assert(strlen(line) == ((n+2)/3)*4);
	assert(dec(ys, line) == n);
	assert(memcmp(xs, ys, n) == 0);
}

static void testfuzz(void)
{
	uint8_t xs[3000];
	uint8_t ys[ARRAY_LENGTH(xs)];
	char line[4100];
	for (int i0 = 0; i0 < 100; i0++) {
		const int n = ARRAY_LENGTH(xs) - (rand() & 255);
		for (int i1 = 0; i1 < n; i1++) xs[i1] = rand() & 0xff;
		enc(line, xs, n);
		assert(strlen(line) == ((n+2)/3)*4);
		assert(dec(ys, line) == n);
		assert(memcmp(xs, ys, n) == 0);
	}
}

int main(int argc, char** argv)
{
	{
		uint8_t xs[] = {
			0x10, 0x20, 0x30,
			0x60, 0x70, 0x90,
			0xB0, 0xC0, 0xD0,
			0xFF, 0xEE, 0xDD,
			0x01, 0x02, 0x03,
			0x3c, 0x3b, 0x3a,
			0x77, 0x66, 0x55,
		};
		const size_t nxs = ARRAY_LENGTH(xs);
		test0(0, xs, nxs, "ECAwYHCQsMDQ/+7dAQIDPDs6d2ZV");
		test0(1, xs, nxs, "ECAwYHCQsMDQ/+7dAQIDPDs6d2Y=");
		test0(2, xs, nxs, "ECAwYHCQsMDQ/+7dAQIDPDs6dw==");
		test0(3, xs, nxs, "ECAwYHCQsMDQ/+7dAQIDPDs6");
	}

	testfuzz();

	printf("OK\n");

	return EXIT_SUCCESS;
}

void PANIC(uint32_t error)
{
	fprintf(stderr, "PANIC! (CODE 0x%X)\n", error);
	abort();
}

#endif
