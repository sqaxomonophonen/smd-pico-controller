#include <stdint.h>
#include <string.h>

/*
dumbrle run-length encodes runs of 0x00, 0xff, or "bytes". Each run begins with
a u16 word:
 2 + (n<<2); encodes <n> bytes of 0x00
 3 + (n<<2); encodes <n> bytes of 0xff
 1 + (n<<2), followed by <n> bytes of data; encodes the data as-is
 0 + (n<<2); reserved/unused
<n> is an unsigned 14-bit number

So,
   0x13, 0x00,       0x11, 0x00,  0x10, 0x20, 0x30, 0x40
   (3+(4<<2)=0x13)   (1+(4<<2)=0x11)
encodes the sequence 0xff, 0xff, 0xff, 0xff, 0x10, 0x20, 0x30, 0x40
u16 values are stored as a little-endian number, so that the first byte read
determines the run type.

Rationale:
 - The goal is to transfer data to the PC as fast as possible. The drive reads
   data at roughly 10Mbit/s, but sending the data over USB-TTY with base64
   encoding bottlenecks at ~1/3 of that. So a very fast ~3-4x compression is
   ideal.
 - There are a lot of 0x00/0xff runs in the data we see, sometimes as much as
   90%+ of the total data.
 - There's no word/byte alignment in the data read from the drive, so the
   "fixed dictionary entries" that make most sense are 0x00/0xff.
 - Compression can be done with very little buffer overhead; I prefer to
   reserve most of the RP2040 RAM for DMA buffers.
 - Compression can be streamed; no need to analyze data first.
 - I also considered using ryg_rans with a predefined probability distribution.
*/

#include "dumbrle.h"

void dumbrle_enc_init(struct dumbrle_enc* dre)
{
	memset(dre, 0, sizeof *dre);
}

static void enc_flush(

unsigned dumbrle_enc_push(struct dumbrle_enc* e, uint8_t* data, unsigned n)
{
	unsigned remaining = n;
	uint8_t* p = data;
	while (remaining > 0) {
		uint8_t b = *(p++);
	}
}

// -----------------------------------------------------------------------------------------
// cc -DSTANDALONE_TEST dumbrle.c -o dumbrle && ./dumbrle
#ifdef STANDALONE_TEST

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

int main(int argc, char** argv)
{
	if (argc != 2) {
		fprintf(stderr, "Usage: %s </path/to/nrz>\n", argv[0]);
		exit(EXIT_FAILURE);
	}

	FILE* f = fopen(argv[1], "rb");
	if (f == NULL) {
		fprintf(stderr, "%s: could not open\n", argv[1]);
		exit(EXIT_FAILURE);
	}

	assert(fseek(f, 0, SEEK_END) == 0);
	long sz = ftell(f);
	assert(fseek(f, 0, SEEK_SET) == 0);

	uint8_t* data = malloc(sz);
	assert(fread(data, sz, 1, f) == 1);

	assert(fclose(f) == 0);

	for (;;) {
	}

	return EXIT_SUCCESS;
}

#endif
