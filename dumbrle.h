#ifndef DUMBRLE_H

#include <stdint.h>

#include "base.h"

#define DUMBRLE_ENC_IN_SZ (1<<4)
#define DUMBRLE_ENC_OUT_SZ (1<<8)

_Static_assert(((DUMBRLE_ENC_IN_SZ)  & ((DUMBRLE_ENC_IN_SZ)-1))  == 0, "input buffer size must be power-of-two");
_Static_assert(((DUMBRLE_ENC_OUT_SZ) & ((DUMBRLE_ENC_OUT_SZ)-1)) == 0, "output buffer size must be power-of-two");

struct dumbrle_enc {
	uint8_t in_ringbuf[DUMBRLE_ENC_IN_SZ];
	uint8_t out_ringbuf[DUMBRLE_ENC_OUT_SZ];

	unsigned in_write_cursor;
	unsigned in_read_cursor;
	unsigned out_write_cursor;
	unsigned out_read_cursor;

	unsigned cc_00;
	unsigned cc_FF;
	unsigned cc_other;
};


struct dumbrle_dec {
};

void dumbrle_enc_init(struct dumbrle_enc*);

unsigned dumbrle_enc_push(struct dumbrle_enc*, uint8_t* data, unsigned n);




void dumbrle_dec_init(struct dumbrle_dec*);

#define DUMBRLE_H
#endif
