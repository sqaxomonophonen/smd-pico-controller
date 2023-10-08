#ifndef XOP_H // "eXecute drive OPerations".

#include "pico/stdlib.h"
#include "controller_protocol.h"

enum xop_status {
	XST_RUNNING               = 0,
	XST_DONE                  = 1,
	XST_IDLE                  = 2,
	XST_ERR0                  = 1000,
	XST_ERR_DRIVE_ERROR       = 1001,
	XST_ERR_DRIVE_NOT_READY   = 1002,
	XST_ERR_TIMEOUT           = 1999,
	XST_ERR_TEST              = 2001,
};

enum xop_status poll_xop_status(void);
absolute_time_t xop_duration_us(void);
int terminate_op(void);

void xop_reset(void);
void xop_blink_test(int fail);
void xop_tag3_strobe(unsigned ctrl);
void xop_select_unit0(void);
void xop_select_cylinder(unsigned cylinder);
void xop_broken_seek(unsigned cylinder);
void xop_select_head(unsigned head);
void xop_read_enable(int servo_offset, int data_strobe_delay);
unsigned xop_read_data(unsigned n_bytes, unsigned index_sync, unsigned raw);
void xop_read_batch(unsigned cylinder0, unsigned cylinder1, unsigned head_set, unsigned n_bytes_per_track, int servo_offset, int data_strobe_delay);
void xop_read_loop(void);

#define XOP_H
#endif
