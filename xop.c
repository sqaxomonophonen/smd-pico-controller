// DESIGN NOTE: The "drive operations code" in here is executed on core1. It's
// slightly wasteful because operations busy wait most of the time. One
// reedeming quality with this design is that core0 operations can't delay
// drive operations, but frankly I chose this design because it's easier to
// write than various ways of doing async code in C. Also, I really don't have
// anything else to use core1 for? (all the high bandwidth heavy lifting is
// entirely handled by PIO/DMA)

#include <stdio.h>
#include "pico/multicore.h"

#include "base.h"
#include "pin_config.h"
#include "drive.h"
#include "controller_protocol.h"
#include "xop.h"
#include "clocked_read.h"

#define ERROR_MASK                \
	( (1 << GPIO_FAULT)       \
	| (1 << GPIO_SEEK_ERROR)  \
	)

#define READY_MASK                   \
	( (1 << GPIO_UNIT_READY)     \
	| (1 << GPIO_UNIT_SELECTED)  \
	)

#define TAG_STROBE_SLEEP() sleep_us(2)
// I haven't seen anything in the docs about how long a "tag pin" should be
// held high before the drive registers the signal. For specific
// operations/pins I'm seeing quotes of 250 ns to 1.0 µs, so 2.0 µs should be
// abundant?

absolute_time_t job_begin_time_us;
absolute_time_t job_duration_us;
volatile enum xop_status status;
unsigned current_cylinder_according_to_the_controller;

static void unit0_select_tag(void)
{
	gpio_put(GPIO_UNIT_SELECT_TAG, 1);
}

static void set_bits(unsigned value)
{
	#define PUT(N) gpio_put(GPIO_BIT ## N, value & (1<<N))
	PUT(0); PUT(1); PUT(2); PUT(3); PUT(4);
	PUT(5); PUT(6); PUT(7); PUT(8); PUT(9);
	#undef PUT
}

static void clear_output(void)
{
	gpio_put(GPIO_TAG1, 0);
	gpio_put(GPIO_TAG2, 0);
	gpio_put(GPIO_TAG3, 0);
	set_bits(0);
}

#define TAG_SLEEP_US (10)

static void tag1_cylinder(unsigned cylinder)
{
	clear_output();
	set_bits(cylinder);
	sleep_us(TAG_SLEEP_US);
	gpio_put(GPIO_TAG1, 1);
	sleep_us(TAG_SLEEP_US);
	clear_output();
	// NOTE: does not set current_cylinder_according_to_the_controller
}

static void tag2_head(unsigned head)
{
	clear_output();
	set_bits(head);
	gpio_put(GPIO_TAG2, 1);
	sleep_us(TAG_SLEEP_US);
	clear_output();
}

static void tag3_ctrl(unsigned ctrl)
{
	clear_output();
	set_bits(ctrl);
	sleep_us(TAG_SLEEP_US);
	gpio_put(GPIO_TAG3, 1);
}

static void tag3_ctrl_strobe(unsigned ctrl)
{
	tag3_ctrl(ctrl);
	sleep_us(TAG_SLEEP_US);
	clear_output();
}

static void BEGIN(void)
{
	job_begin_time_us = get_absolute_time();
}

__attribute__ ((noreturn))
static void job_halt(void)
{
	job_duration_us = get_absolute_time() - job_begin_time_us;
	while (1) {}
}

__attribute__ ((noreturn))
static void DONE(void)
{
	status = XST_DONE;
	job_halt();
}

__attribute__ ((noreturn))
static void ERROR(enum xop_status error_code)
{
	status = error_code;
	job_halt();
}

static void check_drive_error(void)
{
	unsigned pins = gpio_get_all();
	if ((pins & ERROR_MASK) != 0)          ERROR(XST_ERR_DRIVE_ERROR);
	if ((pins & READY_MASK) != READY_MASK) ERROR(XST_ERR_DRIVE_NOT_READY);
}

static void pin_mask_wait(unsigned mask, unsigned value, unsigned timeout_us, int check_error)
{
	const absolute_time_t t0 = get_absolute_time();
	while (1) {
		if (check_error) check_drive_error();
		if ((gpio_get_all() & mask) == value) break;
		if ((get_absolute_time() - t0) > timeout_us) {
			ERROR(XST_ERR_TIMEOUT);
		}
		sleep_us(1);
	}
}

static void pin_wait(unsigned gpio, unsigned value, unsigned timeout_us, int check_error)
{
	pin_mask_wait((1<<gpio), value ? (1<<gpio) : 0, timeout_us, check_error);
}

static void pin_wait_for_one(unsigned gpio, unsigned timeout_us, int check_error)
{
	pin_wait(gpio, 1, timeout_us, check_error);
}

static void pin_wait_for_zero(unsigned gpio, unsigned timeout_us, int check_error)
{
	pin_wait(gpio, 0, timeout_us, check_error);
}

static void return_to_normal(void)
{
	current_cylinder_according_to_the_controller = 0;

	if ((gpio_get_all() & (1 << GPIO_FAULT))) {
		tag3_ctrl_strobe(TAG3BIT_FAULT_CLEAR);
		pin_wait_for_zero(GPIO_FAULT, 1000000, 0);
	}

	tag3_ctrl(TAG3BIT_RTZ);
	sleep_us(500000);
	clear_output();
}

static void select_unit0(void)
{
	unit0_select_tag();
	pin_wait_for_one(GPIO_UNIT_SELECTED, 100000, 0);
	check_drive_error();
}

static void read_enable_ex(int servo_offset, int data_strobe_delay)
{
	check_drive_error();
	const unsigned ctrl =

		TAG3BIT_READ_GATE

		| (data_strobe_delay > 0 ? TAG3BIT_DATA_STROBE_LATE
		:  data_strobe_delay < 0 ? TAG3BIT_DATA_STROBE_EARLY
		: 0);

	tag3_ctrl(ctrl);
}

static void select_cylinder(unsigned cylinder)
{
	tag1_cylinder(cylinder);
	// Assuming it might take a little while before ON_CYLINDER and
	// SEEK_END go low?
	sleep_us(1000);
	// NOTE: the drive should signal SEEK_ERROR (which IS caught by
	// pin_mask_wait()) if the seek does not complete within 500ms
	const unsigned bits = (1<<GPIO_ON_CYLINDER) | (1<<GPIO_SEEK_END);
	// NOTE: drive doc says that "Seek End is a combination of ON CYL or
	// SEEK ERROR" suggesting it's a simple OR-gate of those signals. But
	// it's a good sanity check nevertheless (cable/drive may be broken).
	pin_mask_wait(bits, bits, 1000000, 1);
	current_cylinder_according_to_the_controller = cylinder;
}

// seek in single-cylinder steps; the drive divides seeking into two phases:
// coarse seek (acceleration, coasting, deacceleration) and fine seek. as far
// as we can tell from the schematics, single-stepping skips coarse seeking
// entirely. so because coarse seeking both takes up a significant fraction of
// the ICs, and also because it is finicky (subject to a lot of tuning), there
// are a lot of things that can go wrong with it. in "real-life" we had a drive
// that could only single-step.
static void broken_seek(unsigned cylinder)
{
	for (;;) {
		if (cylinder > current_cylinder_according_to_the_controller) {
			select_cylinder(current_cylinder_according_to_the_controller + 1);
		} else if (cylinder < current_cylinder_according_to_the_controller) {
			select_cylinder(current_cylinder_according_to_the_controller - 1);
		} else {
			break;
		}
	}
}

static void select_head(unsigned head)
{
	check_drive_error();
	tag2_head(head);
}

static inline void read_data(unsigned buffer_index, unsigned n_bytes, unsigned index_sync, unsigned skip_checks)
{
	if (!skip_checks) check_drive_error();
	if (index_sync) {
		//pin_wait_for_zero(GPIO_INDEX, FREQ_IN_MICROS(DRIVE_RPS)/10, !skip_checks);
		//pin_wait_for_one(GPIO_INDEX, FREQ_IN_MICROS(DRIVE_RPS/3), !skip_checks); // wait at most 3 revolutions
		pin_wait_for_zero(GPIO_INDEX, 1000000, !skip_checks);
		pin_wait_for_one(GPIO_INDEX,  1000000, !skip_checks);
	}
	clocked_read_into_buffer(buffer_index, n_bytes);
	while (1) {
		if (!clocked_read_is_running()) break;
		if (!skip_checks) check_drive_error();
		sleep_us(1);
	}
	wrote_buffer(buffer_index);
}

#if 0
static void read_data_normal(unsigned buffer_index, unsigned n_bytes)
{
	read_data(buffer_index, n_bytes, /*index_sync=*/1, /*skip_checks=*/0);
}
#endif

static inline void reset(void)
{
	multicore_reset_core1(); // waits until core1 is down
	status = XST_IDLE;
}

static inline void reset_and_kill_output(void)
{
	reset();
	sleep_us(1);
	clear_output();
}


static void run(void(*fn)(void))
{
	status = XST_RUNNING;
	multicore_launch_core1(fn);
}

enum xop_status poll_xop_status(void)
{
	return status;
}

absolute_time_t xop_duration_us(void)
{
	return job_duration_us;
}

int terminate_op(void)
{
	int was_running = (status == XST_RUNNING);
	reset_and_kill_output();
	return was_running;
}

union {
	struct {
		int fail;
	} blink_test;
	struct {
		unsigned cylinder;
	} select_cylinder;
	struct {
		unsigned cylinder;
	} broken_seek;
	struct {
		unsigned head;
	} select_head;
	struct {
		unsigned ctrl;
	} tag3_strobe;
	struct {
		int servo_offset;
		int data_strobe_delay;
	} read_enable;
	struct {
		unsigned buffer_index;
		unsigned n_bytes;
		unsigned index_sync;
		unsigned skip_checks;
	} read_data;
	struct {
		unsigned n_bytes_per_track;
		unsigned cylinder0;
		unsigned cylinder1;
		unsigned head_set;
		int servo_offset;
		int data_strobe_delay;
	} batch_read;

} job_args;

////////////////////////////////////
// reset ///////////////////////////
void job_reset(void)
{
	BEGIN();
	return_to_normal();
	DONE();
}
void xop_reset(void)
{
	reset();
	run(job_reset);
}

////////////////////////////////////
// blink test //////////////////////
void job_blink_test(void)
{
	BEGIN();
	for (int i = 0; i < 15; i++) {
		gpio_put(LED_PIN, 1);
		sleep_ms(50);
		gpio_put(LED_PIN, 0);
		sleep_ms(50);
	}
	if (!job_args.blink_test.fail) {
		DONE();
	} else {
		ERROR(XST_ERR_TEST);
	}
}
void xop_blink_test(int fail)
{
	reset();
	job_args.blink_test.fail = fail;
	run(job_blink_test);
}


////////////////////////////////////
// select unit 0 ///////////////////
void job_select_unit0(void)
{
	BEGIN();
	select_unit0();
	DONE();
}
void xop_select_unit0(void)
{
	reset_and_kill_output();
	run(job_select_unit0);
}


/////////////////////////////////////////////////////////////////////////////
// tag3 / short strobe //////////////////////////////////////////////////////
void job_tag3_strobe(void)
{
	BEGIN();
	tag3_ctrl_strobe(job_args.tag3_strobe.ctrl);
	DONE();
}
void xop_tag3_strobe(unsigned ctrl)
{
	reset_and_kill_output();
	job_args.tag3_strobe.ctrl = ctrl;
	run(job_tag3_strobe);
}

/////////////////////////////////////////////////////////////////////////////
// select cylinder //////////////////////////////////////////////////////////
void job_select_cylinder(void)
{
	BEGIN();
	select_cylinder(job_args.broken_seek.cylinder);
	DONE();
}
void xop_select_cylinder(unsigned cylinder)
{
	reset_and_kill_output();
	job_args.broken_seek.cylinder = cylinder;
	run(job_select_cylinder);
}


/////////////////////////////////////////////////////////////////////////////
// "broken seek" ////////////////////////////////////////////////////////////
void job_broken_seek(void)
{
	BEGIN();
	broken_seek(job_args.broken_seek.cylinder);
	DONE();
}
void xop_broken_seek(unsigned cylinder)
{
	reset_and_kill_output();
	job_args.broken_seek.cylinder = cylinder;
	run(job_broken_seek);
}

/////////////////////////////////////////////////////////////////////////////
// select head //////////////////////////////////////////////////////////////
void job_select_head(void)
{
	BEGIN();
	select_head(job_args.select_head.head);
	DONE();
}
void xop_select_head(unsigned head)
{
	reset_and_kill_output();
	job_args.select_head.head = head;
	run(job_select_head);
}

/////////////////////////////////////////////////////////////////////////////
// read enable //////////////////////////////////////////////////////////////
void job_read_enable(void)
{
	BEGIN();
	read_enable_ex(
		job_args.read_enable.servo_offset,
		job_args.read_enable.data_strobe_delay);
	DONE();
}
void xop_read_enable(int servo_offset, int data_strobe_delay)
{
	reset_and_kill_output();
	job_args.read_enable.servo_offset = servo_offset;
	job_args.read_enable.data_strobe_delay = data_strobe_delay;
	run(job_read_enable);
}

/////////////////////////////////////////////////////////////////////////////
// read data ////////////////////////////////////////////////////////////////
unsigned next_read_data_serial = 1;
void job_read_data(void)
{
	BEGIN();
	const unsigned buffer_index = job_args.read_data.buffer_index;
	snprintf(
		get_buffer_filename(buffer_index),
		CLOCKED_READ_BUFFER_FILENAME_MAX_LENGTH,
		"custom%.4d.nrz",
		next_read_data_serial++);
	read_data(
		buffer_index,
		job_args.read_data.n_bytes,
		job_args.read_data.index_sync,
		job_args.read_data.skip_checks);
	DONE();
}
unsigned xop_read_data(unsigned n_bytes, unsigned index_sync, unsigned skip_checks)
{
	reset();
	const unsigned buffer_index = allocate_buffer(n_bytes);
	job_args.read_data.buffer_index = buffer_index;
	job_args.read_data.n_bytes = n_bytes;
	job_args.read_data.index_sync = index_sync;
	job_args.read_data.skip_checks = skip_checks;
	run(job_read_data);
	return buffer_index;
}


/////////////////////////////////////////////////////////////////////////////
// batch read ///////////////////////////////////////////////////////////////
void job_batch_read(void)
{
	BEGIN();
	check_drive_error();
	const unsigned cylinder0 = job_args.batch_read.cylinder0;
	const unsigned cylinder1 = job_args.batch_read.cylinder1;
	const unsigned head_set = job_args.batch_read.head_set;
	const unsigned n_bytes_per_track = job_args.batch_read.n_bytes_per_track;
	const int arg_servo_offset = job_args.batch_read.servo_offset;
	const int arg_data_strobe_delay = job_args.batch_read.data_strobe_delay;

	int servo_offset0 = arg_servo_offset == ENTIRE_RANGE ? -1 : arg_servo_offset;
	if (servo_offset0 < -1) servo_offset0 = -1;

	int servo_offset1 = arg_servo_offset == ENTIRE_RANGE ?  1 : arg_servo_offset;
	if (servo_offset1 >  1) servo_offset1 = 1;

	int data_strobe_delay0 = arg_data_strobe_delay == ENTIRE_RANGE ? -1 : arg_data_strobe_delay;
	if (data_strobe_delay0 < -1) data_strobe_delay0 = -1;

	int data_strobe_delay1 = arg_data_strobe_delay == ENTIRE_RANGE ?  1 : arg_data_strobe_delay;
	if (data_strobe_delay1 > 1) data_strobe_delay1 = 1;

	for (unsigned cylinder = cylinder0; cylinder <= cylinder1; cylinder++) {
		select_cylinder(cylinder);
		// The CDC docs lists "read while off cylinder" as one of the
		// conditions that can trigger a FAULT. Although the following
		// section suggests the fault is only generated if requested
		// while seeking?:
		//   "(Read or Write) and Off Cylinder Fault"
		//   "This fault is generated if the drive is in an Off
		//   Cylinder condition and it receives a Read or Write gate
		//   from the controller."
		unsigned mask = 1;
		for (unsigned head = 0; head < DRIVE_HEAD_COUNT; head++, mask <<= 1) {
			if ((head_set & mask) == 0) continue;
			select_head(head);
			// XXX not sure if a delay is required here?

			// FIXME: TRY: it might be possible to first set tag3,
			// then servo offset, then wait for on cylinder, and
			// then read gate

			for (int data_strobe_delay = data_strobe_delay0; data_strobe_delay <= data_strobe_delay1; data_strobe_delay++) {
				sleep_us(5);
				const absolute_time_t t0 = get_absolute_time();
				while (!can_allocate_buffer()) {
					if ((get_absolute_time() - t0) > 10000000) {
						ERROR(XST_ERR_TIMEOUT);
					}
					sleep_us(5);
				}
				const unsigned buffer_index = allocate_buffer(n_bytes_per_track);
				snprintf(
					get_buffer_filename(buffer_index),
					CLOCKED_READ_BUFFER_FILENAME_MAX_LENGTH,
					//"cylinder%.4d-head%d-servo-%s-strobe-%s.nrz", cylinder, head, servo_offset+1, data_strobe_delay+1);
					"cylinder%.4d-head%d%s.nrz", cylinder, head,

					data_strobe_delay == -1 ? "-strobe-early" :
					data_strobe_delay ==  1 ? "-strobe-late" :
					""
				);

				read_enable_ex(0, data_strobe_delay);
				read_data(
					// XXX combine these 2? I don't like the redundancy
					buffer_index,
					n_bytes_per_track,
					/*index_sync=*/1,
					/*skip_checks=*/0);
				clear_output();
			}
		}
	}
	DONE();
}
void xop_read_batch(unsigned cylinder0, unsigned cylinder1, unsigned head_set, unsigned n_bytes_per_track, int servo_offset, int data_strobe_delay)
{
	reset_and_kill_output();
	job_args.batch_read.n_bytes_per_track = n_bytes_per_track;
	job_args.batch_read.cylinder0 = cylinder0;
	job_args.batch_read.cylinder1 = cylinder1;
	job_args.batch_read.head_set = head_set;
	job_args.batch_read.servo_offset = servo_offset;
	job_args.batch_read.data_strobe_delay = data_strobe_delay;
	run(job_batch_read);
}


/////////////////////////////////////////////////////////////////////////////
// read loop ////////////////////////////////////////////////////////////////
unsigned loop_counter;
void job_read_loop(void)
{
	BEGIN();
	const unsigned n_bytes = MAX_DATA_BUFFER_SIZE;
	for (;;) {
		const absolute_time_t t0 = get_absolute_time();
		while (!can_allocate_buffer()) {
			if ((get_absolute_time() - t0) > 10000000) {
				ERROR(XST_ERR_TIMEOUT);
			}
			sleep_us(5);
		}
		const unsigned buffer_index = allocate_buffer(n_bytes);
		loop_counter++;
		snprintf(
			get_buffer_filename(buffer_index),
			CLOCKED_READ_BUFFER_FILENAME_MAX_LENGTH,
			"loop-%.5d.nrz", loop_counter);
		read_data(
			buffer_index,
			n_bytes,
			/*index_sync=*/1,
			/*skip_checks=*/1);
	}
	PANIC(PANIC_UNREACHABLE);
}
void xop_read_loop(void)
{
	reset();
	run(job_read_loop);
}
