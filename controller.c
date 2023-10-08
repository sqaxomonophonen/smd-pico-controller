// deps
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "tusb.h"

// local
#include "pin_config.h"
#include "base.h"
#include "command_parser.h"
#include "clocked_read.h"
#include "controller_protocol.h"
#include "xop.h"
#include "base64.h"
#include "adler32.h"
#include "dumbrle.h"
#include "loopback_test.h"

unsigned stdin_received_bytes;
unsigned is_subscribing_to_status;
struct command_parser command_parser;
int is_job_polling;

static inline int gpio_type_to_dir(enum gpio_type t)
{
	switch (t) {
	case DATA:     return GPIO_IN;
	case FREQ:     return GPIO_IN;
	case STATUS:   return GPIO_IN;
	case CONTROL:  return GPIO_OUT;
	default: PANIC(PANIC_XXX);
	}
}

#define MAX_FREQUENCY_PINS (4)
unsigned frequency_counters[MAX_FREQUENCY_PINS];
absolute_time_t last_frequency_tick_timestamp;

absolute_time_t last_status_push_timestamp;
unsigned pushed_status;
int push_status_now;
unsigned prev_gpio_all;

static void status_housekeeping(void)
{
	const absolute_time_t now = get_absolute_time();
	const unsigned gpio_all = gpio_get_all();
	const unsigned gpio_edge_set = gpio_all ^ prev_gpio_all;
	prev_gpio_all = gpio_all;

	{ // handle frequent pins
		int i = 0;
		const int is_tick = (now - last_frequency_tick_timestamp) > FREQ_IN_MICROS(FREQ_FREQ_HZ);
		#define PIN(TYPE,NAME,GPN)                       \
			if (TYPE==FREQ) {                         \
				if (i >= MAX_FREQUENCY_PINS) PANIC(PANIC_BOUNDS_CHECK_FAILED); \
				if (gpio_edge_set & (1<<GPN)) {     \
					frequency_counters[i]++;     \
				}                                     \
				if (is_tick) {                         \
					if (is_subscribing_to_status) { \
						printf("%s %d %d\n", CPPP_FREQ, i, frequency_counters[i]); \
					}                                 \
					frequency_counters[i] = 0;         \
				}                                           \
				i++;                                         \
			}
		EMIT_PIN_CONFIG
		#undef PIN
		if (is_tick) last_frequency_tick_timestamp = now;
	}

	unsigned status = 0;
	{ // map status pins
		unsigned mask = 1;
		#define PIN(TYPE,NAME,GPN)                                    \
			if (TYPE==STATUS) {                                   \
				if (gpio_all & (1<<GPN)) { status |= mask; }  \
				mask <<= 1;                                   \
			}
		EMIT_PIN_CONFIG
		#undef PIN
	}

	if (push_status_now || (status != pushed_status && ((now - last_status_push_timestamp) > FREQ_IN_MICROS(200)))) {
		if (is_subscribing_to_status) {
			printf("%s %llu %d\n", CPPP_STATUS, now, status);
			last_status_push_timestamp = now;
		}
		push_status_now = 0;
		pushed_status = status;
	} else if ((now - last_status_push_timestamp) > FREQ_IN_MICROS(60)) {
		// report controller time once in a while if nothing else is
		// happening...
		if (is_subscribing_to_status) {
			printf("%s %llu\n", CPPP_TIME, now);
		}
		last_status_push_timestamp = now;
	}
}

#define DATA_TRANSFER_BYTES_PER_LINE (60)
_Static_assert((DATA_TRANSFER_BYTES_PER_LINE % 3) == 0, "must be divisible by 3 (to make base-64 encoding easier)");
#define DATA_TRANSFER_CHARACTERS_PER_LINE ((DATA_TRANSFER_BYTES_PER_LINE/3)*4)
#define DATA_TRANSFER_LINES_PER_CHUNK (10)

struct {
	int is_transfering;
	unsigned buffer_index;
	unsigned bytes_transferred;
	unsigned bytes_total;
	unsigned sequence;
	struct adler32 adler;
	struct dumbrle_enc dumbrle_enc;
} data_transfer;

static void restart_data_transfer(unsigned buffer_index)
{
	enum buffer_status st = get_buffer_status(buffer_index);
	if (st != WRITTEN && st != TRANSFERRED) PANIC(PANIC_UNEXPECTED_STATE);
	memset(&data_transfer, 0, sizeof data_transfer);
	data_transfer.is_transfering = 1;
	data_transfer.buffer_index = buffer_index;
	data_transfer.bytes_total = get_buffer_size(buffer_index);
	printf("%s %u %d %s\n", CPPP_DATA_HEADER, buffer_index, data_transfer.bytes_total, get_buffer_filename(buffer_index));
	adler32_init(&data_transfer.adler);
	dumbrle_enc_init(&data_transfer.dumbrle_enc);
}

static void handle_frontend_data_transfers(void)
{
	if (!data_transfer.is_transfering) {
		int buffer_index = get_written_buffer_index();
		if (buffer_index < 0) {
			// nothing to transfer
			return;
		}
		restart_data_transfer(buffer_index);
	}

	if (!data_transfer.is_transfering) PANIC(PANIC_UNEXPECTED_STATE);

	for (int i = 0; i < DATA_TRANSFER_LINES_PER_CHUNK && data_transfer.is_transfering; i++) {
		const int remaining = data_transfer.bytes_total - data_transfer.bytes_transferred;
		const int n = remaining > DATA_TRANSFER_BYTES_PER_LINE ? DATA_TRANSFER_BYTES_PER_LINE : remaining;

		char line[DATA_TRANSFER_CHARACTERS_PER_LINE+20];
		char* wp = line;
		int offset = snprintf(line, sizeof line, "%s %.05d ", CPPP_DATA_LINE, data_transfer.sequence++);
		if (offset <= 0) PANIC(PANIC_XXX);
		wp += offset;
		const int buffer_index = data_transfer.buffer_index;

		uint8_t* data = get_buffer_data(buffer_index) + data_transfer.bytes_transferred;
		adler32_push(&data_transfer.adler, data, n);
		wp = base64_encode(wp, data, n);
		*(wp++) = '\n';
		*(wp++) = 0;
		puts(line);
		data_transfer.bytes_transferred += n;
		if (data_transfer.bytes_transferred == data_transfer.bytes_total) {
			data_transfer.is_transfering = 0;
			transferred_buffer(buffer_index);
			uint32_t checksum = adler32_sum(&data_transfer.adler);
			printf("%s %.05d %lu\n", CPPP_DATA_FOOTER, data_transfer.sequence, checksum);
			break;
		} else if (data_transfer.bytes_transferred > data_transfer.bytes_total) {
			PANIC(PANIC_XXX);
		}
	}
}

static void handle_job_status(void)
{
	if (!is_job_polling) return;
	enum xop_status st = poll_xop_status();
	if (st == XST_DONE) {
		printf(CPPP_INFO "Job OK! (took %llu microseconds)\n", xop_duration_us());
		is_job_polling = 0;
	} else if (st >= XST_ERR0) {
		printf(CPPP_INFO "Job FAILED! (error:%d, took %llu microseconds)\n", st, xop_duration_us());
		is_job_polling = 0;
	}
}

static void job_begin(void)
{
	is_job_polling = 1;
}

static int parse(void)
{
	int got_char = getchar_timeout_us(0);

	if (got_char == PICO_ERROR_TIMEOUT || got_char == 0 || got_char >= 256) {
		return 0;
	}

	stdin_received_bytes++;
	if (!command_parser_put_char(&command_parser, got_char)) {
		return 1; // "more please"
	}

	switch (command_parser.command) {
	case COMMAND_led: {
		set_led(command_parser.arguments[0].u);
	} break;
	case COMMAND_subscribe_to_status: {
		is_subscribing_to_status = command_parser.arguments[0].b;
		push_status_now = 1;
		printf(CPPP_DEBUG "status subscription = %d\n", is_subscribing_to_status);
	} break;
	case COMMAND_poll_gpio: {
		printf(CPPP_INFO " GPIO %lx\n", gpio_get_all() & ~0x1000000);
	} break;
	case COMMAND_set_ctrl: {
		unsigned ctrl = command_parser.arguments[0].u;
		#define PUT(NAME) \
			{ \
				const unsigned mask = 1 << CONTROL_ ## NAME; \
				gpio_put(GPIO_ ## NAME, ctrl & mask); \
				ctrl = ctrl & ~mask; \
			}
		PUT(UNIT_SELECT_TAG)
		PUT(TAG1)
		PUT(TAG2)
		PUT(TAG3)
		PUT(BIT0)
		PUT(BIT1)
		PUT(BIT2)
		PUT(BIT3)
		PUT(BIT4)
		PUT(BIT5)
		PUT(BIT6)
		PUT(BIT7)
		PUT(BIT8)
		PUT(BIT9)
		#undef PUT
		if (ctrl != 0) {
			printf(CPPP_WARNING "unsupported remaining ctrl pins: %x", ctrl);
		}
	} break;
	case COMMAND_xfer_test: {
		if (!can_allocate_buffer()) {
			printf(CPPP_ERROR "no buffer available\n");
		} else {
			unsigned size = command_parser.arguments[0].u;
			const unsigned buffer_index = allocate_buffer(size);
			char* s = get_buffer_filename(buffer_index);
			snprintf(s, CLOCKED_READ_BUFFER_FILENAME_MAX_LENGTH, "_xfertest-bufidx%d-%dbytes.garbage", buffer_index, size);
			size = get_buffer_size(buffer_index); // size can be truncated by MAX_DATA_BUFFER_SIZE
			uint8_t* p = get_buffer_data(buffer_index);
			for (unsigned i = 0; i < size; i++) {
				*(p++) = (i & 0xff) + ((i >> 8) & 0xff) + ((i >> 16) & 0xff) + ((i >> 24) & 0xff);
			}
			wrote_buffer(buffer_index);
		}
	} break;
	case COMMAND_loopback_test: {
		uint n_bytes = command_parser.arguments[0].u;
		printf(CPPP_INFO "firing loopback test with %d bytes\n", n_bytes);
		loopback_test_fire(n_bytes);
	} break;
	case COMMAND_terminate_op: {
		if (terminate_op()) {
			printf(CPPP_INFO "op terminated\n");
		} else {
			printf(CPPP_INFO "no op is running\n");
		}
	} break;
	case COMMAND_data_ack: {
		release_buffer(command_parser.arguments[0].u);
	} break;
	case COMMAND_data_retry: {
		restart_data_transfer(command_parser.arguments[0].u);
	} break;
	case COMMAND_data_cancel: {
		const unsigned buffer_index = command_parser.arguments[0].u; // TODO
	} break;
	case COMMAND_op_reset: {
		job_begin();
		reset_buffers();
		xop_reset();
	} break;
	case COMMAND_op_blink_test: {
		const int fail = command_parser.arguments[0].u;
		job_begin();
		xop_blink_test(fail);
	} break;
	case COMMAND_op_tag3_strobe: {
		job_begin();
		const int ctrl = command_parser.arguments[0].u;
		xop_tag3_strobe(ctrl);
	} break;
	case COMMAND_op_select_unit0: {
		job_begin();
		xop_select_unit0();
	} break;
	case COMMAND_op_select_cylinder: {
		job_begin();
		xop_select_cylinder(command_parser.arguments[0].u);
	} break;
	case COMMAND_op_broken_seek: {
		job_begin();
		xop_broken_seek(command_parser.arguments[0].u);
	} break;
	case COMMAND_op_select_head: {
		job_begin();
		xop_select_head(command_parser.arguments[0].u);
	} break;
	case COMMAND_op_read_enable: {
		int servo_offset = command_parser.arguments[0].i;
		int data_strobe_delay = command_parser.arguments[1].i;
		job_begin();
		xop_read_enable(servo_offset, data_strobe_delay);
	} break;
	case COMMAND_op_read_data: {
		if (!can_allocate_buffer()) {
			printf(CPPP_ERROR "no buffer available\n");
		} else {
			job_begin();
			unsigned buffer_index = xop_read_data(
				command_parser.arguments[0].u,
				command_parser.arguments[1].u,
				command_parser.arguments[2].u);
			printf(CPPP_DEBUG "reading into buffer %d\n", buffer_index);
		}
	} break;
	case COMMAND_op_read_batch: {
		const unsigned cylinder0     = command_parser.arguments[0].u;
		const unsigned cylinder1     = command_parser.arguments[1].u;
		const unsigned head_set      = command_parser.arguments[2].u;
		const unsigned n_bytes       = command_parser.arguments[3].u;
		const int servo_offset       = command_parser.arguments[4].i;
		const int data_strobe_delay  = command_parser.arguments[5].i;
		job_begin();
		xop_read_batch(cylinder0, cylinder1, head_set, n_bytes, servo_offset, data_strobe_delay);
	} break;
	case COMMAND_op_read_loop: {
		job_begin();
		xop_read_loop();
	} break;
	default: {
		printf(CPPP_ERROR "unhandled command %s/%d\n",
			command_to_string(command_parser.command),
			command_parser.command);
	} break;
	}
	return 0;
}

int main()
{
	// I/O pin config
	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	gpio_put(LED_PIN, 0);
	#define PIN(TYPE, NAME, GPN)                                       \
		gpio_init(GPN);                                            \
		gpio_set_dir(GPN, gpio_type_to_dir(TYPE));                 \
		if (gpio_type_to_dir(TYPE) == GPIO_OUT) {                  \
			gpio_put(GPN, 0);                                  \
		} else {                                                   \
			gpio_pull_down(GPN); /* prevent floating inputs */ \
		}
	EMIT_PIN_CONFIG
	#undef PIN

	clocked_read_init(pio0,  /*dma_channel=*/0);
	loopback_test_prep(pio1, /*dma_channel=*/1);

	stdio_init_all();

	blink(50, 0); // "Hi, we're up!"

	for (;;) {
		for (int i = 0; i < 50; i++) {
			if (!parse()) break;
		}
		status_housekeeping();
		handle_frontend_data_transfers();
		handle_job_status();
		loopback_test_tick();
		//tight_loop_contents(); // does nothing
		tud_task(); // tinyusb work
	}

	PANIC(PANIC_STOP);
}
