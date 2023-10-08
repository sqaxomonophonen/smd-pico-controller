#include "hardware/dma.h"

#include "clocked_read.h"
#include "clocked_read.pio.h"
#include "base.h"
#include "pin_config.h"

struct buffer {
	uint8_t data[MAX_DATA_BUFFER_SIZE];
	unsigned size;
	enum buffer_status status;
	char filename[CLOCKED_READ_BUFFER_FILENAME_MAX_LENGTH];
};

struct buffer buffers[CLOCKED_READ_BUFFER_COUNT];

_Static_assert(sizeof(buffers[0]) % 4 == 0);

static PIO pio;
static uint sm;
static uint dma_channel;

static inline void clocked_read_program_init(PIO pio, uint sm, uint offset, uint pin0_data)
{
	pio_sm_config cfg = clocked_read_program_get_default_config(offset);
	sm_config_set_in_pins(&cfg, pin0_data);
	pio_sm_set_consecutive_pindirs(pio, sm, pin0_data, /*pin_count=*/2, /*is_out=*/false);
	pio_gpio_init(pio, pin0_data);
	pio_gpio_init(pio, pin0_data + 1); // clk
	//
	sm_config_set_in_shift(&cfg, /*shift_right=*/true, /*autopush=*/true, /*push_threshold=*/32);
	sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX);
	//
	pio_sm_init(pio, sm, offset, &cfg);
}
//
static inline uint clocked_read_program_add_and_get_sm(PIO pio, uint pin0_data)
{
	const uint offset = pio_add_program(pio, &clocked_read_program);
	const uint sm = pio_claim_unused_sm(pio, true);
	clocked_read_program_init(pio, sm, offset, pin0_data);
	return sm;
}

void clocked_read_init(PIO _pio, uint _dma_channel)
{
	pio = _pio;
	dma_channel = _dma_channel;
	sm = clocked_read_program_add_and_get_sm(pio, GPIO_READ_DATA);
}

static void check_buffer_index(unsigned buffer_index)
{
	if (buffer_index >= CLOCKED_READ_BUFFER_COUNT) PANIC(PANIC_BOUNDS_CHECK_FAILED);
}

void clocked_read_into_buffer(unsigned buffer_index, unsigned n_bytes)
{
	if (n_bytes > MAX_DATA_BUFFER_SIZE) n_bytes = MAX_DATA_BUFFER_SIZE;

	check_buffer_index(buffer_index);
	struct buffer* buf = &buffers[buffer_index];
	if (buf->status != BUSY) PANIC(PANIC_UNEXPECTED_STATE);

	pio_sm_set_enabled(pio, sm, false);

	pio_sm_clear_fifos(pio, sm);
	pio_sm_restart(pio, sm);

	dma_channel_config dma_channel_cfg = dma_channel_get_default_config(dma_channel);
	channel_config_set_read_increment(&dma_channel_cfg,  false);
	channel_config_set_write_increment(&dma_channel_cfg, true);
	channel_config_set_dreq(&dma_channel_cfg, pio_get_dreq(pio, sm, false));

	dma_channel_configure(
		dma_channel,
		&dma_channel_cfg,
		buf->data,     // write to buffer
		&pio->rxf[sm], // read from PIO RX FIFO
		BYTES_TO_32BIT_WORDS(n_bytes),
		true // start now!
	);

	pio_sm_set_enabled(pio, sm, true);
}

int clocked_read_is_running(void)
{
	return dma_channel_is_busy(dma_channel);
}

static int find_buffer_index(enum buffer_status with_buffer_status)
{
	for (unsigned i = 0; i < CLOCKED_READ_BUFFER_COUNT; i++) {
		if (buffers[i].status == with_buffer_status) {
			return i;
		}
	}
	return -1;
}

static int get_next_free_buffer_index(void)
{
	return find_buffer_index(FREE);
}

int get_written_buffer_index(void)
{
	return find_buffer_index(WRITTEN);
}

unsigned can_allocate_buffer(void)
{
	return get_next_free_buffer_index() >= 0;
}

unsigned allocate_buffer(unsigned size)
{
	const int i = get_next_free_buffer_index();
	if (i < 0) PANIC(PANIC_ALLOCATION_ERROR);
	check_buffer_index(i);
	struct buffer* buf = &buffers[i];
	buf->status = BUSY;
	if (size > MAX_DATA_BUFFER_SIZE) size = MAX_DATA_BUFFER_SIZE;
	buf->size = size;
	return i;
}

uint8_t* get_buffer_data(unsigned buffer_index)
{
	check_buffer_index(buffer_index);
	return buffers[buffer_index].data;
}

char* get_buffer_filename(unsigned buffer_index)
{
	check_buffer_index(buffer_index);
	return buffers[buffer_index].filename;
}

enum buffer_status get_buffer_status(unsigned buffer_index)
{
	check_buffer_index(buffer_index);
	return buffers[buffer_index].status;
}

void wrote_buffer(unsigned buffer_index)
{
	check_buffer_index(buffer_index);
	if (get_buffer_status(buffer_index) != BUSY) PANIC(PANIC_UNEXPECTED_STATE);
	buffers[buffer_index].status = WRITTEN;
}

void transferred_buffer(unsigned buffer_index)
{
	check_buffer_index(buffer_index);
	if (get_buffer_status(buffer_index) != WRITTEN) PANIC(PANIC_UNEXPECTED_STATE);
	buffers[buffer_index].status = TRANSFERRED;
}

void release_buffer(unsigned buffer_index)
{
	check_buffer_index(buffer_index);
	if (get_buffer_status(buffer_index) != TRANSFERRED) PANIC(PANIC_UNEXPECTED_STATE);
	buffers[buffer_index].status = FREE;
}

unsigned get_buffer_size(unsigned buffer_index)
{
	check_buffer_index(buffer_index);
	return buffers[buffer_index].size;
}

void reset_buffers(void)
{
	for (int i = 0; i < CLOCKED_READ_BUFFER_COUNT; i++) {
		buffers[i].status = FREE;
	}
}
