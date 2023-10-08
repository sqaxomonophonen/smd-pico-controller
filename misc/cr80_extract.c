// cc -O3 -I.. cr80_extract.c -o cr80_extract
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "bits.h"
#include "drive.h"

#define BITS_PER_SECTOR (5028) // CR8044M says 628.5 bytes per sector, 628.5*8=5028
#define DATA_FIELD_BITS (551*8)

int expected_cylinder;

enum {
	SECTOR_OK = 0,
	SECTOR_BAD_GAP_A,
	SECTOR_BAD_ADDRESS_CRC,
	SECTOR_NO_HEAD_SYNC,
	SECTOR_NO_DATA_SYNC,
	SECTOR_BAD_DATA_CRC,
};

#define SYNC "10111001"

static int try_sector(struct bits bits)
{
	if (bits.n == 0 || bits.bd[0] != 0) return SECTOR_BAD_GAP_A;

	// assumption is that we're in GAP-A (see CR8044M doc), which should
	// consist of 31 bytes of zeroes before being terminated by a SYNC byte
	const uint8_t* bd0 = bits.bd;
	while (bits.n > 0 && bits_match_prefix_ascii(bits, "00000")) {
		bits = bits_slice(bits, 1, -1);
	}
	bits = bits_slice(bits, 4, -1);
	const int n_skipped = bits.bd - bd0;

	if (n_skipped < 10) return SECTOR_BAD_GAP_A;

	const int n_addr_bits = 8*9;
	struct bits addr = bits_slice(bits, 0, n_addr_bits);
	const int crc16_addr = bits_crc16(addr);
	if (crc16_addr != 0) {
		return SECTOR_BAD_ADDRESS_CRC;
	}
	if (!bits_match_prefix_ascii(addr, SYNC)) return SECTOR_NO_HEAD_SYNC;
	addr = bits_slice(addr, 8, -1);

	uint16_t cylinder_cf_w = bits_u16(addr);
	addr = bits_slice(addr, 16, -1);

	uint8_t head = bits_u8(addr);
	addr = bits_slice(addr, 8, -1);

	uint8_t sector = bits_u8(addr);
	addr = bits_slice(addr, 8, -1);

	uint16_t auxiliary = bits_u16(addr);
	addr = bits_slice(addr, 16, -1);

	//printf("ADDRESS cyl=%d head=%d sector=%d aux=%d\n", cylinder_cf_w, head, sector, auxiliary);
	if (cylinder_cf_w != expected_cylinder) {
		printf("expected cyl=%d, got ADDRESS cyl=%d head=%d sector=%d aux=%d\n", expected_cylinder, cylinder_cf_w, head, sector, auxiliary);
	}

	bits = bits_slice(bits, n_addr_bits, -1);

	while (bits.n > 0 && bits_match_prefix_ascii(bits, "00000")) {
		bits = bits_slice(bits, 1, -1);
	}
	bits = bits_slice(bits, 4, -1);
	if (!bits_match_prefix_ascii(bits, SYNC)) return SECTOR_NO_DATA_SYNC;

	bits = bits_slice(bits, 0, DATA_FIELD_BITS);

	const int crc16_data = bits_crc16(bits);
	if (crc16_data != 0) {
		return SECTOR_BAD_DATA_CRC;
	}

	return SECTOR_OK;
}

static int sector_gap_a_appears_noisy(struct bits bits)
{
	const int n = 75;
	bits = bits_slice(bits, 0, n);
	const int popcnt = bits_popcnt(bits);
	return 0 < popcnt && popcnt < bits.n;
}

static int sector_gap_a_appears_inverted(struct bits bits)
{
	const int n = 75;
	bits = bits_slice(bits, 0, n);
	return bits_popcnt(bits) == n;
}

static void process_track(struct bits bits)
{
	const int bits_per_track = 8*DRIVE_BYTES_PER_TRACK; // XXX allow a bit of drift?

	int sector = 0;
	for (int offset0 = 0; offset0 < (8*DRIVE_BYTES_PER_TRACK); offset0 += BITS_PER_SECTOR, sector++) {
		int ok = 0;

		{
			// first try if any sector is OK without any attempt at
			// correction
			int rotation = 0;
			for (int offset1 = offset0; offset1 < bits.n; offset1 += bits_per_track, rotation++) {
				struct bits window = bits_slice(bits, offset1, BITS_PER_SECTOR);
				if (sector_gap_a_appears_noisy(window)) {
					continue;
				}
				if (sector_gap_a_appears_inverted(window)) {
					continue;
				}

				int e = try_sector(window);
				if (e == SECTOR_OK) {
					ok = 1;
					break;
				}
			}
		}

		if (!ok) {
			// try "voting"
			int rotation = 0;
			const int MAX_WINDOWS = 256;
			struct bits windows[MAX_WINDOWS];
			int window_sync_offsets[MAX_WINDOWS];
			int wi = 0;
			for (int offset1 = offset0; offset1 < bits.n; offset1 += bits_per_track, rotation++) {
				struct bits window = bits_slice(bits, offset1, BITS_PER_SECTOR);

				if (sector_gap_a_appears_inverted(window)) {
					window = bits_invert(window);
				}

				struct bits find_sync = window;
				while (find_sync.n > 0 && bits_match_prefix_ascii(find_sync, "00000")) {
					find_sync = bits_slice(find_sync, 1, -1);
				}
				window_sync_offsets[wi] = find_sync.bd - window.bd;

				if (window.n < BITS_PER_SECTOR/2) continue;
				windows[wi] = window;
				wi++;
			}

			if (wi >= 2) {
				int min_off = 0;
				for (int i = 0; i < wi; i++) {
					int off = window_sync_offsets[i];
					if (i == 0 || off < min_off) min_off = off;
				}
				for (int i = 0; i < wi; i++) {
					windows[i] = bits_slice(windows[i], window_sync_offsets[i] - min_off, -1);
					//printf("  w[%d]", i);bits_dump(windows[i]);printf("\n");
				}
				struct bits best_guess = bits_vote(windows, wi);
				//printf("  w[X]");bits_dump(best_guess);printf("\n");
				int e = try_sector(best_guess);
				if (e == SECTOR_OK) {
					//printf("FIXED?\n");
					ok = 2;
				}
			}
		}

		//printf("SECTOR %.2d: %s\n", sector, ok == 2 ? "OK(VOTE)" : ok == 1 ? "OK" : "FAIL");
	}
}

int main(int argc, char** argv)
{
	if (argc != 3) {
		fprintf(stderr, "Usage: %s <dump.nrz> \n", argv[0]);
		exit(EXIT_FAILURE);
	}

	expected_cylinder = atoi(argv[2]);

	process_track(bits_load_lsb_first(argv[1]));

	return EXIT_SUCCESS;
}
