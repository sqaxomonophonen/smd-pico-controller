//#define LOOPBACK_TEST
#define DIAGNOSTICS
#define TELEMETRY_LOG

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>

#include <SDL.h>
#include <SDL_opengl.h>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl2.h"

#define STB_DS_IMPLEMENTATION
#include "stb_ds.h"

#include "controller_protocol.h"
#define COMMAND(NAME,ARGFMT) const char* CMDSTR_##NAME = #NAME;
EMIT_COMMANDS
#undef COMMAND
#include "pin_config.h"

#include "drive.h"
#include "base64.h"
void PANIC(uint32_t error) { fprintf(stderr, "PANIC(%d)\n", error); abort(); } // heh
#include "base64.c" // eheheh
#include "adler32.h"
#include "adler32.c" // ;-)

struct cond {
	int value;
	pthread_cond_t pt_cond;
	pthread_mutex_t pt_mutex;
};

static void cond_init(struct cond* cond)
{
	memset(cond, 0, sizeof *cond);
	assert(pthread_cond_init(&cond->pt_cond, NULL) == 0);
	assert(pthread_mutex_init(&cond->pt_mutex, NULL) == 0);
}

static void cond_wait_nonzero(struct cond* cond)
{
	pthread_mutex_lock(&cond->pt_mutex);
	while (cond->value == 0) pthread_cond_wait(&cond->pt_cond, &cond->pt_mutex);
	pthread_mutex_unlock(&cond->pt_mutex);
}

static void cond_signal_value(struct cond* cond, int new_value)
{
	pthread_mutex_lock(&cond->pt_mutex);
	cond->value = new_value;
	pthread_cond_signal(&cond->pt_cond);
	pthread_mutex_unlock(&cond->pt_mutex);
}

static void cond_signal(struct cond* cond)
{
	cond_signal_value(cond, 1);
}

struct controller_status {
	int64_t timestamp_us;
	uint32_t status;
};

char* nrz_analyzer_path;

enum nrz_analyzer_state {
	NA_IDLE = 0,
	NA_NEW,
	NA_RUNNING,
	NA_DONE,
};

struct nrz_analyzer_entry {
	uint32_t ts_ticks;
	int integrity0;
	int integrity1;
	int cylinder;
};

struct nrz_analyzer_shared {
	pthread_mutex_t mutex;
	pthread_cond_t cond;
	char nrz_path[1<<12];
	enum nrz_analyzer_state state;
	char stdout_buf[1<<20];
	ssize_t stdout_n_bytes;
	struct nrz_analyzer_entry entry;
} *nrz_analyzer_shared;

struct nrz_analyzer_entry* nrz_analyzer_log = NULL;

struct com_file {
	int in_use;
	int fd;
	int sequence;
	size_t bytes_written;
	size_t bytes_total;
	unsigned buffer_index;
	struct adler32 adler;
	int n_non_zero_bytes;
};

#define MAX_FREQUNCIES (4)
struct com {
	int fd;
	char* tty_path;
	char* recv_line_arr;
	pthread_mutex_t queue_mutex;
	char** queue_arr;

	pthread_rwlock_t rwlock;
	char** controller_log;
	struct controller_status* controller_status_arr;
	uint64_t controller_timestamp_us;
	uint32_t frequencies[MAX_FREQUNCIES];

	struct com_file file;
	int file_serial;

	bool log_status_changes = false;

	#ifdef TELEMETRY_LOG
	FILE* telemetry_log_file;
	#endif
} com;


#ifdef TELEMETRY_LOG
__attribute__((format(printf, 1, 2)))
static void telemetry_log(const char* fmt, ...)
{
	if (com.telemetry_log_file == NULL) return;

	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	const time_t t = ts.tv_sec;
	const struct tm* tmp = localtime(&t);
	char buf[1<<12];
	strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", tmp);
	FILE* f = com.telemetry_log_file;
	int fractional =  (int)((double)ts.tv_nsec * 1e-5);
	fprintf(f, "%s.%.4d  ", buf, fractional);
	va_list ap;
	va_start(ap, fmt);
	vfprintf(f, fmt, ap);
	va_end(ap);
	fprintf(f, "\n");
	fflush(com.telemetry_log_file);
}

int telemetry_pending = 0;
int current_controls = 0;
int last_controls = 0;
int current_st = 0;
int last_st = 0;

static void telemetry_log_status(void)
{
	if (com.telemetry_log_file == NULL) return;

	// mask out unwanted status changes (unwanted in telemetry.log)
	current_st &= ~( (1<<0) | (1<<1) | (1<<6));

	if (current_controls == last_controls && current_st == last_st) return;
	telemetry_pending = 0;

	// XXX check this, I think FA(ULT) has become O(n)C(ylinder), but it
	// also looks rather fragile

	#define RC(x) \
		((current_controls&(1<<(CONTROL_ ## x))) != (last_controls&(1<<(CONTROL_ ## x)))) ? '>' : ':', \
		(current_controls&(1<<(CONTROL_ ## x))) ? '*' : '.'
	#define RS(x) \
		((current_st&(1<<(x))) != (last_st&(1<<(x)))) ? '>' : ':', \
		(current_st&(1<<(x))) ? '*' : '.'
	telemetry_log(
		" TU%c%c"
		" T1%c%c"
		" T2%c%c"
		" T3%c%c"
		" B0%c%c"
		" B1%c%c"
		" B2%c%c"
		" B3%c%c"
		" B4%c%c"
		" B5%c%c"
		" B6%c%c"
		" B7%c%c"
		" B8%c%c"
		" B9%c%c"
		" |"
		" FA%c%c"
		" SR%c%c"
		" OC%c%c"
		" UR%c%c"
		" US%c%c"
		" SE%c%c"
	,
		RC(UNIT_SELECT_TAG),
		RC(TAG1), RC(TAG2), RC(TAG3),
		RC(BIT0), RC(BIT1), RC(BIT2), RC(BIT3), RC(BIT4), RC(BIT5), RC(BIT6), RC(BIT7), RC(BIT8), RC(BIT9),
		RS(2), RS(3), RS(4), RS(5), RS(7), RS(8)
	);
	#undef RS
	#undef RC

	last_controls = current_controls;
	last_st = current_st;
}
#endif

static int starts_with(char* s, const char* prefix)
{
	const int prefix_length = strlen(prefix);
	if (prefix_length > strlen(s)) return 0;
	for (int i = 0; i < prefix_length; i++) {
		if (s[i] != prefix[i]) return 0;
	}
	return 1;
}

static int is_payload(char* s, const char* cppp, char** tail)
{
	size_t ncppp = strlen(cppp);
	if (tail) *tail = s + ncppp;
	return starts_with(s, cppp) && (s[ncppp] == ' ' || s[ncppp] == 0);
}

static char* duplicate_string(char* s) // strdup() is deprecated?
{
	const size_t sz = strlen(s)+1;
	void* p = malloc(sz);
	memcpy(p, s, sz);
	return (char*)p;
}

static void end_com_file(void)
{
	struct com_file* cf = &com.file;
	if (!cf->in_use) return;
	close(cf->fd);
	memset(cf, 0, sizeof *cf);
}

__attribute__((format(printf, 1, 2)))
static void com_printf(const char* fmt, ...)
{
	char buf[1<<16];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(buf, sizeof buf, fmt, ap);
	va_end(ap);
	char* msg = (char*)malloc(n+1);
	memcpy(msg, buf, n+1);
	arrput(com.controller_log, msg);
}

static void bad_msg(char* msg)
{
	com_printf("WARNING: garbage message from controller: [%s]", msg);
}

static void com__handle_msg(char* msg)
{
	struct com_file* comfile = &com.file;
	char* tail = NULL;
	if (starts_with(msg, CPPP_LOG)) {
		printf("(CTRL) %s\n", msg);
		msg = duplicate_string(msg);
		pthread_rwlock_wrlock(&com.rwlock);
		arrput(com.controller_log, msg);
		pthread_rwlock_unlock(&com.rwlock);
	} else if (is_payload(msg, CPPP_FREQ, &tail)) {
		uint32_t num = 0, value = 0;
		if (sscanf(tail, " %u %u", &num, &value) == 2) {
			if (0 <= num && num < MAX_FREQUNCIES) {
				com.frequencies[num] = value * FREQ_FREQ_HZ;
			} else {
				bad_msg(msg);
			}
		} else {
			bad_msg(msg);
		}
	} else if (is_payload(msg, CPPP_STATUS, &tail)) {
		int64_t timestamp_us = 0;
		uint32_t status = 0;
		if (sscanf(tail, " %ld %u", &timestamp_us, &status) == 2) {
			struct controller_status s;
			s.timestamp_us = timestamp_us;
			s.status = status;
			pthread_rwlock_wrlock(&com.rwlock);
			arrput(com.controller_status_arr, s);
			if (timestamp_us > com.controller_timestamp_us) {
				com.controller_timestamp_us = timestamp_us;
			}
			if (com.log_status_changes) {
				com_printf("STAT t=%lu st=%d", s.timestamp_us, s.status);
			}
			pthread_rwlock_unlock(&com.rwlock);
		} else {
			bad_msg(msg);
		}
	} else if (is_payload(msg, CPPP_TIME, &tail)) {
		int64_t timestamp_us;
		if (sscanf(tail, " %ld", &timestamp_us) == 1) {
			pthread_rwlock_wrlock(&com.rwlock);
			if (timestamp_us > com.controller_timestamp_us) com.controller_timestamp_us = timestamp_us;
			pthread_rwlock_unlock(&com.rwlock);
		} else {
			bad_msg(msg);
		}
	} else if (is_payload(msg, CPPP_DATA_HEADER, &tail)) {
		int n_bytes = -1;
		char filename[1<<10];
		unsigned buffer_index = 0;
		if (sscanf(tail, " %u %d %s", &buffer_index, &n_bytes, filename) == 2) {
			assert(!comfile->in_use);
			char other_filename[1<<11];
			char* path = filename;
			for (;;) {
				// XXX reuse same file / truncate if retrying a download?
				int fd = open(path, O_CREAT | O_EXCL | O_WRONLY, 0644);
				if (fd == -1) {
					if (errno == EEXIST) {
						snprintf(
							other_filename, sizeof other_filename,
							"%s-resolv%d", filename, rand());
						path = other_filename;
						continue;
					} else {
						fprintf(stderr, "%s: %s\n", path, strerror(errno));
						exit(EXIT_FAILURE);
					}
				}
				memset(comfile, 0, sizeof *comfile);
				comfile->in_use = 1;
				comfile->fd = fd;
				comfile->bytes_total = n_bytes;
				comfile->buffer_index = buffer_index;
				adler32_init(&comfile->adler);
				com_printf("D/L %d bytes [%s]...", n_bytes, path);
				telemetry_log("beginning to download %d bytes...", n_bytes);
				comfile->n_non_zero_bytes = 0;
				break;
			}
		} else {
			bad_msg(msg);
		}
	} else if (is_payload(msg, CPPP_DATA_LINE, &tail)) {
		if (!comfile->in_use) {
			com_printf("ERROR: out of sequence (not-in-use) data line [%s]", msg);
		} else {
			int sequence = -1;
			char b64[1<<10];
			if (sscanf(tail, " %d %s", &sequence, b64) == 2) {
				if (sequence != comfile->sequence) {
					com_printf("ERROR: out of sequence (expected %d, got %d) data line [%s]", comfile->sequence, sequence, msg);
					end_com_file();
					return;
				} else {
					comfile->sequence++;
					uint8_t buffer[1<<10];
					uint8_t* eb = base64_decode_line(buffer, b64);
					if (eb == NULL) {
						com_printf("ERROR: could not decode data line [%s]", msg);
						end_com_file();
						return;
					} else {
						const size_t n_recv = eb - buffer;
						adler32_push(&comfile->adler, buffer, n_recv);
						for (size_t i = 0; i < n_recv; i++) if (buffer[i] != 0) comfile->n_non_zero_bytes++;
						comfile->bytes_written += n_recv;
						size_t remaining = n_recv;
						uint8_t* tp = buffer;
						while (remaining > 0) {
							ssize_t nw = write(comfile->fd, tp, remaining);
							if (nw == -1) {
								if (errno == EINTR) {
									continue;
								} else {
									assert(!"write error");
								}
							}
							tp += nw;
							remaining -= nw;
						}
					}
				}
			} else {
				bad_msg(msg);
				end_com_file();
			}
		}
	} else if (is_payload(msg, CPPP_DATA_FOOTER, &tail)) {
		if (!comfile->in_use) {
			com_printf("WARNING: out of sequence (not-in-use) footer [%s]", msg);
		} else {
			int sequence = -1;
			uint32_t pico_checksum = 0;
			if (sscanf(tail, " %d %u", &sequence, &pico_checksum) == 2) {
				const uint32_t our_checksum = adler32_sum(&comfile->adler);
				if (comfile->bytes_written != comfile->bytes_total) {
					com_printf("ERROR: expected %zd bytes; only received %zd", comfile->bytes_total, comfile->bytes_written);
					end_com_file();
					return;
				}
				if (our_checksum != pico_checksum) {
					com_printf("ERROR: bad checksum; pico says %u; our calc says %u", pico_checksum, our_checksum);
					end_com_file();
					return;
				}
				if (sequence != comfile->sequence) {
					com_printf("ERROR: out of sequence (expected %d, got %d) footer [%s]", comfile->sequence, sequence, msg);
					end_com_file();
					return;
				}
				close(comfile->fd);
				if (comfile->n_non_zero_bytes == 0) {
					com_printf("WARNING: downloaded file contains only zeroes");
					telemetry_log("download done (all zeroes!)");
				} else {
					telemetry_log("download done");
				}
				com.file_serial++;
				comfile->in_use = 0;
			} else {
				bad_msg(msg);
				end_com_file();
			}
		}
	} else {
		bad_msg(msg);
	}
}

static void com_recv_char(char ch)
{
	if (ch == '\r' || ch == '\n') {
		if (arrlen(com.recv_line_arr) > 0) {
			arrput(com.recv_line_arr, 0);
			com__handle_msg(com.recv_line_arr);
			arrsetlen(com.recv_line_arr, 0);
		}
	} else {
		arrput(com.recv_line_arr, ch);
	}
}

__attribute__((format(printf, 1, 2)))
static void com_enqueue(const char* fmt, ...)
{
	char buf[1<<16];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(buf, sizeof buf, fmt, ap);
	va_end(ap);
	char* s = (char*)malloc(n+1);
	memcpy(s, buf, n+1);
	pthread_mutex_lock(&com.queue_mutex);
	arrput(com.queue_arr, s);
	pthread_mutex_unlock(&com.queue_mutex);
}

// caller should free return value when done with it
static char* com_shift(void)
{
	char* c = NULL;
	pthread_mutex_lock(&com.queue_mutex);
	if (arrlen(com.queue_arr) > 0) {
		c = com.queue_arr[0];
		arrdel(com.queue_arr, 0);
	}
	pthread_mutex_unlock(&com.queue_mutex);
	return c;
}

static int com_has_pending_writes(void)
{
	pthread_mutex_lock(&com.queue_mutex);
	int p = arrlen(com.queue_arr) > 0;
	pthread_mutex_unlock(&com.queue_mutex);
	return p;
}

void* io_thread_start(void* arg)
{
	assert(com.fd >= 0);

	struct timeval timeout = {0};

	com_enqueue("%s 1", CMDSTR_subscribe_to_status);

	for (;;) {
		pthread_mutex_lock(&nrz_analyzer_shared->mutex);
		if (nrz_analyzer_shared->state == NA_DONE) {
			//nrz_analyzer_shared->entry.ts_ticks = SDL_GetTicks();
			arrput(nrz_analyzer_log, nrz_analyzer_shared->entry);
			nrz_analyzer_shared->state = NA_IDLE;
		}
		pthread_mutex_unlock(&nrz_analyzer_shared->mutex);

		fd_set rfds, wfds;

		FD_ZERO(&rfds);
		FD_SET(com.fd, &rfds);

		FD_ZERO(&wfds);
		if (com_has_pending_writes()) {
			FD_SET(com.fd, &wfds);
		}

		memset(&timeout, 0, sizeof timeout);
		timeout.tv_sec = 0;
		timeout.tv_usec = 1000000/30;
		int r = select(com.fd+1, &rfds, &wfds, NULL, &timeout);
		if (r == -1) {
			fprintf(stderr, "select(): %s\n", strerror(errno));
			exit(EXIT_FAILURE);
		} else if (r == 0) {
			continue;
		}

		assert(r > 0);

		if (FD_ISSET(com.fd, &rfds)) {
			char buf[1<<16];
			int n = read(com.fd, buf, sizeof buf);
			if (n == -1) {
				fprintf(stderr, "tty: %s\n", strerror(errno));
				exit(EXIT_FAILURE);
			}
			for (int i = 0; i < n; i++) com_recv_char(buf[i]);
		}

		if (FD_ISSET(com.fd, &wfds)) {
			char* c = com_shift();
			assert((c != NULL) && "expected to shift command");
			size_t n = strlen(c);
			assert(write(com.fd, c, n) != -1);
			free(c);
			assert(write(com.fd, "\r\n", 2) != -1); // XXX didn't I switch to "\n"-only?
		}
	}
	return NULL;
}

static void com_startup(char* tty_path)
{
	com.tty_path = tty_path;
	com.fd = open(com.tty_path, O_RDWR | O_NOCTTY);
	if (com.fd == -1) {
		fprintf(stderr, "%s: %s\n", com.tty_path, strerror(errno));
		exit(EXIT_FAILURE);
	}

	if (!isatty(com.fd)) {
		fprintf(stderr, "%s: not a tty\n", com.tty_path);
		assert(close(com.fd) == 0);
		exit(EXIT_FAILURE);
	}

	if (flock(com.fd, LOCK_EX | LOCK_NB) == -1) {
		if (errno == EWOULDBLOCK) {
			fprintf(stderr, "%s: already locked by another process\n", com.tty_path);
		} else {
			fprintf(stderr, "%s: %s\n", com.tty_path, strerror(errno));
		}
		exit(EXIT_FAILURE);
	}

	{
		struct termios t;
		tcgetattr(com.fd, &t);
		t.c_lflag &= ~(ICANON | ECHO);
		tcsetattr(com.fd, TCSANOW, &t);
	}

	pthread_mutex_init(&com.queue_mutex, NULL);
	pthread_t io_thread;
	assert(pthread_create(&io_thread, NULL, io_thread_start, NULL) == 0);

	assert(pthread_rwlock_init(&com.rwlock, NULL) == 0);

	printf("COM: ready!\n");
}

static void com_shutdown(void)
{
	#ifdef TELEMETRY_LOG
	if (com.telemetry_log_file != NULL) {
		telemetry_log("END");
		fclose(com.telemetry_log_file);
	}
	#endif
	if (flock(com.fd, LOCK_UN) == -1) {
		fprintf(stderr, "%s: %s\n", com.tty_path, strerror(errno));
		exit(EXIT_FAILURE);
	}
	assert(close(com.fd) == 0); // XXX hangs... is it because the other end has to "ACK" the close? maybe?
}

__attribute__ ((noreturn))
static void SDL2FATAL(void)
{
	fprintf(stderr, "SDL2: %s\n", SDL_GetError());
	exit(EXIT_FAILURE);
}

static ImU32 get_status_label_color(const char* label, int st)
{
	//INDEX SECTOR FAULT SEEK_ERROR ON_CYLINDER UNIT_READY ADDRESS_MARK UNIT_SELECTED SEEK_END
	double r,g,b;
	if (strcmp(label, "FAULT") == 0 || strcmp(label, "SEEK_ERROR") == 0) {
		r = 0.8;
		g = 0.2;
		b = 0;
	} else if (strcmp(label, "ON_CYLINDER") == 0 || strcmp(label, "UNIT_READY") == 0 || strcmp(label, "UNIT_SELECTED") == 0 || strcmp(label, "SEEK_END") == 0) {
		r = 0.2;
		g = 0.5;
		b = 0;
	} else {
		r = 0.5;
		g = 0.6;
		b = 1.0;
	}

	double m = 1.0;
	if (st == 0) {
		m = 0.2;
	} else if (st == 1) {
		m = 1.0;
	} else {
		assert(!"UNREACHABLE");
	}
	return ImGui::GetColorU32(ImVec4(r*m, g*m, b*m, 1.0));
}

static void push_danger_style(void)
{
	const float r = 0.8;
	const float g = 0.0;
	const float b = 0.2;
	const float a0 = 0.2;
	const float a1 = 0.5;
	ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(r,g,b,1));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(r+a0,g+a0,b+a0,1));
	ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(r+a1,g+a1,b+a1,1));
}

static void pop_danger_style(void)
{
	ImGui::PopStyleColor(3);
}

const char* prg;

__attribute__ ((noreturn))
static void usage(int exit_status)
{
	//                         1         2         3         4         5         6         7
	//               01234567890123456789012345678901234567890123456789012345678901234567890123456789
	fprintf(stderr, "\n");
	fprintf(stderr, "Usage: %s </path/to/tty/for/smd-pico-controller> [-p <px>] [-x </path/to/exe>]\n", prg);
	fprintf(stderr, "\n");
	fprintf(stderr, "Try `/dev/ttyACM0`, or run `dmesg` or `ls -ltr /dev/` to see/guess what tty is\n");
	fprintf(stderr, "assigned to the device (after plugging it in). You can also pass an empty string\n");
	fprintf(stderr, "as path to test the GUI.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, " -p <font size px>\n");
	fprintf(stderr, "    Override font size\n");
	fprintf(stderr, "\n");
	fprintf(stderr, " -x <path to nrz analyzer / head alignment executable>\n");
	fprintf(stderr, "    Enables NRZ analyzer / head alignment functionality via an executable provided\n");
	fprintf(stderr, "    by you. It must take 1 argument; a path to an .nrz file, and must print\n");
	fprintf(stderr, "    key/value pairs to stdout:\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "      integrity0=420\n");
	fprintf(stderr, "      integrity1=666\n");
	fprintf(stderr, "      cylinder=123\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    The integrity values must be between 0 and 1000, where 0 is worst and 1000\n");
	fprintf(stderr, "    is best. `integrity0` should be present, but `integrity1` is optional.\n");
	fprintf(stderr, "    `cylinder` (optional) is the cylinder detected by your analyzer; this is\n");
	fprintf(stderr, "    used for displaying discrepancies between expected/detected cylinder\n");
	fprintf(stderr, "    (CDC 9760/9762 heads can be misaligned by several cylinders).\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    \"Back in the day\" head alignment was typically done with a CDC FTU\n");
	fprintf(stderr, "    (\"Field Test Unit\") and a special \"alignment disk pack\". But if data\n");
	fprintf(stderr, "    disk packs is all you have, you cannot do head alignment without also\n");
	fprintf(stderr, "    understanding the sector format (which hopefully contains checksum and\n");
	fprintf(stderr, "    cylinder) well enough to assess data integrity. The drive itself doesn't\n");
	fprintf(stderr, "    know the sector format, which is why you must provide your own executable.\n");
	fprintf(stderr, "\n");
	exit(exit_status);
}

static pid_t start_analyzer_host(void)
{
	nrz_analyzer_shared = (struct nrz_analyzer_shared*) mmap(
		NULL,
		sizeof *nrz_analyzer_shared,
		PROT_READ | PROT_WRITE,
		MAP_SHARED | MAP_ANONYMOUS,
		-1,
		0);
	if (nrz_analyzer_shared == MAP_FAILED) {
		fprintf(stderr, "mmap() failed\n");
		exit(EXIT_FAILURE);
	}

	assert(pthread_mutex_init(&nrz_analyzer_shared->mutex, NULL) == 0);
	assert(pthread_cond_init(&nrz_analyzer_shared->cond, NULL) == 0);

	pid_t child_pid = fork();
	if (child_pid == 0) {
		// child
		for (;;) {
			// wait for new job and grab it
			pthread_mutex_lock(&nrz_analyzer_shared->mutex);
			while (nrz_analyzer_shared->state != NA_NEW) {
				pthread_cond_wait(&nrz_analyzer_shared->cond, &nrz_analyzer_shared->mutex);
			}
			char* nrz_path = duplicate_string(nrz_analyzer_shared->nrz_path);
			nrz_analyzer_shared->state = NA_RUNNING;
			pthread_mutex_unlock(&nrz_analyzer_shared->mutex);

			// fork again; child executes analyzer; parent reads
			// stdout and waits for exit
			int pair[2];
			assert(pipe(pair) != -1);
			pid_t exec_pid = fork();
			if (exec_pid == 0) {
				// child
				assert(close(0) != -1);
				assert(dup2(pair[1], 1) != -1);
				assert(close(pair[0]) != -1);
				assert(close(pair[1]) != -1);
				char* child_argv[] = {
					nrz_analyzer_path,
					nrz_path,
					NULL,
				};
				execv(nrz_analyzer_path, child_argv);
				assert(!"UNREACHABLE");
			} else {
				// parent
				free(nrz_path);
				assert(close(pair[1]) != -1);
				nrz_analyzer_shared->stdout_n_bytes = read(pair[0], nrz_analyzer_shared->stdout_buf, sizeof nrz_analyzer_shared->stdout_buf);
				assert(nrz_analyzer_shared->stdout_n_bytes != -1);
				int wstatus;
				assert(waitpid(exec_pid, &wstatus, 0) != -1);

				pthread_mutex_lock(&nrz_analyzer_shared->mutex);
				nrz_analyzer_shared->state = NA_DONE;
				pthread_mutex_unlock(&nrz_analyzer_shared->mutex);
			}
		}
		assert(!"UNREACHABLE");
	} else {
		return child_pid;
	}
}


int main(int argc, char** argv)
{
	prg = argv[0];
	if (argc < 2) usage(EXIT_FAILURE);

	int font_size = 18; // overridable with -p <px> argument

	const char* tty_path = NULL;

	{
		int switching = 0;
		for (int argi = 1; argi < argc; argi++) {
			char* arg = argv[argi];
			const int arglen = strlen(arg);
			if (switching == 0) {
				if (arglen >= 2 && arg[0] == '-') {
					if (arg[1] == 'p') {
						switching = 'p';
					} else if (arg[1] == 'x') {
						switching = 'x';
					} else if (arg[1] == 'h') {
						usage(EXIT_SUCCESS);
					} else {
						fprintf(stderr, "invalid switch %s\n", arg);
						usage(EXIT_FAILURE);
					}
				} else {
					if (tty_path == NULL) {
						tty_path = arg;
					} else {
						fprintf(stderr, "only one non-positional arg (tty path) allowed\n");
						usage(EXIT_FAILURE);
					}
				}
			} else if (switching == 'p') {
				font_size = atoi(arg);
				if (font_size < 1) {
					fprintf(stderr, "invalid font size %s\n", arg);
					usage(EXIT_FAILURE);
				}
				switching = 0;
			} else if (switching == 'x') {
				nrz_analyzer_path = arg;
				struct stat st;
				if (stat(nrz_analyzer_path, &st) == -1) {
					fprintf(stderr, "%s: %s\n", nrz_analyzer_path, strerror(errno));
					exit(EXIT_FAILURE);
				}
				if (!(st.st_mode & S_IXUSR) || (st.st_mode & S_IFDIR)) {
					fprintf(stderr, "%s: not an exectuable\n", nrz_analyzer_path);
					exit(EXIT_FAILURE);
				}
				switching = 0;
			} else {
				assert(!"UNREACHABLE");
			}
		}
		if (switching != 0) {
			fprintf(stderr, "switch -%c requires an argument\n", switching);
			usage(EXIT_FAILURE);
		}
	}

	pid_t nrz_analyzer_child_pid = nrz_analyzer_path != NULL ? start_analyzer_host() : 0;

	#ifdef TELEMETRY_LOG
	com.telemetry_log_file = fopen("telemetry.log", "a");
	assert((com.telemetry_log_file != NULL) && "failed to open telemetry.log for appending");
	telemetry_log("BEGIN");
	#endif

	const int has_com = strcmp(argv[1], "") != 0;
	if (has_com) com_startup(argv[1]);

	if (SDL_Init(SDL_INIT_VIDEO) != 0) SDL2FATAL();

	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);

	SDL_GL_SetAttribute(SDL_GL_RED_SIZE,     8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE,   8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE,    8);
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE,   0);

	SDL_Window* window = SDL_CreateWindow(
		"spcfront",
		SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
		1920, 1080,
		SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);

	SDL_GLContext glctx = SDL_GL_CreateContext(window);

	SDL_GL_MakeCurrent(window, glctx);
	SDL_GL_SetSwapInterval(1);

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	ImGui::StyleColorsDark();

	ImGui_ImplSDL2_InitForOpenGL(window, glctx);
	ImGui_ImplOpenGL2_Init();

	float status_scope_scale = 7.5;

	bool previous_debug_led = false,     debug_led = false;
	int previous_debug_control_pins = 0, debug_control_pins = 0;
	int basic_selected_index = 0;
	int basic_cylinder = 0;
	bool basic_cylinder_allow_overflow = false;
	int basic_head = 0;
	bool basic_index_sync  = true;
	bool basic_skip_checks = false;
	int batch_cylinder0 = 0;
	int batch_cylinder1 = 822;
	int batch_head_set = 31;
	int common_byte_count = MAX_DATA_BUFFER_SIZE;
	int common_servo_offset = 0;
	int common_data_strobe_delay = 0;
	bool poll_gpio = false;
	uint32_t last_poll_gpio = 0;
	int broken_seek_cylinder = 0;
	bool continuous_read = false;
	int continuous_read_serial = 0;

	int max_status_txt_width = 0;

	ImFont* font = io.Fonts->AddFontFromFileTTF("Inconsolata-Medium.ttf", font_size);
	io.Fonts->Build();

	int exiting = 0;
	while (!exiting) {

		// XXX does this belong in com thread?
		#if 0
		pthread_mutex_lock(&nrz_analyzer_shared->mutex);
		if (nrz_analyzer_shared->state == NA_DONE) {
			nrz_analyzer_shared->entry.ts_ticks = SDL_GetTicks();
			arrput(nrz_analyzer_log, nrz_analyzer_shared->entry);
			nrz_analyzer_shared->state = NA_IDLE;
		}
		pthread_mutex_unlock(&nrz_analyzer_shared->mutex);
		#endif

		SDL_Event ev;
		while (SDL_PollEvent(&ev)) {
			if ((ev.type == SDL_QUIT) || (ev.type == SDL_WINDOWEVENT && ev.window.event == SDL_WINDOWEVENT_CLOSE)) {
				exiting = 1;
			} else {
				ImGui_ImplSDL2_ProcessEvent(&ev);
			}
		}

		ImGui_ImplOpenGL2_NewFrame();
		ImGui_ImplSDL2_NewFrame();
		ImGui::NewFrame();

		pthread_rwlock_rdlock(&com.rwlock);

		if (nrz_analyzer_path != NULL) {
			ImGui::Begin("NRZ Analyzer / Head Alignment");
			ImGui::End();
		}

		#ifdef LOOPBACK_TEST
		{
			ImGui::Begin("LOOPBACK TEST");
			if (ImGui::Button("Read 8k")) {
				com_enqueue("%s 8192 0 1", CMDSTR_op_read_data);
			}
			ImGui::SameLine();
			if (ImGui::Button("Transmit Data")) {
				com_enqueue("%s 10000", CMDSTR_loopback_test);
			}
			ImGui::End();
		}
		#endif

		#ifdef DIAGNOSTICS
		{
			ImGui::Begin("DIAGNOSTICS");

			ImGui::SeparatorText("Output pins");
			ImGui::CheckboxFlags("UNIT SELECT TAG", &debug_control_pins,                 0x000001);
			ImGui::CheckboxFlags("TAG1 (seek)", &debug_control_pins,                     0x000020);
			ImGui::CheckboxFlags("TAG2 (head)", &debug_control_pins,                     0x000040);
			ImGui::CheckboxFlags("TAG3 (control)", &debug_control_pins,                  0x000080);
			ImGui::CheckboxFlags("BIT0 (/write gate)", &debug_control_pins,              0x000100);
			ImGui::CheckboxFlags("BIT1 (/read gate)", &debug_control_pins,               0x000200);
			ImGui::CheckboxFlags("BIT2 (/servo offset positive)", &debug_control_pins,   0x000400);
			ImGui::CheckboxFlags("BIT3 (/servo offset negative)", &debug_control_pins,   0x000800);
			ImGui::CheckboxFlags("BIT4 (/controller fault clear)", &debug_control_pins,  0x001000);
			ImGui::CheckboxFlags("BIT5 (/address mark enable)", &debug_control_pins,     0x002000);
			ImGui::CheckboxFlags("BIT6 (/return to zero)", &debug_control_pins,          0x004000);
			ImGui::CheckboxFlags("BIT7 (/data strobe early)", &debug_control_pins,       0x008000);
			ImGui::CheckboxFlags("BIT8 (/data strobe late)", &debug_control_pins,        0x010000);
			ImGui::CheckboxFlags("BIT9 (/dual channel release)", &debug_control_pins,    0x020000);

			ImGui::Checkbox("*LED", &debug_led);
			if (ImGui::Button("Set all")) {
				int mask = 1;
				#define CONTROL(NAME,SUPPORTED) \
					if (SUPPORTED) debug_control_pins |= mask; \
					mask <<= 1;
				EMIT_CONTROLS
				#undef CONTROL
				debug_led = 1;
			}
			if (ImGui::Button("Clear all")) {
				debug_control_pins = 0;
				debug_led = 0;
			}

			const int bits_shift = 8; // hehe
			int tmp_bits = debug_control_pins >> bits_shift;
			ImGui::Text("Bits:");
			ImGui::SetNextItemWidth(150);
			ImGui::InputInt("##bits value", &tmp_bits);
			tmp_bits &= ((1<<10)-1);
			debug_control_pins = (debug_control_pins & ((1<<bits_shift)-1)) | (tmp_bits << bits_shift);

			ImGui::SeparatorText("Ops");
			if (ImGui::Button("Read data (no checks)")) {
				com_enqueue("%s %d %d %d", CMDSTR_op_read_data, MAX_DATA_BUFFER_SIZE, /*index_sync=*/1, /*skip_checks=*/0);
			}
			ImGui::SameLine();
			if (ImGui::Checkbox("Continuous", &continuous_read) && continuous_read) {
				com_enqueue("%s %d %d %d", CMDSTR_op_read_data, MAX_DATA_BUFFER_SIZE, /*index_sync=*/1, /*skip_checks=*/0);
			}
			if (continuous_read && com.file_serial > continuous_read_serial) {
				com_enqueue("%s %d %d %d", CMDSTR_op_read_data, MAX_DATA_BUFFER_SIZE, /*index_sync=*/1, /*skip_checks=*/0);
				continuous_read_serial = com.file_serial;
			}

			ImGui::SameLine();
			if (ImGui::Button("Proper Batch Read")) {
				com_enqueue("%s %d %d %d %d %d %d",
					CMDSTR_op_read_batch,
					0,
					DRIVE_CYLINDER_COUNT-1,
					((1 << DRIVE_HEAD_COUNT)-1),
					MAX_DATA_BUFFER_SIZE,
					ENTIRE_RANGE,
					ENTIRE_RANGE);

			}

			ImGui::SameLine();
			if (ImGui::Button("Reset")) {
				com_enqueue("%s", CMDSTR_op_reset);
			}
			ImGui::SetItemTooltip("Clears cylinder register; clears FAULT; executes a RTZ");

			ImGui::InputInt("##broken seek", &broken_seek_cylinder);
			if (broken_seek_cylinder < 0) broken_seek_cylinder = 0;
			if (broken_seek_cylinder >= DRIVE_CYLINDER_COUNT) broken_seek_cylinder = DRIVE_CYLINDER_COUNT-1;
			ImGui::SameLine();
			if (ImGui::Button("Broken Seek")) {
				com_enqueue("%s %d", CMDSTR_op_broken_seek, broken_seek_cylinder);
			}

			#ifdef TELEMETRY_LOG
			ImGui::SeparatorText("Write to telemetry.log");
			static char telemtry_log_message[1<<10] = "";
			if (ImGui::Button("LOG:")) {
				telemetry_log("%s", telemtry_log_message);
			}
			ImGui::SameLine();
			ImGui::InputText("##logmsg", telemtry_log_message, IM_ARRAYSIZE(telemtry_log_message));
			if (ImGui::Button("head move (canned log)")) {
				telemetry_log("head move");
			}
			ImGui::SameLine();
			if (ImGui::Button("fault lamp")) {
				telemetry_log("fault lamp");
			}
			#endif

			ImGui::End();
		}
		#endif

		{ // controller status window
			ImGui::Begin("Controller Status");
			const int64_t now_us = com.controller_timestamp_us;

			ImGui::Text("Uptime: %.1fs", (double)now_us * 1e-6);
			int fi = 0;
			const int FW = 160;
			#define PIN(TYPE,NAME,GPN) \
				if (TYPE==FREQ) { \
					ImGui::SameLine(FW+fi*FW); \
					ImGui::Text(#NAME ": %uhz", com.frequencies[fi++]); \
				}
			EMIT_PIN_CONFIG
			#undef PIN

			ImGui::SliderFloat("scale", &status_scope_scale, 1.0f, 60.0f, "%.1f seconds");

			#define MAX_NAMES (30)
			const char* status_names[MAX_NAMES];

			int n_status_names = 0;
			#define PIN(TYPE,NAME,GPN)                              \
				if (TYPE==STATUS) {                             \
					status_names[n_status_names++] = #NAME; \
				}
			EMIT_PIN_CONFIG
			#undef PIN

			const int n_columns = 2;
			if (ImGui::BeginTable("table", n_columns)) {
				ImGui::TableSetupColumn("0", ImGuiTableColumnFlags_WidthStretch);
				ImGui::TableSetupColumn("1", ImGuiTableColumnFlags_WidthFixed);
				const struct controller_status* cs = com.controller_status_arr;
				const int ncs = arrlen(cs);
				for (int row = 0; row < n_status_names; row++) {
					const unsigned mask = 1 << row;
					const char* label = status_names[row];
					ImU32 st0col = get_status_label_color(label, 0);
					ImU32 st1col = get_status_label_color(label, 1);

					ImGui::TableNextRow();
					ImGui::TableSetColumnIndex(0);

					int eps_count = 0;
					if (ncs > 0) {
						const int64_t ts_horizon = now_us - (int64_t)(status_scope_scale * 1e6f);
						ImVec2 area_p0 = ImGui::GetCursorScreenPos();
						ImVec2 area_sz = ImGui::GetContentRegionAvail();
						ImDrawList* draw_list = ImGui::GetWindowDrawList();
						float x_right = area_p0.x+area_sz.x;
						for (int cursor = ncs-1; cursor >= 0 && draw_list->_VtxCurrentIdx < (1<<15); cursor--) {
							const struct controller_status* st = &cs[cursor];
							const double q = (double)(st->timestamp_us - ts_horizon) / (double)(now_us - ts_horizon);
							const int on = (st->status & mask) != 0;
							const int prev_on = cursor == 0 ? 0 : (cs[cursor-1].status & mask) != 0;
							const int edge = (on != prev_on);
							const float x_left = area_p0.x + area_sz.x*q;
							if (edge && (now_us - st->timestamp_us) < 1000000) {
								eps_count++;
							}
							if (on && edge) { // 1->0
								draw_list->AddRectFilled(
									ImVec2(x_left, area_p0.y),
									ImVec2(x_right, area_p0.y+font_size),
									st1col);
							}
							if (!on && edge) { // 0->1
								x_right = x_left;
							}
							if (x_right < area_p0.x) break;
						}
					}

					ImGui::TableSetColumnIndex(1);
					const int is_on = (ncs > 0) && (cs[ncs-1].status & mask);
					ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, is_on ? st1col : st0col);
					{
						char txt[1<<12];
						int w = eps_count > 0
							? snprintf(txt, sizeof txt, "%s (%dEPS)", label, eps_count)
							: snprintf(txt, sizeof txt, "%s", label);
						assert(0 < w && w < sizeof(txt));
						while (w < max_status_txt_width) txt[w++] = ' ';
						txt[w] = 0;
						if (w > max_status_txt_width) max_status_txt_width = w;
						ImGui::Text("%s", txt);
					}
				}
				ImGui::EndTable();
			}

			ImGui::End();
		}

		{ // controller log window
			ImGui::Begin("Controller Log");

			ImGuiListClipper clipper;
			const int n = arrlen(com.controller_log);
			clipper.Begin(n);
			while (clipper.Step()) {
				for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; i++) {
					char* entry = com.controller_log[i];
					ImGui::TextUnformatted(entry);
				}
			}
			clipper.End();
			if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
				ImGui::SetScrollHereY(1.0f);
			}
			ImGui::End();
		}

		#ifndef DIAGNOSTICS
		{ // controller control
			ImGui::Begin("Controller Control");

			if (ImGui::CollapsingHeader("Batch Read")) {
				if (ImGui::InputInt("First cylinder", &batch_cylinder0)) {
					if (batch_cylinder0 < 0) batch_cylinder0 = 0;
					if (batch_cylinder0 >= DRIVE_CYLINDER_COUNT) batch_cylinder0 = DRIVE_CYLINDER_COUNT-1;
					if (batch_cylinder0 > batch_cylinder1) batch_cylinder1 = batch_cylinder0;
				}
				if (ImGui::InputInt("Last cylinder (inclusive)", &batch_cylinder1)) {
					if (batch_cylinder1 < 0) batch_cylinder1 = 0;
					if (batch_cylinder1 >= DRIVE_CYLINDER_COUNT) batch_cylinder1 = DRIVE_CYLINDER_COUNT-1;
					if (batch_cylinder1 < batch_cylinder0) batch_cylinder0 = batch_cylinder1;
				}
				ImGui::CheckboxFlags("Head 0", &batch_head_set, 1<<0);
				ImGui::SameLine();
				ImGui::CheckboxFlags("Head 1", &batch_head_set, 1<<1);
				ImGui::SameLine();
				ImGui::CheckboxFlags("Head 2", &batch_head_set, 1<<2);
				ImGui::SameLine();
				ImGui::CheckboxFlags("Head 3", &batch_head_set, 1<<3);
				ImGui::SameLine();
				ImGui::CheckboxFlags("Head 4", &batch_head_set, 1<<4);
				ImGui::InputInt("Byte Count", &common_byte_count);
				if (common_byte_count < 0) common_byte_count = 0;
				if (ImGui::Button("Execute!")) {
					com_enqueue("%s %d %d %d %d %d %d",
						CMDSTR_op_read_batch,
						batch_cylinder0,
						batch_cylinder1,
						batch_head_set,
						common_byte_count,
						common_servo_offset,
						common_data_strobe_delay);
				}
			}

			if (ImGui::CollapsingHeader("Basic Operation")) {
				const char* items[] = {
					"Select Unit 0",
					"Select Cylinder",
					"Select Head",
					"Read Enable",
					"Read Data",
				};
				ImGui::Combo("##selected_basic", &basic_selected_index, items, IM_ARRAYSIZE(items));
				switch (basic_selected_index) {
				case 0: {
					// nothing
				} break;
				case 1: {
					ImGui::InputInt("Cylinder##bo", &basic_cylinder);
					ImGui::Checkbox("Allow overflow (full 10-bit range)", &basic_cylinder_allow_overflow);
					if (basic_cylinder < 0) basic_cylinder = 0;
					const int max_cylinder = (basic_cylinder_allow_overflow ? (1<<10) : DRIVE_CYLINDER_COUNT) - 1;
					if (basic_cylinder > max_cylinder) basic_cylinder = max_cylinder;
				} break;
				case 2: {
					ImGui::InputInt("Head##bo", &basic_head);
					if (basic_head < 0) basic_head = 0;
					if (basic_head >= DRIVE_HEAD_COUNT) basic_head = DRIVE_HEAD_COUNT-1;
				} break;
				case 3: {
					// nothing
				} break;
				case 4: {
					ImGui::InputInt("Byte Count", &common_byte_count);
					if (common_byte_count < 0) common_byte_count = 0;
					ImGui::Checkbox("Index Sync", &basic_index_sync);
					ImGui::SetItemTooltip("Waits until INDEX signal from drive before reading");
					ImGui::Checkbox("Skip Checks", &basic_skip_checks);
					ImGui::SetItemTooltip("Skip error/readyness checking; read regardless of status");
				} break;
				}

				if (ImGui::Button("Execute!")) {
					switch (basic_selected_index) {
					case 0: {
						com_enqueue("%s", CMDSTR_op_select_unit0);
					} break;
					case 1: {
						com_enqueue("%s %d", CMDSTR_op_select_cylinder, basic_cylinder);
					} break;
					case 2: {
						com_enqueue("%s %d", CMDSTR_op_select_head, basic_head);
					} break;
					case 3: {
						com_enqueue("%s %d %d", CMDSTR_op_read_enable, common_servo_offset, common_data_strobe_delay);
					} break;
					case 4: {
						com_enqueue("%s %d %d %d",
							CMDSTR_op_read_data,
							common_byte_count,
							basic_index_sync?1:0,
							basic_skip_checks?1:0);
					} break;
					}
				}
			}

			if (ImGui::CollapsingHeader("Control Pins Manual ON/OFF")) {
				{
					int mask = 1;
					#define CONTROL(NAME,SUPPORTED) \
						if (SUPPORTED) { \
							ImGui::CheckboxFlags(#NAME, &debug_control_pins, mask); \
						} \
						mask <<= 1;
					EMIT_CONTROLS
					#undef CONTROL
				}
				ImGui::Checkbox("*LED", &debug_led);
				if (ImGui::Button("Set all")) {
					int mask = 1;
					#define CONTROL(NAME,SUPPORTED) \
						if (SUPPORTED) debug_control_pins |= mask; \
						mask <<= 1;
					EMIT_CONTROLS
					#undef CONTROL
					debug_led = 1;
				}
				if (ImGui::Button("Clear all")) {
					debug_control_pins = 0;
					debug_led = 0;
				}
			}

			if (ImGui::CollapsingHeader("Misc Debugging")) {
				if (ImGui::Button("Execute Blink Test Job (Succeed)")) {
					com_enqueue("%s %d", CMDSTR_op_blink_test, 0);
				}
				ImGui::SameLine();
				if (ImGui::Button("(Fail)")) {
					com_enqueue("%s %d", CMDSTR_op_blink_test, 1);
				}
				if (ImGui::Button("Loopback Test (1000b)")) {
					com_enqueue("%s %d", CMDSTR_loopback_test, 1000);
				}
				ImGui::SameLine();
				if (ImGui::Button("(10000b)")) {
					com_enqueue("%s %d", CMDSTR_loopback_test, 10000);
				}
				ImGui::Checkbox("Poll all GPIO (see log output)", &poll_gpio);
				if (ImGui::Button("Execute Data Download Test (1000b)")) {
					com_enqueue("%s %d", CMDSTR_xfer_test, 1000);
				}
				ImGui::SameLine();
				if (ImGui::Button("(10000b)")) {
					com_enqueue("%s %d", CMDSTR_xfer_test, 10000);
				}
				ImGui::Checkbox("Log status changes", &com.log_status_changes);
			}

			if (ImGui::CollapsingHeader("Fire Extinguishers")) {
				{
					push_danger_style();
					if (ImGui::Button("TERMINATE OPERATION")) {
						com_enqueue("%s", CMDSTR_terminate_op);
					}
					ImGui::SetItemTooltip("Terminates current drive operation on Pico and sets all control pins to zero");
					pop_danger_style();
				}

				if (ImGui::Button("RTZ")) {
					com_enqueue("%s %d", CMDSTR_op_tag3_strobe, TAG3BIT_RTZ);
				}
				ImGui::SetItemTooltip("Return to cylinder zero, clear fault");

				if (ImGui::Button("Clear FAULT")) {
					com_enqueue("%s %d", CMDSTR_op_tag3_strobe, TAG3BIT_FAULT_CLEAR);
				}
			}

			if (ImGui::CollapsingHeader("TAG3/READ_GATE modifiers")) {
				ImGui::TextWrapped("Normal read operations begin by strobing TAG3 with READ_GATE=1; the following translates into modifier bits (see \"Raw Tag\") during the strobe:");
				ImGui::SeparatorText("Servo Offset");
				ImGui::InputInt("##servo_offset", &common_servo_offset);
				ImGui::TextWrapped("\"Offsets the actuator from the nominal on cylinder position toward/away from the spindle.\". +1 is towards (SERVO_OFFSET_POSITIVE), -1 is away (SERVO_OFFSET_NEGATIVE)");
				if (common_servo_offset < -1) common_servo_offset = -1;
				if (common_servo_offset > 1) common_servo_offset = 1;

				ImGui::SeparatorText("Data Strobe Delay");
				ImGui::InputInt("##data_strobe_delay", &common_data_strobe_delay);
				ImGui::TextWrapped("\"Enables the PLO data separator to strobe the data at a time earlier/later than optimum.\". -1 is earlier (DATA_STROBE_EARLY), 1 is later (DATA_STROBE_LATE)");
				if (common_data_strobe_delay < -1) common_data_strobe_delay = -1;
				if (common_data_strobe_delay > 1) common_data_strobe_delay = 1;
			}

			ImGui::End();
		}
		#endif

		if (debug_control_pins != previous_debug_control_pins) {
			com_enqueue("%s %d", CMDSTR_set_ctrl, debug_control_pins);
			previous_debug_control_pins = debug_control_pins;
		}

		if (debug_led != previous_debug_led) {
			com_enqueue("%s %d", CMDSTR_led, debug_led?1:0);
			previous_debug_led = debug_led;
		}

		if (poll_gpio) {
			uint32_t t = SDL_GetTicks();
			if (t > (last_poll_gpio + 25)) {
				com_enqueue("%s", CMDSTR_poll_gpio);
				last_poll_gpio = t;
			}
		}

		pthread_rwlock_unlock(&com.rwlock);

		ImGui::Render();
		glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
		glClearColor(0, 0, 0, 0);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
		SDL_GL_SwapWindow(window);

		#ifdef TELEMETRY_LOG
		current_controls = debug_control_pins;
		if (has_com) {
			pthread_rwlock_rdlock(&com.rwlock);
			const int n = arrlen(com.controller_status_arr);
			current_st = n == 0 ? 0 : com.controller_status_arr[n-1].status;
			pthread_rwlock_unlock(&com.rwlock);
		}
		telemetry_log_status();
		#endif
	}

	// TODO signal thread to quit
	//assert(pthread_join(io_thread, NULL) == 0);

	SDL_GL_DeleteContext(glctx);
	SDL_DestroyWindow(window);

	com_shutdown();

	if (nrz_analyzer_path != NULL) {
		kill(nrz_analyzer_child_pid, SIGTERM);
	}

	return EXIT_SUCCESS;
}
