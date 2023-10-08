#ifndef BASE_H

#include <stdint.h>

// blink codes for PANIC()
#define PANIC_XXX                       (0xAAA)
#define PANIC_STOP                      (0xA1)
#define PANIC_UNREACHABLE               (0xA3)
#define PANIC_UNHANDLED_COMMAND         (0xAC)
#define PANIC_BOUNDS_CHECK_FAILED       (0x123)
#define PANIC_UNEXPECTED_STATE          (0x33)
#define PANIC_ALLOCATION_ERROR          (0x22)

#define ARRAY_LENGTH(xs) (sizeof(xs) / sizeof(xs[0]))
#define ARRAY_END(xs) ((xs) + ARRAY_LENGTH(xs))
//#define LOG2_ALIGN(LG2, VALUE) (((VALUE) + ((1<<LG2)-1)) & (~((1<<LG2)-1)))

#define  LED_PIN  PICO_DEFAULT_LED_PIN // GP25

#define FREQ_IN_MICROS(HZ) (1000000/(HZ))

void set_led(int p);
void blink(int on_ms, int off_ms);

__attribute__ ((noreturn))
void PANIC(uint32_t error);

#define BYTES_TO_32BIT_WORDS(b) (((b)+3)>>2)

#define BASE_H
#endif
