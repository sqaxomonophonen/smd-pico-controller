// HINT! See also `print_pin_config_html.c` and `doc/pin_config.html`
#ifndef CONFIG_H
//                                     GP0-GP28 (GPIO number, *not* pin number)
// NOTE: READ_DATA and READ_CLOCK must    |
// be consequtive; see _Static_assert     |
#define EMIT_PIN_CONFIG \
	PIN(  DATA,      READ_DATA,       0   ) \
	PIN(  DATA,      READ_CLOCK,      1   ) \
	PIN(  FREQ,      INDEX,           2   ) \
	PIN(  FREQ,      SECTOR,          3   ) \
	PIN(  STATUS,    FAULT,           4   ) \
	PIN(  STATUS,    SEEK_ERROR,      5   ) \
	PIN(  STATUS,    ON_CYLINDER,     6   ) \
	PIN(  STATUS,    UNIT_READY,      7   ) \
	PIN(  STATUS,    ADDRESS_MARK,    8   ) \
	PIN(  STATUS,    UNIT_SELECTED,   9   ) \
	PIN(  STATUS,    SEEK_END,        10  ) \
	PIN(  CONTROL,   UNIT_SELECT_TAG, 14  ) \
	PIN(  CONTROL,   TAG1,            11  ) \
	PIN(  CONTROL,   TAG2,            12  ) \
	PIN(  CONTROL,   TAG3,            13  ) \
	PIN(  CONTROL,   BIT0,            16  ) \
	PIN(  CONTROL,   BIT1,            17  ) \
	PIN(  CONTROL,   BIT2,            18  ) \
	PIN(  CONTROL,   BIT3,            19  ) \
	PIN(  CONTROL,   BIT4,            20  ) \
	PIN(  CONTROL,   BIT5,            21  ) \
	PIN(  CONTROL,   BIT6,            22  ) \
	PIN(  CONTROL,   BIT7,            26  ) \
	PIN(  CONTROL,   BIT8,            27  ) \
	PIN(  CONTROL,   BIT9,            28  )

// Naming matches figure 3-11 in this document:
// https://bitsavers.org/pdf/cdc/discs/smd/83322200M_CDC_BK4XX_BK5XX_Hardware_Reference_Manual_Jun1980.pdf

enum gpio_type { // <- first PIN() column
	DATA     = 1, // input/data:   9.67MHz data rate
	FREQ,         // input/freq:   INDEX is 3600RPM/60=60hz, SECTOR is a multiple of that (potentially "non-integer")
	STATUS,       // input/status: low bitrate
	CONTROL,      // output: low bitrate
};

enum gpio_map {
	#define PIN(TYPE, NAME, GPIO) GPIO_ ## NAME = GPIO,
	EMIT_PIN_CONFIG
	#undef PIN
};

#ifndef __cplusplus
_Static_assert(
	GPIO_READ_CLOCK == (GPIO_READ_DATA+1),
	"READ_DATA and READ_CLOCK must have consequtive GPIO pins due to PIO constraints (see PINCTRL discussion in RP2040 datasheet)");
#endif

#define FREQ_FREQ_HZ (1)

#define CONFIG_H
#endif
