#ifndef DROID_POSES
#define DROID_POSES

#include <avr/pgmspace.h>

PROGMEM prog_uint16_t liftLeft[] = {0x0C, 612, 620, 450, 381};

PROGMEM prog_uint16_t plantRight[] = {0x0C, 612, 620, 508, 516};

PROGMEM prog_uint16_t swingRight[] = {0x0C, 612, 620, 710, 593};

PROGMEM prog_uint16_t swingLeft[] = {0x0C, 413, 410, 450, 390};

PROGMEM prog_uint16_t stand[] = {0x0C, 512, 512, 512, 512};

PROGMEM prog_uint16_t plantLeft[] = {0x0C, 418, 411, 503, 516};

PROGMEM prog_uint16_t liftRight[] = {0x0C, 523, 520, 712, 592};

#endif
