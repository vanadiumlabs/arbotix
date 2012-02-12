#ifndef DROID_POSES
#define DROID_POSES

#include <avr/pgmspace.h>

PROGMEM prog_uint16_t liftLeft[] = {4, 612, 620, 450, 380};
PROGMEM prog_uint16_t plantRight[] = {4, 612, 620, 508, 516};
PROGMEM prog_uint16_t swingRight[] = {4, 612, 620, 710, 593};
PROGMEM prog_uint16_t swingLeft[] = {4, 413, 410, 450, 390};
PROGMEM prog_uint16_t plantLeft[] = {4, 418, 411, 503, 516};
PROGMEM prog_uint16_t liftRight[] = {4, 523, 520, 712, 592};

PROGMEM transition_t forward[] = {{0,7} ,{plantLeft,250} ,{liftRight,250} ,{swingRight,250} ,{plantRight,250} ,{liftLeft,250} ,{swingLeft,250} ,{plantLeft,250} };

#endif
