#ifndef POSES
#define POSES

#include <avr/pgmspace.h>



const PROGMEM uint16_t Center[] = {6, 2048, 2048, 2048, 2048, 512, 512};
const PROGMEM uint16_t Home[] = {6,2033, 1698, 1448, 2336, 512, 512};
const PROGMEM uint16_t Sleep[] = {6,2083, 931, 1039, 1677, 486, 476};
//const PROGMEM uint16_t Home[] = {6, 2160, 1514, 1657, 1601, 486, 476};
const PROGMEM uint16_t Up[] = {6,2247, 1674, 2237, 2556, 488, 476};
const PROGMEM uint16_t DownGripper[] = {6, 2427, 2257, 1922, 1368, 793, 149};
const PROGMEM uint16_t RightAngle[] = {6,2222, 2021, 2019, 1988, 829, 476};
const PROGMEM uint16_t Turn[] = {6, 3291, 2351, 1665, 3067, 828, 474};



#endif


