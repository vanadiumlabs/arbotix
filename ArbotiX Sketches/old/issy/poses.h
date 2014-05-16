#include <avr/pgmspace.h>

/* default star-shaped stand */
PROGMEM prog_uint16_t stand[ ]  =    {0x0C,512,512,482,542,662,362,512,512,542,482,362,662}; 

/* pick up and move front left and right rear legs forwards, horizontals are centered on range */
PROGMEM prog_uint16_t walk0[ ] =     {0x0C,552,532,482,492,612,412,532,552,492,482,412,612};
PROGMEM prog_uint16_t walkend[ ] =   {0x0C,552,472,482,542,562,462,472,552,542,482,412,612};

/* pick up and move right front and left rear legs forwards, horizontals are centered on range */
PROGMEM prog_uint16_t walk1[ ] =     {0x0C,492,472,532,542,612,412,472,492,542,532,412,612};
