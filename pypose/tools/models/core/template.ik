@GAIT_GENERATORS
/* ripple gaits move one leg at a time TODO: THIS SHOULD BE IN NUKE.H
 *  for going forward, or turning left/right
 */
#define RIPPLE                  0
@IF legs 4
#define RIPPLE_LEFT             1
#define RIPPLE_RIGHT            2
/* amble gaits move two alternate legs at a time */
#define AMBLE                   5 
@ELSE
/* tripod gaits are only for hexapods */
#define TRIPOD                  7
@END
@END_SECTION
