#include "robot_config.h"

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/


/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

// set the robot ID (represents the robots that we want to use)
#if WRONG_ROBOT_SETTINGS
int8_t rid = UNDEF;
#elif IS_STD
int8_t rid = STD;
#elif IS_BR1
int8_t rid = BR1;
#elif IS_BR2
int8_t rid = BR2;
#elif IS_LBR1
int8_t rid = LBR1;
#elif IS_LBR2
int8_t rid = LBR2;
#elif IS_SENTRY
int8_t rid = SENTRY;
#endif



/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/




