#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdlib.h>
#include <stdint.h>

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/

#define NUM_ROBOTS 	6			// number of robots we have (1 standard, 2 balancing, 2 legged-balancing, 1 sentry)
#define UNDEF		-1		// undefined (can be used for anything that you want to leave blank or is not defined)

/********** SET THE FLAG CORRESPONDING TO THE ROBOT ON WHICH THE CODE WILL BE LOADED **********/

#define IS_STD 		1
#define IS_BR1 		0			// not available (disassembled)
#define IS_BR2 		0
#define IS_LBR1 	0
#define IS_LBR2 	0
#define IS_SENTRY	0

/********** SET THE FLAGS CORRESPONDING TO THE ROBOT PARTS/MOTORS TO ENABLE **********/

// NOTE: robot parts/motors not selected here will be disabled (i.e. will not move)
#define IS_CHASSIS_ENABLED 				1			// enable/disable chassis
#define IS_GIMBAL_ENABLED 				1			// enable/disable gimbal
#define IS_SHOOT_ENABLED 				1			// enable/disable shooting wheels
#define IS_REV_ENABLED 					1			// enable/disable REV motor
#define IS_SENTRY_CHASSIS_ROT_ENABLED	1			// enable/disable the autonomous chassis rotation of sentry robot
#define IS_SENTRY_AUTO_SHOOT_ENABLED	1			// enable/disable the autonomous shooting of sentry robot (1 if sentry shoots autonomously; 0 if sentry can shoot only through the remote controller commands)
#define IS_PILOT_AUTO_AIM_ENABLED		0			// enable/disable the CV-driven gimbal auto-aim on robots with a pilot (i.e., standard and balancing robot)


/********** SET THE FLAGS FOR ENABLING/DISABLING POWER CONSUMPTION MANAGEMENT **********/

#define IS_POW_CONS_CHASSIS_ENABLED 		0
#define IS_POW_CONS_SHOOT_FREQ_ENABLED 		0
#define IS_POW_CONS_SHOOT_SPEED_ENABLED 	0


/********** CHECK THAT ABOVE FLAGS ARE CORRECTLY SET **********/
#define WRONG_ROBOT_SETTINGS ((IS_STD < 0) || (IS_BR1 < 0) || (IS_BR2 < 0) || (IS_LBR1 < 0) || (IS_LBR2 < 0) || (IS_SENTRY < 0) || (IS_STD + IS_BR1 + IS_BR2 + IS_LBR1 + IS_LBR2 + IS_SENTRY != 1))


/********** ROBOTS ENUMERATION **********/
#define STD 	0
#define BR1 	1
#define BR2 	2
#define LBR1 	3
#define LBR2 	4
#define SENTRY	5


/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

extern int8_t rid;		// robot ID: identifies the robot that you want to use (according to the settings in \controller\robot_config.h)



/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/


#endif
