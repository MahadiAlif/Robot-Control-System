#ifndef REFEREE_ALG_H
#define REFEREE_ALG_H

#include <stdint.h>
#include "referee.h"
#include "shoot_rev_control.h"
#include "std_chassis_control.h"
#include "gimbal_control.h"
#include "br_chassis_control.h"

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/

#define USE_REFEREE_SYS_SHOOT_HEAT 0
#define USE_REFEREE_SYS_SHOOT_SPEED 0
#define USE_REFEREE_SYS_CHASSIS_POWER_CONSUMPTION 0

#define SECURITY_PERCENTAGE_CHASSIS_POWER_CONSUMPTION 0.8
#define SECURITY_PERCENTAGE_BARREL_HEAT 0.8
#define SECURITY_PERCENTAGE_SHOOT_SPEED 0.8

/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/* barrel heat referee for rev motor */
typedef struct barrel_heat_referee_s {		// this is for the shooting frequency
	
	// barrel heat values
	uint16_t barrel_heat_estimation;	// sum of real_barrel_heat and pre_shoot_heat
	uint16_t real_barrel_heat;
	uint16_t real_barrel_heat_prev;
	uint16_t pre_shoot_heat;
	
	// max limit of barrel heat
	uint16_t barrel_heat_limit;
	
	// counters used to manage the pre_shoot_heat
	uint16_t start_clock;
	uint16_t counter_avoiding_pre_shoot_gets_stuck;
	
	// flag to check if (theoretically) we still need to shoot a bullet
	int still_need_to_shoot;
	
} barrel_heat_referee_t;

/* shooting speed referee for shoot motors */
typedef struct shoot_speed_referee_s {
	
	// speeds of current and previous bullets
	float bullet_speed;
	float bullet_speed_prev;
	
	// flag to check that a new bullet has been shooted
	int new_bullet_shooted;
	
} shoot_speed_referee_t;

/* chassis power consumption referee */
typedef struct chassis_power_consumption_referee_s {

	float current_power_consumption;
	float power_consumption_limit;
	
} chassis_power_consumption_referee_t;


/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

// referee structures
extern barrel_heat_referee_t barrel_heat;
extern chassis_power_consumption_referee_t chassis_power_consumption;
extern shoot_speed_referee_t shoot_speed;

// functions to initialize the referee structures
void barrel_heat_init(void);
void chassis_power_consumption_init(void);
void shoot_speed_init(void);

// functions to check that we fulfill the competition rules
void check_barrel_heat_shooting(int *allowed_to_shoot);
void check_chassis_power_consumption(void);
void check_shoot_speed(void);

#endif



