#include "referee_alg.h"
#include "control_util.h"
#include "control_alg.h"
#include "robot_config.h"

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/


/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

barrel_heat_referee_t barrel_heat;
shoot_speed_referee_t shoot_speed;
chassis_power_consumption_referee_t chassis_power_consumption;

float power_limit_to_chassis_control_signal_limit = (IS_STD) ? 1.0 : ((IS_SENTRY) ? 1.0 : ((IS_BR1) ? 1.0 : 1.0));

// shooting referee
uint16_t curr_shoot_heat;				// current shooting heat
uint16_t shoot_heat_limit;			// shooting heat limit
float single_bullet_heat = 8;		// barrel heat generated by a single bullet

/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void barrel_heat_init(void) {
	
	// barrel heat values
	barrel_heat.real_barrel_heat = 0;
	barrel_heat.real_barrel_heat_prev = 0;
	barrel_heat.pre_shoot_heat = 0;
	barrel_heat.barrel_heat_estimation = barrel_heat.real_barrel_heat + barrel_heat.pre_shoot_heat;
	
}

void shoot_speed_init(void) {
	
	shoot_speed.bullet_speed = 0;
	shoot_speed.new_bullet_shooted = 0;
}

void chassis_power_consumption_init(void) {
	
	chassis_power_consumption.power_consumption_limit = get_robot_state().chassis_power_limit;
}








void check_barrel_heat_shooting(int *allowed_to_shoot) {
	
	get_shoot_heat0_limit_and_heat0(&shoot_heat_limit, &curr_shoot_heat);
	
	if (curr_shoot_heat + single_bullet_heat*9 >= shoot_heat_limit)			// ensure that, even if we do a triple shoot, we'll never exceed the barrel heat limit
		*allowed_to_shoot = FALSE;
	
	
	
//	barrel_heat.real_barrel_heat_prev = barrel_heat.real_barrel_heat;
//	get_shoot_heat0_limit_and_heat0(&barrel_heat.barrel_heat_limit, &barrel_heat.real_barrel_heat);
	
//	if (barrel_heat.real_barrel_heat > barrel_heat.barrel_heat_limit * SECURITY_PERCENTAGE_BARREL_HEAT)
//		allowed_to_shoot_single_bullet = 0;		// deny to shoot the new bullet
//	
//	barrel_heat.real_barrel_heat_prev = barrel_heat.real_barrel_heat;
//	get_shoot_heat0_limit_and_heat0(&barrel_heat.barrel_heat_limit, &barrel_heat.real_barrel_heat);
//	
//	if (barrel_heat.pre_shoot_heat == 0)
//		barrel_heat.start_clock = clock_ms;
//	
//	if (barrel_heat.real_barrel_heat > barrel_heat.real_barrel_heat_prev ||
//			clock_ms - barrel_heat.start_clock > 1500) {		// this means that we just shooted a bullet
//		barrel_heat.pre_shoot_heat -= 10;
//		barrel_heat.start_clock = clock_ms;
//	}
//	
//	// uodate overall barrel heat estimation, and check if we can shoot
//	barrel_heat.barrel_heat_estimation = barrel_heat.real_barrel_heat + barrel_heat.pre_shoot_heat;
//	if (barrel_heat.barrel_heat_estimation <= barrel_heat.barrel_heat_limit * SECURITY_PERCENTAGE_BARREL_HEAT)
//		barrel_heat.pre_shoot_heat += 10;			// allowed to shoot, hence need to update pre_shoot_heat
//	else {
//		allowed_to_shoot_single_bullet = 0;		// deny to shoot the new bullet
//	}
}

void check_shoot_speed(void) {
	
	shoot_speed.bullet_speed_prev = shoot_speed.bullet_speed;
	shoot_speed.bullet_speed = get_shoot_data().bullet_speed;
	
	// quando sparo un bullet, il valore della sua velocit� rimane memorizzato fino al prossimo bullet
	
	// check that we don't exceed the max shooting speed allowed for our robot's level
	if (ref_shoot_motors_speed_radsec > initial_shoot_speed_when_restart)
		ref_shoot_motors_speed_radsec = initial_shoot_speed_when_restart;
	
	// se il nuovo sparo � andato troppo veloce, diminuire la velocit� degli shoot motors di 50
	if (shoot_speed.bullet_speed != shoot_speed.bullet_speed_prev)
		shoot_speed.new_bullet_shooted = 1;
	else
		shoot_speed.new_bullet_shooted = 0;
	
	if (shoot_speed.new_bullet_shooted == 1 && shoot_speed.bullet_speed > get_robot_state().shooter_id1_17mm_speed_limit)
		ref_shoot_motors_speed_radsec = max(ref_shoot_motors_speed_radsec - 50, 0.0);
}

void check_chassis_power_consumption(void) {
	
	chassis_power_consumption.power_consumption_limit = get_robot_state().chassis_power_limit;		// update chassis power consumption limit
	get_chassis_power_and_buffer(&chassis_power_consumption.current_power_consumption, (fp32 *) NULL);	// get current power consumption
	
	float chassis_control_signal_limit = chassis_power_consumption.power_consumption_limit * power_limit_to_chassis_control_signal_limit;
	
	if (IS_STD) {
		for (int i = 0; i < 4; i++) {
//			saturate(&standard_chassis.control_signals[i], chassis_control_signal_limit * SECURITY_PERCENTAGE_CHASSIS_POWER_CONSUMPTION);
		}
	}
	else if (IS_BR2) {
		for (int i = 0; i < 2; i++) {
//			saturate(&BR_chassis.control_signals[i], chassis_control_signal_limit * SECURITY_PERCENTAGE_CHASSIS_POWER_CONSUMPTION);
		}
	}
	else if (IS_SENTRY) {
//		saturate(&sentry_chassis.control_signal, chassis_control_signal_limit * SECURITY_PERCENTAGE_CHASSIS_POWER_CONSUMPTION);
	}
}



