#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP_t;

ext_event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_referee_warning_t referee_warning_t;


ext_game_robot_state_t robot_state;
ext_power_heat_data_t power_heat_data_t;
ext_game_robot_pos_t game_robot_pos_t;
ext_buff_musk_t buff_musk_t;
aerial_robot_energy_t robot_energy_t;
ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;
ext_bullet_remaining_t bullet_remaining_t;
ext_student_interactive_data_t student_interactive_data_t;



// init of every structure/variables of the referee sys	;		void * memset ( void * ptr, int value, size_t num ); this function is used to clear the structure
void init_referee_struct_data(void)				
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));			//init of struct for variable of received messages
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));					//init of struct for variable of sent messages

    memset(&game_state, 0, sizeof(ext_game_state_t));												//init of struct for variable like game time and game type
    memset(&game_result, 0, sizeof(ext_game_result_t));											//init of struct for variable game win
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));								//init of struct for variable life of every robot, both red or blue


    memset(&field_event, 0, sizeof(ext_event_data_t));																		//init of struct for variable of the event type
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));				//init of struct for variable related to the supply robot 
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));													//init of struct for variable related to referee warning


		memset(&robot_state, 0, sizeof(ext_game_robot_state_t));								// !!! most important one: init of struct for variable realted to hp of robot, power consumption...
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));						//init of struct for variable of shooter and chassis main control parameter
    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));							//init of struct for variable of the position of the robot in the arena (?)
    memset(&buff_musk_t, 0, sizeof(ext_buff_musk_t));												//init of struct for variable of power buffer
    memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));							//init of struct for variable of energy point and attack time
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));											//init of struct for variable of armor type and hurt type
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));											//init of struct for variable of bullet type, freq and speed
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));					//init of struct for variable f the number of remaining bullets


    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));					//init of struct for variable related to student id and packet



}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;


        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&field_event, frame + index, sizeof(field_event));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
        }
        break;
				
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, frame + index, sizeof(robot_state));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
        }
        break;
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&robot_energy_t, frame + index, sizeof(robot_energy_t));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
        }
        break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
        }
        break;
        default:
        {
            break;
        }
    }
}

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.chassis_power_buffer;

}


uint8_t get_robot_id(void)
{
    return robot_state.robot_id;
}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
    *heat0_limit = robot_state.shooter_id1_17mm_cooling_limit;
    *heat0 = power_heat_data_t.shooter_id1_17mm_cooling_heat;
}

void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    *heat1_limit = robot_state.shooter_id2_17mm_cooling_limit;
    *heat1 = power_heat_data_t.shooter_id2_17mm_cooling_heat;
}


/* functions created by RoboTO */

ext_game_robot_state_t get_robot_state(void)
{
	return robot_state;
}

ext_shoot_data_t get_shoot_data(void)
{
	return shoot_data_t;
}

ext_game_state_t get_game_state(void)
{
	return game_state;
}

uint8_t get_game_progress(void)
{
	return game_state.game_progress;
}


