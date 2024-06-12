#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"

/*--------------------------------------------------------*/
typedef enum
{
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    BLUE_HERO = 11,
    BLUE_ENGINEER = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL = 16,
    BLUE_SENTRY = 17,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART = 0,
    PROGRESS_PREPARE = 1,
    PROGRESS_SELFCHECK = 2,
    PROGRESS_5sCOUNTDOWN = 3,
    PROGRESS_BATTLE = 4,
    PROGRESS_CALCULATING = 5,
} game_progress_t;

/*--------------------------------------------------------*/
//Competiotion Status data 0x0001
typedef __packed struct
{
    uint8_t game_type : 4;			//Those are bit fields. Basically, the number after the colon describes how many bits that field uses
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_state_t;

/*--------------------------------------------------------*/
//Competion result data 0x0002
typedef __packed struct
{
    uint8_t winner;
} ext_game_result_t;

/*--------------------------------------------------------*/
//Robot HP data 0x003
typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_oupost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/*--------------------------------------------------------*/
//AI Buff and debuff zone distribution and lurking mode 0x0005
typedef __packed struct
{
    uint8_t F1_zone_status : 1;
    uint8_t F1_zone_buff_debuff_status : 3;
    uint8_t F2_zone_status : 1;
    uint8_t F2_zone_buff_debuff_status : 3;
    uint8_t F3_zone_status : 1;
    uint8_t F3_zone_buff_debuff_status : 3;
    uint8_t F4_zone_status : 1;
    uint8_t F4_zone_buff_debuff_status : 3;
    uint8_t F5_zone_status : 1;
    uint8_t F5_zone_buff_debuff_status : 3;
    uint8_t F6_zone_status : 1;
    uint8_t F6_zone_buff_debuff_status : 3;

    uint16_t red1_bullet_left;
    uint16_t red2_bullet_left;
    uint16_t blue1_bullet_left;
    uint16_t blue2_bullet_left;
    uint8_t lurk_mode;
    uint8_t res;
} ext_ICRA_buff_debuff_zone_and_lurk_status_t;

/*--------------------------------------------------------*/
//Site event data 0x0101
typedef __packed struct
{
    uint32_t event_type;
} ext_event_data_t;

/*--------------------------------------------------------*/
//Action identifier of official projectile supplier 0x0102
typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/*--------------------------------------------------------*/
//Referee warning information 0x0104
typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;

/*--------------------------------------------------------*/
//Data launching opening countdown 0x0105
typedef __packed struct
{
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

/*--------------------------------------------------------*/
//Robot status competition 0x0201
typedef __packed struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

/*--------------------------------------------------------*/
//Real-time power and heat data 0x0202
typedef __packed struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

/*--------------------------------------------------------*/
//Robot position 0x0203
typedef __packed struct
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

/*--------------------------------------------------------*/
//Robot buffs 0x0204
typedef __packed struct
{
    uint8_t power_rune_buff;
} ext_buff_musk_t;

/*--------------------------------------------------------*/
//Aerial robot energy status 0x0205
typedef __packed struct
{
    uint8_t attack_time;
} aerial_robot_energy_t;

/*--------------------------------------------------------*/
//Demage status 0x0206
typedef __packed struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/*--------------------------------------------------------*/
//Real-time launching information 0x0207
typedef __packed struct
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;

/*--------------------------------------------------------*/
//Remaining projectile 0x0208
typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

/*--------------------------------------------------------*/
//Robot RFID status 0x0209
typedef __packed struct
{
    uint32_t rfid_status;
} ext_rfid_status_t;

/*--------------------------------------------------------*/
//Instruction data of dart robot client 0x020A
typedef __packed struct
{
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/*--------------------------------------------------------*/
//Graphic data
typedef __packed struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t radius : 10;
    uint32_t end_x : 11;
    uint32_t end_y : 11;
} graphic_data_struct_t;

/*--------------------------------------------------------*/
//INTERACTION DATA BETWEEN ROBOTS
//Interaction data receiving information 0x0301
typedef __packed struct
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
} ext_student_interactive_data_t;

//Communication of interaction data between robots 0x0301
//typedef __packed struct
//{
//    uint8_t data[];
//} robot_interactive_data_t;

//Communication between robots when client delates graphic 0x0301
typedef __packed struct
{
    uint8_t operate_tpye;
    uint8_t layer;
} ext_client_custom_graphic_delete_t;

//Communication between robots when client draws one graphic 0x0301
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

//Communication between robots when client draws two graphics 0X0301
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

//Communication between robots when client draws five graphics 0x0301
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

//Communication between robots when client draws seven graphics 0x0301
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

//Communication between robots when client draws a character 0x0301
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;

/*--------------------------------------------------------*/
//Interaction data receiving information 0x0302
//typedef __packed struct
//{
//uint8_t data[];
//} robot_interactive_data_t;

/*--------------------------------------------------------*/
//Information transmission identifier: transmitted when triggered 0x0303
//typedef __packed struct
//{
//float target_position_x;
//float target_position_y;
//float target_position_z;
//uint8_t commd_keyboard;
//uint16_t target_robot_ID;
//} ext_robot_command_t;

//Information transmission identifier:client receipt information 0x0305
typedef __packed struct
{
    uint16_t target_robot_ID;
    float target_position_x;
    float target_position_y;
} ext_client_map_command_t;

/*--------------------------------------------------------*/
//Image transmission remote control 0x0304
typedef __packed struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} ext_robot_command_t;

/*--------------------------------------------------------*/
extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t* frame);

extern void get_chassis_power_and_buffer(fp32* power, fp32* buffer);

extern uint8_t get_robot_id(void);

extern void get_shoot_heat0_limit_and_heat0(uint16_t* heat0_limit, uint16_t* heat0);
extern void get_shoot_heat1_limit_and_heat1(uint16_t* heat1_limit, uint16_t* heat1);

/* functions created by RoboTO */
ext_game_robot_state_t get_robot_state(void);
ext_shoot_data_t get_shoot_data(void);
ext_game_state_t get_game_state(void);
uint8_t get_game_progress(void);

#endif
