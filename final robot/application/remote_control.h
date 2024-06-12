/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "bsp_rc.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_W            ((uint16_t)1 << 0)
#define KEY_S            ((uint16_t)1 << 1)
#define KEY_A            ((uint16_t)1 << 2)
#define KEY_D            ((uint16_t)1 << 3)
#define KEY_SHIFT        ((uint16_t)1 << 4)
#define KEY_CTRL         ((uint16_t)1 << 5)
#define KEY_Q            ((uint16_t)1 << 6)
#define KEY_E            ((uint16_t)1 << 7)
#define KEY_R            ((uint16_t)1 << 8)
#define KEY_F            ((uint16_t)1 << 9)
#define KEY_G            ((uint16_t)1 << 10)
#define KEY_Z            ((uint16_t)1 << 11)
#define KEY_X            ((uint16_t)1 << 12)
#define KEY_C            ((uint16_t)1 << 13)
#define KEY_V            ((uint16_t)1 << 14)
#define KEY_B            ((uint16_t)1 << 15)

/* ----------------------- PC Mouse-Key Definition-------------------------------- */
#define MOUSE_LEFT_KEY							1
#define MOUSE_RIGHT_KEY							0




/*
Commands to standard robot from keyboard:
	W 								-> chassis forward
	S									-> chassis backward
	D									-> chassis right
	A									-> chassis left
	Shift + W/S/D/A		-> move chassis slower
	Q									-> increase speed of shoot motors
	Ctrl + Q					-> decrease speed of shoot motors
	F									-> increase gain of gimbal pitch
	Ctrl + F					-> decrease gain of gimbal pitch
	G									-> increase gain of gimbal yaw
	Ctrl + G					-> decrease gain of gimbal yaw
	V									-> increase mouse sensibility of gimbal pitch
	Ctrl + V					-> decrease mouse sensibility of gimbal pitch
	B									-> increase mouse sensibility of gimbal yaw
	Ctrl + B					-> decrease mouse sensibility of gimbal yaw
	E									-> activate/deactivate defence mode, where chassis starts rotating (if not yet in "chassis-gimbal independent", this key puts the robot into this control mode)
	R									-> switch robot control mode (from "chassis-follow-gimbal" to "chassis-gimbal independent" and vice versa)
	
	implement the increase/decrease velocity of rev motor when it is in multiple shoot mode?
	implement the increase/decrease pitch_goes_down gain?


Commands to balancing robot from keyboard:
	W 								-> chassis forward (only when chassis-follow-gimbal)
	S									-> chassis backward (only when chassis-follow-gimbal)
	D									-> chassis right (only when chassis is 90 deg w.r.t. gimbal)
	A									-> chassis left (only when chassis is 90 deg w.r.t. gimbal)
	C (hold it)				-> brake chassis movement
	Q									-> increase speed of shoot motors
	Ctrl + Q					-> decrease speed of shoot motors
	F									-> increase gain of gimbal pitch
	Ctrl + F					-> decrease gain of gimbal pitch
	G									-> increase gain of gimbal yaw
	Ctrl + G					-> decrease gain of gimbal yaw
	V									-> increase mouse sensibility of gimbal pitch
	Ctrl + V					-> decrease mouse sensibility of gimbal pitch
	B									-> increase mouse sensibility of gimbal yaw
	Ctrl + B					-> decrease mouse sensibility of gimbal yaw
	E									-> activate/deactivate defence mode, where chassis starts rotating (if not yet in "chassis-gimbal independent", this key puts the robot into this control mode)
	R									-> switch robot control mode (from "chassis-follow-gimbal" to "chassis 90 deg w.r.t. gimbal" and vice versa)
	Z									-> increase theta_balance_offset (only the one when chassis-follow-gimbal)
	Ctrl + Z					-> increase theta_balance_offset (only the one when chassis-follow-gimbal)
	

*/

/* ----------------------- Data Struct ------------------------------------- */
typedef __attribute__((packed)) struct
{
        __attribute__((packed)) struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __attribute__((packed)) struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __attribute__((packed)) struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;


/* ----------------------- Robot modes ------------------------------------- */

#define REMOTE_CONTROLLER_MODE											0
#define CHASSIS_FOLLOW_GIMBAL_PC_MODE								1
#define FIXED_GIMBAL_PC_MODE												2
#define BALANCING_90_DEGREE_PC_MODE									2
#define ERROR_STOP_MODE															3


/* ---------------------- Shooting modes ---------------------------------- */

#define NO_SHOOT_MODE											2
#define SINGLE_SHOOT_MODE									3
#define MULTIPLE_SHOOT_MODE								1	
#define TRIPLE_SHOOT_MODE									4			


/* ---------------------- Get switches position --------------------------- */

#define rc_left_switch get_remote_control_point()->rc.s[1]
#define rc_right_switch get_remote_control_point()->rc.s[0]
#define rc_left_switch_prev get_old_remote_control_point()->rc.s[1]
#define rc_right_switch_prev get_old_remote_control_point()->rc.s[0]

/* ---------------------- Get "wheel" button position in up-left part of remote controller --------------------------- */

#define rc_right_joystick_rl get_remote_control_point()->rc.ch[0]			// right joystick, in range [-660,+660], from left to right
#define rc_right_joystick_ud get_remote_control_point()->rc.ch[1]			// right joystick, in range [-660,+660], from up to down
#define rc_left_joystick_rl get_remote_control_point()->rc.ch[2]			// left joystick, in range [-660,+660], from left to right
#define rc_left_joystick_ud get_remote_control_point()->rc.ch[3]			// left joystick, in range [-660,+660], from up to down
#define rc_up_left_wheel get_remote_control_point()->rc.ch[4]			// in range [-660,+660] in counter-clockwise direction


/* ----------------------- Internal Data ----------------------------------- */

extern uint8_t robot_control_mode;
extern uint8_t new_robot_control_mode;

extern void remote_control_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
extern const RC_ctrl_t *get_old_remote_control_point(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
extern void sbus_to_usart1(uint8_t *sbus);

extern int is_key_pressed(uint16_t key);
extern int is_key_falling_edge(uint16_t key);
extern int is_key_raising_edge(uint16_t key);
extern int is_any_WASD_key_pressed(void);
extern int is_mouse_left_key_pressed(void);
extern int is_mouse_right_key_pressed(void);

/************************************************************************************************************************
	NAME: is_mouse_falling_edge
	
	DESCRIPTION: detect the falling edge of the mouse keys

	ARGUMENTS:
	- mouse_key:		1 --> left
									0 --> right
************************************************************************************************************************/
extern int is_mouse_falling_edge(uint8_t mouse_key);
/************************************************************************************************************************
	NAME: is_mouse_raising_edge
	
	DESCRIPTION: detect the raising edge of the mouse keys

	ARGUMENTS:
	- mouse_key:		1 --> left
									0 --> right
************************************************************************************************************************/
extern int is_mouse_raising_edge(uint8_t mouse_key);
#endif
