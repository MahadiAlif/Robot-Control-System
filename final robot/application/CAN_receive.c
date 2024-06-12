/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */


#include "CAN_receive.h"
#include "main.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "control_alg.h"
#include "math_util.h"
#include "br_chassis_control.h"
#include "control_util.h"




extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart1;

int need_to_set_initial_motor_angle[9] = {1,1,1,1,1,1,1,1,1};
float motors_encoder_initial_offset[9];

//motor data read
static inline void get_motor_measure(motor_measure_t* ptr, uint8_t* data, int i) 
{
    uint16_t previous_position = (ptr)->ecd;
    uint16_t current_position = (uint16_t)((data)[0] << 8 | (data)[1]);
		(ptr)->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);
		(ptr)->ang_vel_radsec = (float) (ptr)->speed_rpm * rpm_to_radsec;
		if (IS_STD || IS_BR2 || IS_SENTRY)
			(ptr)->ang_vel_radsec *= ((i <= 3) ? M3508_gearbox_ratio : ((i == 8) ? M2006_gearbox_ratio : 1));
		else
			(ptr)->ang_vel_radsec *= ((i == 0 || i == 8) ? M3508_gearbox_ratio : 1);
    (ptr)->given_current = (int16_t)((data)[4] << 8 | (data)[5]);
    (ptr)->temperate = (data)[6];

		if (need_to_set_initial_motor_angle[i]) {	
			
			if (i == 4 || i == 5) {		// only for gimbal yaw and pitch motor
				ptr->contig_ang_pos_rad = (float) current_position * RAD_ENCODER_ANGLE_RATIO;
				
				if (i == 4) {
					if (ptr->contig_ang_pos_rad - CHASSIS_YAW_ENCODER_ZERO_OFFSET > pi)
						ptr->contig_ang_pos_rad -= 2*pi;
					else if (ptr->contig_ang_pos_rad - CHASSIS_YAW_ENCODER_ZERO_OFFSET < -pi)
						ptr->contig_ang_pos_rad += 2*pi;
				}
				else if (i == 5 && (IS_BR2)) {
					if (ptr->contig_ang_pos_rad - CENTER_ENCODER_ANGLE_PITCH > pi)
						ptr->contig_ang_pos_rad -= 2*pi;
					else if (ptr->contig_ang_pos_rad - CENTER_ENCODER_ANGLE_PITCH < -pi)
						ptr->contig_ang_pos_rad += 2*pi;
				}
			}
			else
				ptr->contig_ang_pos_rad = 0;
			
			motors_encoder_initial_offset[i] = ptr->contig_ang_pos_rad;
			
			need_to_set_initial_motor_angle[i] = 0;
		}
		else {
			
			float delta = (float) (current_position - previous_position);
			delta += (delta < -6144) ? 8192 : ((delta > 6144) ? -8192 : 0);
			
			// take into account the gearbox ratio (only for M3508 and M2006, since GM6020 doesn't have it)
			if (IS_STD || IS_BR2 | IS_SENTRY)
				delta *= ((i <= 3) ? M3508_gearbox_ratio : ((i == 8) ? M2006_gearbox_ratio : 1));
			else
				delta *= ((i == 0 || i == 8) ? M3508_gearbox_ratio : 1);
			
			ptr->contig_ang_pos_rad += delta * RAD_ENCODER_ANGLE_RATIO;
		}
		
		(ptr)->last_ecd = previous_position;
    (ptr)->ecd = current_position;
}
/*
motor data for STANDARD ROBOT:
0: chassis motor1 M3508;		1: chassis motor2 M3508;			2: chassis motor3 M3508;	3: chassis motor4 M3508;
4: yaw gimbal motor GM6020;	5: pitch gimbal motor GM6020;
6: shoot left motor M3508 (without gearbox); 	7: shoot right motor M3508 (without gearbox);		8: rev motor M2006;

motor data for BALANCING ROBOT:
0: chassis motor1 M3508;		1: chassis motor2 M3508;
4: yaw gimbal motor GM6020;	5: pitch gimbal motor GM6020;
6: shoot left motor M3508 (without gearbox); 	7: shoot right motor M3508 (without gearbox);		8: rev motor M2006;

motor data for SENTRY ROBOT:
0: shoot left up motor M3508 (without gearbox);		1: shoot left down motor M3508 (without gearbox);
2: shoot right up motor M3508 (without gearbox);	3: shoot right down motor M3508 (without gearbox);
4: yaw gimbal motor GM6020;			5: pitch gimbal motor GM6020;
6: move on track motor M3508; 	8: rev motor M2006;


Standard robot:
- 4 M3508 for chassis wheels
- 2 GM6020 for gimbal
- 2 M3508 for shoot motors
- 1 M2006 for rev motor

Balancing robot:
- 2 M3508 for chassis wheels
- 2 GM6020 for gimbal
- 2 M3508 for shoot motors
- 1 M2006 for rev motor

Sentry robot:
- 1 M3508 for wheels to move along the track
- 2 GM6020 for gimbal
- 4 M3508 for shoot motors
- 1 M3508 for rev motor
*/

static motor_measure_t motor_measures[9];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  shoot_tx_message;
static uint8_t              shoot_can_send_data[8];

		
char motor_speed_print[30]; 		// TODO check if 30 is too much

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		

    switch (rx_header.StdId)
    {
				case 0x201:
				case 0x202:
				case 0x203:
				case 0x204:
				case 0x205:
				case 0x206:
				case 0x207:
				case 0x208:
        {
					static uint8_t i = 0;
					
          // get motor id
					if (IS_BR2) {
						
						if (rx_header.StdId == 0x208)
							i = 4;
						else
							i = rx_header.StdId - 0x201;
						
						if (hcan == &hcan2)
							i += 0x6;
					}
					else if (IS_STD || IS_SENTRY) {
						
						i = rx_header.StdId - CAN_3508_M1_ID + ((hcan == &SHOOT_REV_CAN) ? 0x6 : 0);
					}
					
          get_motor_measure(&motor_measures[i], rx_data, (int) i);
					
					// TEST: print speed in order to see if it is correct
					if (TURN_ON_MOTOR_SPEED_PRINT)
					{
						sprintf(motor_speed_print,"\t\t%d\n\r", motor_measures[i].speed_rpm);
						HAL_UART_Transmit(&huart1, (uint8_t *)&motor_speed_print, sizeof(motor_speed_print), 10);
					}
					break;
        }

        default:
        {
          break;
        }
    }
}

/**
  * @brief          send control to the friction wheel (0x205, 0x206) , CAN 2			ACTUALLY GM6020 MOTOR IS CONTROLLED THROUGH VOLTAGE AND NOT CURRENT
	* @param[in]      friction wheel (0x201): left shooter
	* @param[in]      friction wheel (0x202): right shooter
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      
  * @retval         none
  */
void CAN_cmd_shoot(int16_t shoot_L, int16_t shoot_R, int16_t rev)
{
    uint32_t send_mail_box;
    shoot_tx_message.StdId = CAN_SHOOTER_REV_ALL_ID; // 0x200
    shoot_tx_message.IDE = CAN_ID_STD;
    shoot_tx_message.RTR = CAN_RTR_DATA;
    shoot_tx_message.DLC = 0x08;
    shoot_can_send_data[0] = (shoot_L >> 8);
    shoot_can_send_data[1] =	shoot_L;
    shoot_can_send_data[2] = (shoot_R >> 8);
    shoot_can_send_data[3] =  shoot_R;
    shoot_can_send_data[4] = (rev >> 8);
    shoot_can_send_data[5] =  rev;
    shoot_can_send_data[6] = 0 >> 8;
    shoot_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&SHOOT_REV_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)			ACTUALLY GM6020 MOTOR IS CONTROLLED THROUGH VOLTAGE AND NOT CURRENT
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
		if (IS_STD || IS_SENTRY) {
			gimbal_can_send_data[0] = (yaw >> 8);
			gimbal_can_send_data[1] = yaw;
			gimbal_can_send_data[2] = (pitch >> 8);
			gimbal_can_send_data[3] = pitch;
			gimbal_can_send_data[4] = (0 >> 8);
			gimbal_can_send_data[5] = 0;
			gimbal_can_send_data[6] = (0 >> 8);
			gimbal_can_send_data[7] = 0;
		}
		if (IS_BR2) {
			gimbal_can_send_data[0] = (0 >> 8);
			gimbal_can_send_data[1] = 0;
			gimbal_can_send_data[2] = (pitch >> 8);
			gimbal_can_send_data[3] = pitch;
			gimbal_can_send_data[4] = (0 >> 8);
			gimbal_can_send_data[5] = 0;
			gimbal_can_send_data[6] = (yaw >> 8);
			gimbal_can_send_data[7] = yaw;
		}
    HAL_CAN_AddTxMessage(&GIMBAL_CHASSIS_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&GIMBAL_CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&GIMBAL_CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
const motor_measure_t *get_chassis_motor_measures(uint8_t i)
{
    return &motor_measures[(i & 0x03)];
}


/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_yaw_gimbal_motor_measures(void)
{
    return &motor_measures[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_pitch_gimbal_motor_measures(void)
{
    return &motor_measures[5];
}


/**
  * @brief          return the shooter 3506 motor data point
  * @param[in]     	i: motor number. 0->left shoot motor, 1->right shoot motor
  * @retval         motor data point
  */
const motor_measure_t *get_shoot_motor_measures(uint8_t i)
{
    return &motor_measures[(i & 0x1) + 6];
}


/**
  * @brief          return the rev 3508 motor data point
	* @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_rev_motor_measures(void)
{
    return &motor_measures[8];
}


