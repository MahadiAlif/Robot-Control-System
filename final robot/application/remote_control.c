/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң����������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж���������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"

#include "main.h"

// not needed for now #include "bsp_usart.h"
#include "string.h"
#include "stdio.h"
#include "gimbal_task.h"



#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

char usart1_print_rc_ctrl[15];
extern UART_HandleTypeDef huart1;

// robot control mode variables
uint8_t flag_error_mode;
uint8_t robot_control_mode;
uint8_t new_robot_control_mode;

//check max mouse value
//int16_t mouse_x;
//int16_t mouse_y;
//int16_t mouse_z;

//int16_t mouse_x_max;
//int16_t mouse_y_max;
//int16_t mouse_z_max;

//int16_t mouse_x_min;
//int16_t mouse_y_min;
//int16_t mouse_z_min;

static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl, RC_ctrl_t *rc_ctrl_old);

// function to check the control mode
void check_control_mode_change(RC_ctrl_t *rc_ctrl, RC_ctrl_t *rc_ctrl_old);

//remote control data 
RC_ctrl_t rc_ctrl;
RC_ctrl_t rc_ctrl_old;
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];


/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

const RC_ctrl_t *get_old_remote_control_point(void)
{
    return &rc_ctrl_old;
}


uint8_t RC_data_is_error(void)
{
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}


void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl, &rc_ctrl_old);
							  //detect_hook(DBUS_TOE);
								
// no detect for now                detect_hook(DBUS_TOE);
// no print on pc for now               sbus_to_usart1(sbus_rx_buf[0]);
							
// robot state definition
								check_control_mode_change(&rc_ctrl, &rc_ctrl_old);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl, &rc_ctrl_old);
							  //detect_hook(DBUS_TOE);
// no detect for now                detect_hook(DBUS_TOE);
// no print on pc for now                sbus_to_usart1(sbus_rx_buf[1]);
							
// robot state definition
								check_control_mode_change(&rc_ctrl, &rc_ctrl_old);
            }
        }
    }
	HAL_NVIC_DisableIRQ(USART3_IRQn);
}


static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl, RC_ctrl_t *rc_ctrl_old)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
		
		// copy current RC into old RC (for next iterations)
		for (int i = 0; i < 5; i++) {
			rc_ctrl_old->rc.ch[i] = rc_ctrl->rc.ch[i];
		}
		for (int i = 0; i < 2; i++) {
			rc_ctrl_old->rc.s[i] = rc_ctrl->rc.s[i];
		}
		rc_ctrl_old->key.v = rc_ctrl->key.v;
		rc_ctrl_old->mouse.x = rc_ctrl->mouse.x;
		rc_ctrl_old->mouse.y = rc_ctrl->mouse.y;
		rc_ctrl_old->mouse.z = rc_ctrl->mouse.z;
		rc_ctrl_old->mouse.press_l = rc_ctrl->mouse.press_l;
		rc_ctrl_old->mouse.press_r = rc_ctrl->mouse.press_r;

		// update rc_ctrl values
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

//for printing over USART1
//		sprintf(usart1_print_rc_ctrl,"%d\n\r", rc_ctrl->mouse.x);
//		HAL_UART_Transmit(&huart1, (uint8_t *)&usart1_print_rc_ctrl, sizeof(usart1_print_rc_ctrl), 10);
		
//		mouse_x = rc_ctrl->mouse.x;
//		mouse_y = rc_ctrl->mouse.y;
//		mouse_z = rc_ctrl->mouse.z;
//		
//		if (mouse_x_max < mouse_x)
//		{
//			mouse_x_max = mouse_x;
//		}
//		
//		if (mouse_y_max < mouse_y)
//		{
//			mouse_y_max = mouse_y;
//		}
//		
//		if (mouse_z_max < mouse_z)
//		{
//			mouse_z_max = mouse_z;
//		}
//		
//		if (mouse_x_min > mouse_x)
//		{
//			mouse_x_min = mouse_x;
//		}
//		
//		if (mouse_y_min > mouse_y)
//		{
//			mouse_y_min = mouse_y;
//		}
//		
//		if (mouse_z_min > mouse_z)
//		{
//			mouse_z_min = mouse_z;
//		}
}

int is_key_pressed(uint16_t key) {
	
	if (get_remote_control_point()->key.v & key)
		return 1;
	else
		return 0;
}

int is_key_falling_edge(uint16_t key) {
	
	// check if we just released the key 
	if (!(get_remote_control_point()->key.v & key) &&
			 (get_old_remote_control_point()->key.v & key))
		return 1;
	else
		return 0;
}

int is_key_raising_edge(uint16_t key) {
	
	// check if we just pressed the key 
	if ((get_remote_control_point()->key.v & key) &&
			!(get_old_remote_control_point()->key.v & key))
		return 1;
	else
		return 0;
}

int is_any_WASD_key_pressed() {
	
	if (is_key_pressed(KEY_W) || is_key_pressed(KEY_A) || is_key_pressed(KEY_S) || is_key_pressed(KEY_D))
		return 1;
	else
		return 0;
}

int is_mouse_left_key_pressed() {
	
	return (int) get_remote_control_point()->mouse.press_l;
}

int is_mouse_right_key_pressed() {
	
	return (int) get_remote_control_point()->mouse.press_r;
}

int is_mouse_falling_edge(uint8_t mouse_key) {
	
	if(mouse_key == MOUSE_LEFT_KEY)
	{
		if (!(get_remote_control_point()->mouse.press_l) &&
				 (get_old_remote_control_point()->mouse.press_l))
			return 1;
		else
			return 0;
	}
	else if(mouse_key == MOUSE_RIGHT_KEY)
	{
		if (!(get_remote_control_point()->mouse.press_r) &&
			 (get_old_remote_control_point()->mouse.press_r))
			return 1;
		else
			return 0;
	}
}

int is_mouse_raising_edge(uint8_t mouse_key) {
	
	if(mouse_key == MOUSE_LEFT_KEY)
	{
		if ((get_remote_control_point()->mouse.press_l) &&
				!(get_old_remote_control_point()->mouse.press_l))
			return 1;
		else
			return 0;
	}
	else if(mouse_key == MOUSE_RIGHT_KEY)
	{
		if ((get_remote_control_point()->mouse.press_r) &&
				!(get_old_remote_control_point()->mouse.press_r))
			return 1;
		else
			return 0;		
	}
}


/**
  * @brief          send sbus data by usart1, called in usart3_IRQHandle
  * @param[in]      sbus: sbus data, 18 bytes
  * @retval         none
  */
void sbus_to_usart1(uint8_t *sbus)
{
    static uint8_t usart_tx_buf[20];
    static uint8_t i =0;
    usart_tx_buf[0] = 0xA6;
    memcpy(usart_tx_buf + 1, sbus, 18);
    for(i = 0, usart_tx_buf[19] = 0; i < 19; i++)
    {
        usart_tx_buf[19] += usart_tx_buf[i];
    }
// no print on pc for now    usart1_tx_dma_enable(usart_tx_buf, 20);
}

void check_control_mode_change(RC_ctrl_t *rc_ctrl, RC_ctrl_t *rc_ctrl_old){
	
	if ( (rc_ctrl->rc.s[0] != rc_ctrl_old->rc.s[0]) || (flag_error_mode == 1) || is_key_falling_edge(KEY_R) /*((rc_ctrl->key.v == 0 << 14) && (rc_ctrl_old->key.v == KEY_V))*/ )
	{
		if ( rc_ctrl->rc.s[0] == RC_SW_DOWN || flag_error_mode == 1 )
			new_robot_control_mode = ERROR_STOP_MODE;
		else if ( rc_ctrl->rc.s[0] == RC_SW_UP )
			new_robot_control_mode = REMOTE_CONTROLLER_MODE;
		else if ( rc_ctrl->rc.s[0] == RC_SW_MID )
		{
			if (is_key_falling_edge(KEY_R))
			{
				if (robot_control_mode == CHASSIS_FOLLOW_GIMBAL_PC_MODE)
					new_robot_control_mode = (IS_STD || IS_SENTRY) ? FIXED_GIMBAL_PC_MODE : BALANCING_90_DEGREE_PC_MODE;
				else if (robot_control_mode == (IS_STD || IS_SENTRY) ? FIXED_GIMBAL_PC_MODE : BALANCING_90_DEGREE_PC_MODE)
					new_robot_control_mode = CHASSIS_FOLLOW_GIMBAL_PC_MODE;
				else
					new_robot_control_mode = CHASSIS_FOLLOW_GIMBAL_PC_MODE;
			}
			else
				new_robot_control_mode = (IS_STD || IS_SENTRY) ? FIXED_GIMBAL_PC_MODE : BALANCING_90_DEGREE_PC_MODE;
			
//			if ( rc_ctrl->rc.s[0] == RC_SW_MID && flag_error_mode == 0 )
//			{
//				new_robot_control_mode = CHASSIS_FOLLOW_GIMBAL_PC_MODE;
//			}
//			
//			if ( rc_ctrl->rc.s[0] == RC_SW_MID && flag_error_mode == 0 && rc_ctrl->key.v == 0 << 14 && rc_ctrl_old->key.v == KEY_V)
//			{
//				new_robot_control_mode = FIXED_GIMBAL_PC_MODE;
//			}
		}
		else
			return;
	}
							
}
