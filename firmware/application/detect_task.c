/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       detect_task.c/h
  * @brief      detect error task, judged by receiving data time. provide detect
                hook function, error exist function.
  *             检测错误任务， 通过接收数据时间来判断.提供 检测钩子函数,错误存在函数.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add oled, gyro accel and mag sensors
  *
  @verbatim
  ==============================================================================
    add a sensor 
    1. in detect_task.h, add the sensor name at the end of errorList,like
    enum errorList
    {
        ...
        XXX_TOE,    //new sensor
        ERROR_LIST_LENGHT,
    };
    2.in detect_init function, add the offlineTime, onlinetime, priority params,like
        uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };
    3. if XXX_TOE has data_is_error_fun ,solve_lost_fun,solve_data_error_fun function, 
        please assign to function pointer.
    4. when XXX_TOE sensor data come, add the function detect_hook(XXX_TOE) function.
    如果要添加一个新设备
    1.第一步在detect_task.h，添加设备名字在errorList的最后，像
    enum errorList
    {
        ...
        XXX_TOE,    //新设备
        ERROR_LIST_LENGHT,
    };
    2.在detect_init函数,添加offlineTime, onlinetime, priority参数
        uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };
    3.如果有data_is_error_fun ,solve_lost_fun,solve_data_error_fun函数，赋值到函数指针
    4.在XXX_TOE设备数据来的时候, 添加函数detect_hook(XXX_TOE).
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
  
#include "detect_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "control_alg.h"
#include "robot_config.h"



/**
  * @brief          init error_list, assign  offline_time, online_time, priority.
  * @param[in]      time: system time
  * @retval         none
  */
/**
  * @brief          初始化error_list,赋值 offline_time, online_time, priority
  * @param[in]      time:系统时间
  * @retval         none
  */
static void detect_init(uint32_t time);





error_t error_list[ERROR_LIST_LENGHT + 1];
fp32 freq[ERROR_LIST_LENGHT] = {1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};


uint32_t system_time_detect[2] = {0,0};
fp32 frequency_detect;
fp32 freq_detect = 1000;

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t detect_task_stack;
#endif


/**
  * @brief          detect task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          检测任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */

/* Detect task disabled because of unknown error */
void detect_task(void const *pvParameters)
{
    static uint32_t system_time;
    system_time = xTaskGetTickCount(); //get current time
    detect_init(system_time);
	  
	  if(IS_BR1 || IS_BR2) 
		{
			error_list[3].enable = 0;
			error_list[4].enable = 0;
		}	
//		else if(IS_SENTRY)
//		{
//			
//		}
		
    vTaskDelay(DETECT_TASK_INIT_TIME);

    while (1)
    {
        system_time = xTaskGetTickCount();

        error_list[ERROR_LIST_LENGHT].error_exist = 0;

        for (int i = 0; i < ERROR_LIST_LENGHT; i++)
        {
            //disable, continue
            if (error_list[i].enable == 0)
            {
                continue;
            }

            //judge offline.
            if (system_time - error_list[i].new_time > error_list[i].set_offline_time)
            {
                if (error_list[i].error_exist == 0)
                {
                    //record error
                    error_list[i].error_exist = 1;
									  //error_list[ERROR_LIST_LENGHT].error_exist = 0;
									  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,1);
                }

                error_list[ERROR_LIST_LENGHT].error_exist = 1;
            }

            else
            {
                error_list[i].error_exist = 0;
                //calc frequency
                if (error_list[i].new_time > error_list[i].last_time && xTaskGetTickCount() > 5000)
                {
                    
									error_list[i].frequency = configTICK_RATE_HZ / (fp32)(error_list[i].new_time - error_list[i].last_time);
									freq[i] = (freq[i] <= error_list[i].frequency) ? freq[i]:error_list[i].frequency;
                }
            }
											
						
        }
				
				if (error_list[ERROR_LIST_LENGHT].error_exist == 0)
				{
					HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,0);
				}
				
				if (system_time_detect[1] > system_time_detect[0])
		    {    
									frequency_detect = configTICK_RATE_HZ / (fp32)(system_time_detect[1]-system_time_detect[0]);
			    if (xTaskGetTickCount() > 5000)
			    {
					   				freq_detect = (freq_detect <= frequency_detect) ? freq_detect:frequency_detect;
			    }
        }
				
				

        osDelay(1);
#if INCLUDE_uxTaskGetStackHighWaterMark
        detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


/**
  * @brief          get toe error status
  * @param[in]      toe: table of equipment
  * @retval         true (eror) or false (no error)
  */
/**
  * @brief          获取设备对应的错误状态
  * @param[in]      toe:设备目录
  * @retval         true(错误) 或者false(没错误)
  */
bool_t toe_is_error(uint8_t toe)
{
    return (error_list[toe].error_exist == 1);
}

/**
  * @brief          record the time
  * @param[in]      toe: table of equipment
  * @retval         none
  */
/**
  * @brief          记录时间
  * @param[in]      toe:设备目录
  * @retval         none
  */
void detect_hook(uint8_t toe)
{
    error_list[toe].last_time = error_list[toe].new_time;
    error_list[toe].new_time = xTaskGetTickCount();

}

/**
  * @brief          get error list
  * @param[in]      none
  * @retval         the point of error_list
  */
/**
  * @brief          得到错误列表
  * @param[in]      none
  * @retval         error_list的指针
  */
const error_t *get_error_list_point(void)
{
    return error_list;
}

extern void OLED_com_reset(void);
static void detect_init(uint32_t time)
{

    uint16_t set_item[ERROR_LIST_LENGHT] = {50,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};
    for (uint8_t i = 0; i < ERROR_LIST_LENGHT; i++)
    {
        error_list[i].set_offline_time = set_item[i];


        error_list[i].enable = 1;
        error_list[i].error_exist = 0;
        error_list[i].frequency = 0.0f;
        error_list[i].new_time = time;
        error_list[i].last_time = time;
    }


}
