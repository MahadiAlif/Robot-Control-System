/**
  ****************************ROBOTO TEAM***************************************
  * @file       AI_receive.c/h
  * @brief      script to receive and send data through the UART to the AI
  *             
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#include "AI_receive.h"
#include "referee.h"



extern UART_HandleTypeDef huart1;
uint8_t AI_rxBuffer[13] = {0};
fp32 AI_ref[3] = {0};
bool_t firstRx = false;

uint8_t miniPC_RX_buf[69];
uint8_t miniPC_data[69];
uint8_t miniPC_data_temp[69];
static uint32_t start, stop, time_elapsed;


void AI_receive_init(void)
{
	uint8_t robot_id = get_robot_id(); //107 blue, 7 red
	uint8_t isBlue = 3;
	if (robot_id == 107)
		isBlue = 2;	// 1
	else if(robot_id == 7)
		isBlue = 1;	// 0
  HAL_UART_Transmit(&huart1,&isBlue,1, 100);
}

void from_binary_to_float(uint8_t* data, fp32 *f0, fp32 *f1, fp32 *f2, bool *done)
{
	 uint32_t tmp0, tmp1, tmp2;
	 uint8_t i;
	
   tmp0  = (uint32_t)data[0] << 0;
   tmp0 |= (uint32_t)data[1] << 8;
   tmp0 |= (uint32_t)data[2] << 16;
   tmp0 |= (uint32_t)data[3] << 24;
	
	 tmp1  = (uint32_t)data[4] << 0;
   tmp1 |= (uint32_t)data[5] << 8;
   tmp1 |= (uint32_t)data[6] << 16;
   tmp1 |= (uint32_t)data[7] << 24;
	
	 tmp2  = (uint32_t)data[8] << 0;
   tmp2 |= (uint32_t)data[9] << 8;
   tmp2 |= (uint32_t)data[10] << 16;
   tmp2 |= (uint32_t)data[11] << 24;
		
	 i = data[12];
	 
	 if(i==SYNC_BYTE)
	 {
     *done=true;
   }
	 else
   {
     *done=false;
   }
		
	
   memcpy(f0, &tmp0, sizeof(float));
	 memcpy(f1, &tmp1, sizeof(float));
	 memcpy(f2, &tmp2, sizeof(float));
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		memcpy(miniPC_data_temp, miniPC_RX_buf, 69*sizeof(char));
		memcpy(miniPC_data, &miniPC_data_temp[56], 13*sizeof(char));
		memcpy(&miniPC_data[13], miniPC_data_temp, 56*sizeof(char));
	
		stop = HAL_GetTick();
		time_elapsed = stop - start;
    HAL_UART_Receive_DMA(&huart1, miniPC_RX_buf, 69);		
		//HAL_UART_Receive_DMA(&huart1, RX_buf, 13);
		start = HAL_GetTick();
}


fp32 get_AI_ref0(void)
{
    return AI_ref[0];
}

fp32 get_AI_ref1(void)
{
    return AI_ref[1];
}

fp32 get_AI_ref2(void)
{
    return AI_ref[2];
}