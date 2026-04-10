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

#include "struct_typedef.h"
#include <string.h>
#include "main.h"
#include "bsp_usart.h"


#define SYNC_BYTE 1 //insert 1 when using AI, 49 with ASCII (serial monitor)
#define ERR_BYTE 0 //insert 0 when using AI, 48 with ASCII (serial monitor)

extern uint8_t AI_rxBuffer[13];
extern fp32 get_AI_ref0(void);
extern fp32 get_AI_ref1(void);
extern fp32 get_AI_ref2(void);

void AI_receive_init(void);
void from_binary_to_float(uint8_t* data, fp32 *f0, fp32 *f1, fp32 *f2, bool *done);
