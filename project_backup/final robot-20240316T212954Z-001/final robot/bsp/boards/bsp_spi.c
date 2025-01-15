#include "bsp_spi.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;
//extern DMA_HandleTypeDef hdma_spi1_rx;
//extern DMA_HandleTypeDef hdma_spi1_tx;

//void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num)
//{
//    SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);			// (C |= 2 is same as C = C | 2) which is the operation of set bit, which means that we enable the TX throught DMA in SPI1 through the control register setting it as CR+TXDMAEN
//    SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN);			// enable of the RX throught DMA in SPI1 through the control register

//    __HAL_SPI_ENABLE(&hspi1);														// enable of the SPI


//    //DISABLE DMA FOR RECEIVING
//    //失效DMA
//    __HAL_DMA_DISABLE(&hdma_spi1_rx);										// disable of DMA for RX
//    
//    while(hdma_spi1_rx.Instance->CR & DMA_SxCR_EN)			// control if the DMA has effectively been disabled (if not does again the disabling)
//    {
//        __HAL_DMA_DISABLE(&hdma_spi1_rx);								// redo disabling
//    }

//    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx, DMA_LISR_TCIF2);			// TCIFx: Stream x transfer complete interrupt flag. if set to 0 we have no transfer complete event and with 1 we have transfer complete event (for now no transfer complete event)

//    hdma_spi1_rx.Instance->PAR = (uint32_t) & (SPI1->DR);			// PAR is DMA stream x peripheral address register. so we define hdma_spi1_rx.Instance->PAR as if it is SPI1->DR  (memory buffer 1)
//    //memory buffer 1
//    //内存缓冲区1
//    hdma_spi1_rx.Instance->M0AR = (uint32_t)(rx_buf);					// M0AR is DMA stream x memory 0 address register and is associated to the buffer for rx   (data lenght)
//    //data length
//    //数据长度
//    __HAL_DMA_SET_COUNTER(&hdma_spi1_rx, num);								// Writes the number of data units to be transferred on the DMA Stream.

//    __HAL_DMA_ENABLE_IT(&hdma_spi1_rx, DMA_IT_TC);						// ENABLE SPECIFIED INTERRUPT: DMA_IT_TC: Transfer complete interrupt mask


//    //DISABLE DMA	FOR TRANSFERING				
//    //失效DMA
//    __HAL_DMA_DISABLE(&hdma_spi1_tx);													// transfer disable
//    
//    while(hdma_spi1_tx.Instance->CR & DMA_SxCR_EN)						// retry to disable if somthing went wrong
//    {
//        __HAL_DMA_DISABLE(&hdma_spi1_tx);											// 
//    }


//    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_tx, DMA_LISR_TCIF3);			// transfer complete interrupt disabled

//    hdma_spi1_tx.Instance->PAR = (uint32_t) & (SPI1->DR);			// (AS BEFORE)
//    //memory buffer 1
//    //内存缓冲区1
//    hdma_spi1_tx.Instance->M0AR = (uint32_t)(tx_buf);
//    //data length
//    //数据长度
//    __HAL_DMA_SET_COUNTER(&hdma_spi1_tx, num);


//}

//void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr)
//{
//    //disable DMA
//    //失效DMA
//    __HAL_DMA_DISABLE(&hdma_spi1_rx);
//    __HAL_DMA_DISABLE(&hdma_spi1_tx);
//    while(hdma_spi1_rx.Instance->CR & DMA_SxCR_EN)
//    {
//        __HAL_DMA_DISABLE(&hdma_spi1_rx);
//    }
//    while(hdma_spi1_tx.Instance->CR & DMA_SxCR_EN)
//    {
//        __HAL_DMA_DISABLE(&hdma_spi1_tx);
//    }
//    //clear flag
//    //清除标志位
//    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));
//    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmarx));
//    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmarx));
//    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi1.hdmarx));
//    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmarx));

//    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmatx));
//    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmatx));
//    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmatx));
//    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi1.hdmatx));
//    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmatx));
//    //set memory address
//    //设置数据地址
//    hdma_spi1_rx.Instance->M0AR = rx_buf;
//    hdma_spi1_tx.Instance->M0AR = tx_buf;
//    //set data length
//    //设置数据长度
//    __HAL_DMA_SET_COUNTER(&hdma_spi1_rx, ndtr);
//    __HAL_DMA_SET_COUNTER(&hdma_spi1_tx, ndtr);
//    //enable DMA
//    //使能DMA
//    __HAL_DMA_ENABLE(&hdma_spi1_rx);
//    __HAL_DMA_ENABLE(&hdma_spi1_tx);
//}




