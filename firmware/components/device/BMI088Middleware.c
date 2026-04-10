#include "BMI088Middleware.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_delay.h"
extern SPI_HandleTypeDef hspi1;

void BMI088_GPIO_init(void)
{

}

void BMI088_com_init(void)
{


}

void BMI088_delay_ms(uint16_t ms)
{

    osDelay(ms);
}

void BMI088_delay_us(uint16_t us)
{
    delay_us(us);
}




void BMI088_ACCEL_NS_L(void)																											// reset SPI related pin (because it is the communicaiton protocol), for starting tx,rx
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)																											// set SPI related pin (because it is the communicaiton protocol), for ending tx,rx
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)															// this function allows to read and write at the same time. We pass to the function the data that we want to write and it return the data that he read
{																																						// usually we perform 2 consecutive BMI088_read_write_byte because in the first we send the interested register to read/write and in the second we send the data to write in the register
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);  //read 1 byte from SPI1 with 1000ms of tiimeout
    return rx_data;
}

