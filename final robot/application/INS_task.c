/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "INS_task.h"
#include "control_alg.h"
#include "robot_config.h"

#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)
#define SAMPLE_PERIOD (0.001f)
#define SAMPLE_RATE (1/SAMPLE_PERIOD)


/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
static void imu_temp_control(fp32 temp);


void get_angle(fp32 *yaw, fp32 *pitch, fp32 *roll);

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

bmi088_real_data_t bmi088_real_data;
ist8310_real_data_t ist8310_real_data;

static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;

int INS_angle_counter[3] = {0, 0, 0}; 
fp32 ins_correct_angle[3] = {0.0f, 0.0f, 0.0f};
fp32 gyro_temperature;
uint64_t counter_gyro;

static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};

bool_t bmi088_ist3810_init_flag = 1;  // flag used to perform only once the bmi088 initialization

// char variables useful for UART print, check if 30 is too much
char angle_print_x[30];
char angle_print_y[30];
char angle_print_z[30];
char accel_print_x[30];
char accel_print_y[30];
char accel_print_z[30];
char gyro_temperature_print[30];
char counter_gyro_print[30];

// raw angles
fp32 raw_angles[3];
fp32 raw_angles_prev[3];

// angular velocity and linear accelerations of the board, manually aligned with the axes of the gimbal's reference frame
float gx, gy, gz;
float ax, ay, az;

// coefficients to correct the drift of angles
float correction_drift_coeff_roll = 0;
float correction_drift_coeff_pitch = 0;
#if IS_STD
float correction_drift_coeff_yaw = 0;
#elif IS_BR2
float correction_drift_coeff_yaw = -0.00016;
#elif IS_SENTRY
float correction_drift_coeff_yaw = 0;
#endif


// values for the old code for board's attitude estimation
float Roll, Pitch, Yaw;
float Roll_dot, Pitch_dot, Yaw_dot;
float Roll_cos;
float Roll_sin;
float Pitch_cos;
float Pitch_sin;
float euler_dt = 0.001f;
int flag_init_euler = 1;




/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void INS_task(void const *pvParameters)
{
	
	const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
	const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
	const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
	const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
	const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
	const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
	
 // Initialise the AHRS algo that will perform the sensor fusion
	FusionOffset offset;
	FusionAhrs ahrs;

	//FusionOffsetInitialise(&offset, SAMPLE_RATE);
	FusionAhrsInitialise(&ahrs);

	// Set AHRS algorithm settings
	const FusionAhrsSettings settings = {
					.gain = 0.5f,
					.accelerationRejection = 10.0f,
					.rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
	};
	FusionAhrsSetSettings(&ahrs, &settings);
	
	while(1)
	{ 
		if (bmi088_ist3810_init_flag == 1){
		
			while(BMI088_init())											 
			{
        osDelay(100);
			}
		
		while(ist8310_init())											// init of the BMI088 peripheral
			{
        osDelay(100);
			}
			
			// initialization for the PID
			PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
						
			bmi088_ist3810_init_flag = 0;
		}
		
		// read data from sensors
		BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);	
		gyro_temperature = bmi088_real_data.temp;
		
		/*
		//									^ Axis 0
		//									|
		//									|
		//						--------------
		//						|						 |
		//						|			R 		 |
		//			 <----|	  				 |
		//	Axis 1		| Robomaster |
		//						|		  			 |		
		//						--------------
		//									
		//								 (o) Axis 2		(for Axis 2, right-hand rule with Axis 0 and Axis 1)
		*/
		
		ax = bmi088_real_data.accel[0];
		ay = bmi088_real_data.accel[1];
		az = bmi088_real_data.accel[2];
		
		gx = bmi088_real_data.gyro[0];
		gy = bmi088_real_data.gyro[1];
		gz = bmi088_real_data.gyro[2];
		
		//------ START SENSOR FUSION ------		
		FusionVector gyroscope = {gx*57.297469, gy*57.297469, gz*57.297469}; // degrees/s
	  FusionVector accelerometer = {ax/9.81, ay/9.81, az/9.81}; // in g
		
		 // Apply calibration
		gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
		accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
		
		// Update gyroscope offset correction algorithm
    gyroscope = FusionOffsetUpdate(&offset, gyroscope);
		
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
		
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));		
		//------- END SENSOR FUSION -------
			
		if (flag_init_euler) {
			
			Pitch = 0;
			Roll = 0;
			Yaw = 0;
			flag_init_euler = 0;
		}
		
		//Roll = euler.angle.roll*pi/180;				// per far funzionare il Pitch del vecchio codice, scommentare
		//Roll = 0;
		
		Roll_cos = cos(Roll);
		Roll_sin = sin(Roll);
		Pitch_cos = cos(Pitch);
		Pitch_sin = sin(Pitch);
		
		Pitch_dot = (Roll_cos*gx+Roll_sin*Pitch_sin*gy+Roll_sin*Pitch_cos*gz)/Roll_cos;
		Pitch = Pitch + Pitch_dot*euler_dt;
		
		Roll_dot = (Pitch_cos*Roll_cos*gy-Pitch_sin*Roll_cos*gz)/Roll_cos;
		Roll = Roll + Roll_dot*euler_dt;
		
		Yaw_dot = (Pitch_sin*gy+Pitch_cos*gz)/Roll_cos;
		Yaw = Yaw + Yaw_dot*euler_dt;
		
		
		//-------- START CONTIGOUS ANGLES TRANSFORMATION (i.e. >180°) --------
		raw_angles[0] = (USE_NEW_MAHONY_PITCH) ? euler.angle.pitch : Pitch*180/pi;
		raw_angles[1] = (USE_NEW_MAHONY_ROLL) ? euler.angle.roll : Roll*180/pi;
		raw_angles[2] = (USE_NEW_MAHONY_YAW) ? euler.angle.yaw : Yaw*180/pi;
		
		for (int i = 0; i < 3; i++)
		{
			if (raw_angles[i] * raw_angles_prev[i] < (-27000))
			{
				if (raw_angles[i] < 0)					// zone 1 --> 2 , 3 --> 4
					INS_angle_counter[i] += 1;
				else if (raw_angles[i] > 0)			// zone 2 --> 1 , 4 --> 3
					INS_angle_counter[i] -= 1;
			}
			
			raw_angles_prev[i] = raw_angles[i];
		}
		//--------- END CONTIGOUS ANGLES TRANSFORMATION (i.e. >180°) ---------
		
		// correct angles values to be used in other tasks
		ins_correct_angle[0] = INS_angle_counter[0]*360 + raw_angles[0] + correction_drift_coeff_pitch*counter_gyro;
		ins_correct_angle[1] = INS_angle_counter[1]*360 + raw_angles[1] + correction_drift_coeff_roll*counter_gyro;
		ins_correct_angle[2] = INS_angle_counter[2]*360 + raw_angles[2] + correction_drift_coeff_yaw*counter_gyro;
				
		counter_gyro ++;
		  
//		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
		// TODO add the same as previuous instruction, but for hcan2
		HAL_NVIC_EnableIRQ(USART3_IRQn);
  	osDelay(1);
	}
}



static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, 45.0f);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //in beginning, max power
        if (temp > 45.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}