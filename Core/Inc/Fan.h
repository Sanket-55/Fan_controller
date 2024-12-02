ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE END Private defines */
#include "stdint.h"
uint8_t Door_status;
uint8_t Smoke_sensor;
uint8_t Digital_input_1;
uint8_t Digital_input_2;
uint8_t Digital_input_3;
uint8_t Digital_input_4;
uint8_t Digital_input_5;
uint8_t Digital_input_6;
uint8_t Digital_input_7;
uint8_t Digital_input_8;
uint8_t Button_1;
uint8_t Button_2;
uint8_t Button_3;
uint8_t Button_4;
uint8_t Button_5;
uint8_t Button_6;
uint16_t Temperature_sensor;
uint16_t Ambient_sensor_1;
uint16_t Ambient_sensor_2;
uint16_t Adc_buff[3];
uint8_t Adc_Complition_flag =0;
uint8_t Temprature_threshold;
void Read_ADC_value();
void Read_sensors();
void Set_Fan_speed();
void set_alarm();
