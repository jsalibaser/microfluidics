#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_uart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "main.h"
#include "i2c-lcd.h"


//THESE VALUES ARE FOR HEATING

// Thermistor & ADC Configuration
#define BETA 3950   // Beta coefficient of thermistor
#define R0 3940.0 // Reference resistance thermistor
#define R_REF 99100.0 // Reference resistor (5kΩ)
#define T0 304.95   // Reference Temperature (Kelvin)
#define V_REF 3.29  // Reference Voltage (3.3V)
#define A 4.478028e-03  // Reference Voltage (3.3V)
#define B -3.296712e-04  // Reference Voltage (3.3V)
#define C 1.777459e-06  // Reference Voltage (3.3V)


//THE FOLLOWING ARE VARIABLES NEEDED FOR THE HEATING
double setpoint = 65.0;  // Target temperature in Celsius
double input, output;
double integral = 0, lastError = 0;
uint32_t pwm_value = 0;

// PID Constants (Tune as Needed)
double Kp = 18.0;
double Ki = 0.9;
double Kd = 6.0;

//VALUES FOR HEATING ENDS HERE


#define AS7343_I2C_ADDRESS  (0x39 << 1)  // I2C address shifted for HAL compatibility
GPIO_PinState btn_state;


I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
ADC_HandleTypeDef hadc1;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void UART_Printf(const char *format, ...);
HAL_StatusTypeDef AS7343_WriteRegister(uint8_t reg, uint8_t value);
HAL_StatusTypeDef AS7343_ReadRegister(uint8_t reg, uint8_t *value);
void AS7343_Init(void);
void AS7343_ReadSpectralData(void);
void CheckSensorConnection(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void setTCAChannel(uint8_t i);
static void MX_ADC1_Init(void);
void Get_Time(char *buffer);
double readTemperature(void);
double computePID(double temp);
void heatingLoop_func(void);


//values for initialization
uint8_t gain_value = 0x09; // Default gain value
volatile uint8_t button_pressed = 0;
uint16_t channel_data[14];

//values for integration time
uint32_t atime = 0x28;
uint32_t astep = 10; //default value for ASTEP
uint16_t d4Val = 0x0A; //default value for D4
uint16_t d5Val = 0x00; //default value for D5
uint32_t measuringTime; //time it takes to measure 1 sample
float desiredTime = 1000.0f; //desired int Time in (ms)

uint16_t gain_values[13] = {0.5, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048};
char *sorted_wl[] = {"405nm","425nm","450nm","475nm","515nm","550nm","555nm","600nm","640nm","690nm","745nm","855nm"};
uint8_t wl_value = 0x06;

//values for timer
uint16_t arr;
uint16_t psc;
uint16_t pulse;
float dutyCycle;

uint16_t mode = 7; //set to -1 as -1 is the heating part, then mode 0 onwards are for the sensor

uint8_t ckr2 = 0,ckr1 = 0, cMode8 = 0, cTime = 0, cFlb = 0, cResetPID = 0; //checker for TIM init and AS7343 init


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  static uint32_t last_interrupt_time = 0;  // Store last interrupt time
  uint32_t current_time = HAL_GetTick();   // Get current system time

  // Debounce check: Ignore interrupts that occur too quickly (within 200ms)

  if ((current_time - last_interrupt_time) < 500) {
	  return;
  }
  last_interrupt_time = current_time; // Update last interrupt time

  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

  if (mode == 7){
	  //HAL_Delay(500);
	  if(GPIO_Pin == GPIO_PIN_13){
		  mode = 8;
	  }
	  else{
		  return;
	  }
  }
  else if (mode == 8){
	  if(GPIO_Pin == GPIO_PIN_13){
		  mode = 9;
	  }
	  else{
		  mode = 7;
	  }
  }
  else if (mode == 9){
	  if(GPIO_Pin == GPIO_PIN_13){
		  mode = 0;
		  //stop the timer
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		  lcd_clear();

		  //HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		  //
		  //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		  UART_Printf("HEATING ENDS! WAIT 2 Seconds\r\n");

		  //HAL_Delay(2000);

		  //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		  //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1);

		  UART_Printf("Set the GAIN value\r\n");
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("SET GAIN:");
		  char str[10];
		  sprintf(str, "%u", gain_values[gain_value]);
		  lcd_send_cmd (0x80|0x0A);
		  lcd_send_string(strcat(str,"x"));
	  }
	  else{
		  mode = 8;
	  }

  }
  else if(mode == 0){
	  if(GPIO_Pin == GPIO_PIN_13){
		  mode = 1;
		  lcd_clear();
		  UART_Printf("Set the INT TIME value\r\n");
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("                    ");
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("SET TINT:");
		  char str[10];
		  sprintf(str, "%.0f", desiredTime);
		  lcd_send_string("          ");
		  lcd_send_cmd (0x80|0x0A);
		  lcd_send_string(strcat(str,"ms"));
	  }
	  else{
		  mode = 9;
	  }
  }
  else if(mode == 1){
	  if(GPIO_Pin == GPIO_PIN_13){
		  mode = 2;
		  lcd_clear();
		  UART_Printf("Set WAVELENGTH\r\n");
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("                    ");
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("SET WAVELENGTH:");
		  lcd_send_cmd (0x80|0x0F);
		  lcd_send_string(strcat(sorted_wl[wl_value],"nm"));
	  }
	  else{
		  mode = 0;
		  //stop the timer
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);


		  lcd_clear();

		  //HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		  //
		  //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		  UART_Printf("HEATING ENDS! WAIT 2 Seconds\r\n");

		  //HAL_Delay(2000);

		  //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		  //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1);

		  UART_Printf("Set the GAIN value\r\n");
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("SET GAIN:");
		  char str[10];
		  sprintf(str, "%u", gain_values[gain_value]);
		  lcd_send_cmd (0x80|0x0A);
		  lcd_send_string(strcat(str,"x"));
	  }
  }
  else if(mode == 2){
	  if(GPIO_Pin == GPIO_PIN_13){
		  mode = 3;
		  lcd_clear();
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("                    ");
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("-PRESS [>] TO START-");
	  }
	  else{
		  mode = 1;
		  lcd_clear();
		  UART_Printf("Set the INT TIME value\r\n");
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("                    ");
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("SET TINT:");
		  char str[10];
		  sprintf(str, "%.0f", desiredTime);
		  lcd_send_string("          ");
		  lcd_send_cmd (0x80|0x0A);
		  lcd_send_string(strcat(str,"ms"));
	  }
	  //UART_Printf("Initializing AS7343 sensor...\r\n");
	  //mode = 3;

  }
  else if(mode == 3){
	  if(GPIO_Pin == GPIO_PIN_13){
		  mode = 4;
		  lcd_clear();
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("                    ");
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("DETECTING...");
	  }
	  else{
		  mode = 2;
		  lcd_clear();
		  UART_Printf("Set WAVELENGTH\r\n");
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("                    ");
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("SET WAVELENGTH:");
		  lcd_send_cmd (0x80|0x0F);
		  lcd_send_string(strcat(sorted_wl[wl_value],"nm"));
	  }
  	  //UART_Printf("Initializing AS7343 sensor...\r\n");
  	  //mode = 4;
  }
  else if(mode == 4){
	  if(GPIO_Pin == GPIO_PIN_13){
		  mode = 5;
		  lcd_clear();
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("                    ");
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("-PRESS [>] TO EXIT!-");
	  }
	  else{
		  mode = 3;
		  lcd_clear();
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("                    ");
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("-PRESS [>] TO START-");
	  }
	  //mode = 5;
  }
  else{
	  if(GPIO_Pin == GPIO_PIN_13){
		  mode = 7;
	  }
	  else{
		  mode = 4;
		  lcd_clear();
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("                    ");
		  lcd_send_cmd (0x80|0x00);
		  lcd_send_string("  DETECTING VALUES  ");
	  }
	  /*
	  mode = 0;
	  UART_Printf("Set the GAIN value\r\n");
	  lcd_send_cmd (0x80|0x00);
	  lcd_send_string("                    ");
	  lcd_send_cmd (0x80|0x00);
	  lcd_send_string("SET GAIN:");
	  char str[10];
	  sprintf(str, "%u", gain_values[gain_value]);
	  lcd_send_cmd (0x80|0x0A);
	  lcd_send_string(strcat(str,"x"));
	  */
  }
  cTime = 0;
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}

//this function is for heating purposes only and will end with a press of button

int main(void) {

	//init call starts here
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_USART2_UART_Init();

	MX_TIM3_Init();
	MX_TIM4_Init();

	MX_ADC1_Init();

	//start of the pwm timer
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_Delay(1000);
	UART_Printf("START THE HEATING!!!\r\n");
	//call for the heating function to start heating
	char msg[150];
	sprintf(msg, "STM32 Activated. Monitoring Temperature, Current & Time...\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);



//	sprintf(msg, "%lu", htim3.Init.Period);  // Use %u or %lu depending on the data type
//	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

	//lcd init in this part for fluorescence detection part

	HAL_Delay(1000);
	MX_I2C1_Init();
	HAL_Delay(1000);
	lcd_init();
	HAL_Delay(1000);


	//lcd_send_cmd (0x80|0x00);
	//lcd_send_string("-PRESS [>] TO START-");
	//HAL_Delay(100);



	//declare some variables for heating here
	double input_f = readTemperature();
	double output_f = computePID(input_f);
	char set_temp_out[50];
	char temp_out[50];  // Make sure the buffer is large enough
	char time_out[50];
	sprintf(set_temp_out, "   SET TEMP:%.1fC   ", setpoint);  // .1f formats to 1 decimal places


	//lcd_send_cmd (0x80|0x14);
	//lcd_send_string(" CURRENT TEMP:");
	//HAL_Delay(350);
	//lcd_send_cmd (0x80|0x54);
	//lcd_send_string(" TIME (MM:SS):");
	HAL_Delay(700);
	uint32_t buttonPressTime = 0;
	uint32_t lastPrintTime = 0;
	char time_str[30];

	//inf loop starts here
	while(1) {
		if(mode!=8){
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		}

		if(cResetPID == 0 && input_f >= (setpoint - 5.0f)){
			integral = 0, lastError = 0;
			pwm_value = 0;
			cResetPID = 1;
		}

		if (mode == 7){
			lcd_send_cmd (0x80|0x40);
			lcd_send_string(set_temp_out);
			lcd_send_cmd (0x80|0x14);
			lcd_send_string(" CURRENT TEMP:");
			lcd_send_cmd (0x80|0x54);
			lcd_send_string(" TIME (MM:SS):");
			Get_Time(time_str);

			uint32_t currentTime = HAL_GetTick();
			input_f = readTemperature();
			output_f = computePID(input_f);

			if ((currentTime - lastPrintTime) >= 800 && ckr1 == 0){
				lcd_send_cmd (0x80|0x00);
				lcd_send_string("                    ");
				lastPrintTime = currentTime;
				ckr1 = 1;
			}
			else if((currentTime - lastPrintTime) >= 200 && ckr1 == 1){
				lcd_send_cmd (0x80|0x00);
				lcd_send_string("-PRESS [>] TO START-");
				lastPrintTime = currentTime;
				ckr1 = 0;
			}
			sprintf(temp_out, "%.1fC",input_f);
			sprintf(time_out, "%s", time_str);

			lcd_send_cmd (0x80|0x22);
			lcd_send_string(temp_out);
			lcd_send_cmd (0x80|0x62);
			lcd_send_string(time_out);
			continue;
		}
		else if (mode == 8){
			lcd_send_cmd (0x80|0x00);
			lcd_send_string(" --HEATING  STAGE-- ");

			lcd_send_cmd (0x80|0x14);
			lcd_send_string(" CURRENT TEMP:");
			lcd_send_cmd (0x80|0x54);
			lcd_send_string(" TIME (MM:SS):");

			uint32_t currentTime = HAL_GetTick();
			Get_Time(time_str);

			input_f = readTemperature();
			output_f = computePID(input_f);

			float input = (float)(input_f);
			float output = (float)(output_f);

			// Set PWM Duty Cycle (0-100%)
			pwm_value = (uint32_t)((output / 100.0f) * (htim3.Init.Period));
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);
			//sprintf(msg, "time:%s, temperature:%.2f duty cycle:%.2f \r\n", time_str, input, output);

			/*
			if ((currentTime - lastPrintTime) >= 1000){
				sprintf(msg, "time:%s, temperature:%.2f duty cycle:%.2f \r\n", time_str, input, output);
				HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
				lastPrintTime = currentTime; // Update last print time
			}
			*/


			if ((currentTime - lastPrintTime) >= 2000){
				if(ckr1 == 1){
					lcd_send_cmd (0x80|0x40);
					lcd_send_string("                    ");
					sprintf(msg, "  DUTY CYCLE:%-4.1f%%  ",output);
					lcd_send_cmd (0x80|0x40);
					lcd_send_string(msg);
					ckr1 = 0;
				}
				else{
					lcd_send_cmd (0x80|0x40);
					lcd_send_string(set_temp_out);
					ckr1 = 1;
				}
				lastPrintTime = currentTime;
			}


			sprintf(temp_out, "%.1fC",input);
			lcd_send_cmd (0x80|0x22);
			lcd_send_string(temp_out);
			sprintf(time_out, "%s", time_str);
			lcd_send_cmd (0x80|0x62);
			lcd_send_string(time_out);
			//HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
			continue;
		}

		else if(mode == 9){
			lcd_clear();
			lcd_send_cmd (0x80|0x00);
			lcd_send_string("STOP HEATING?    [>]");
			lcd_send_cmd (0x80|0x40);
			lcd_send_string("BACK TO HEATING? [<]");
			HAL_Delay(800);
			continue;
		}
		//FLUORESCENCE HERE



		else if (mode == 0){
			if(ckr2 != 0){
				UART_Printf("HEATING ENDS!!!!\r\n");
				HAL_Delay(2000);
				ckr2 = 0;
			}
			//gain
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == 1 && HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == 0){
				if(gain_value >= 12){
					gain_value = 0x0C;
					continue;
				}
				gain_value += 1;
			}
			else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == 0 && HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == 1){
				if(gain_value <= 0){
					gain_value = 0x00;
					continue;
				}
				gain_value -= 1;
			}
			else{
				continue;
			}
			char str[10];
			sprintf(str, "%u", gain_values[gain_value]);
			lcd_send_cmd (0x80|0x0A);
			lcd_send_string("     ");
			lcd_send_cmd (0x80|0x0A);
			lcd_send_string(strcat(str,"x"));
			UART_Printf("Set GAIN to: %dx\r\n", gain_values[gain_value]);
			HAL_Delay(100);
			continue;
		}
		else if (mode == 1) {
			if(ckr2 != 0){
				UART_Printf("HEATING ENDS!!!!\r\n");
				HAL_Delay(2000);
				ckr2 = 0;
			}
			//integration time
		    // Button 1 (Increase)
		    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == 1 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 0) {
		        if (buttonPressTime == 0) {
		            buttonPressTime = HAL_GetTick();  // Record time when button is first pressed
		        }
		        uint32_t pressDuration = HAL_GetTick() - buttonPressTime;
		        if (pressDuration >= 10000) {  // 10 seconds
		        	desiredTime += 500.0f;
		        }
		        else if (pressDuration >= 5000) {  // 5 seconds
		        	desiredTime += 100.0f;
		        }
		        else if (pressDuration >= 2000) {  // 2 seconds
		        	desiredTime += 10.0f;
		        }
		        else {
		        	desiredTime += 1.0f;  // Normal increment
		        }
		        if (desiredTime > 10000.0f) {
		        	desiredTime = 10000.0f;
		        	continue;
		        }
		        char str[10];
		        sprintf(str, "%.0f", desiredTime);
		        lcd_send_cmd (0x80|0x0A);
		        lcd_send_string("          ");
		        lcd_send_cmd (0x80|0x0A);
		        lcd_send_string(strcat(str,"ms"));
		        UART_Printf("Set INT TIME to: %.2f ms\r\n", desiredTime);
		    }

		    // Button 2 (Decrease)
		    else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == 0 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 1) {
		        if (buttonPressTime == 0) {
		            buttonPressTime = HAL_GetTick();  // Record time
		        }
		        uint32_t pressDuration = HAL_GetTick() - buttonPressTime;
		        if (pressDuration >= 10000) {  // 10 seconds
		        	desiredTime -= 500.0f;
		        }
		        else if (pressDuration >= 5000) {  // 5 seconds
		        	desiredTime -= 100.0f;
		        }
		        else if (pressDuration >= 2000) {  // 2 seconds
		        	desiredTime -= 10.0f;
		        }
		        else {
		        	desiredTime -= 1.0f;  // Normal increment
		        }
		        if (desiredTime < 1.0f) {
		        	desiredTime = 1.0f;
		        	continue;
		        }
		        char str[10];
		        sprintf(str, "%.0f", desiredTime);
		        lcd_send_cmd (0x80|0x0A);
		        lcd_send_string("          ");
		        lcd_send_cmd (0x80|0x0A);
		        lcd_send_string(strcat(str,"ms"));
		        UART_Printf("Set INT TIME to: %.2f ms\r\n", desiredTime);
		    }
		    else{
		    	buttonPressTime = 0;
		    	continue;
		    }
		    HAL_Delay(200);  // Small delay to prevent excessive polling
		    continue;
		}
	    else if (mode == 2){
			//wavelength
			if(ckr2 != 0){
				UART_Printf("HEATING ENDS!!!!\r\n");
				HAL_Delay(2000);
				ckr2 = 0;
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == 1 && HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == 0){
				if(wl_value >= 11){
					wl_value = 0x0B;
					continue;
				}
				wl_value += 1;
			}
			else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == 0 && HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == 1){
				if(wl_value <= 0){
					wl_value = 0x00;
					continue;
				}
				wl_value -= 1;
			}
			else{
				continue;
			}
			lcd_send_cmd (0x80|0x0F);
			lcd_send_string("     ");
			lcd_send_cmd (0x80|0x0F);
			lcd_send_string(strcat(sorted_wl[wl_value],"nm"));
			HAL_Delay(200);
			continue;
		}
		/*
		else if(mode == 2){
			//brightness
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == 1 && HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == 0){
				if(dutyCycle >= 1.0f){
					dutyCycle = 1.0f;
					continue;
				}
				dutyCycle += 0.05f;
			}
			else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == 0 && HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == 1){
				if(dutyCycle <= 0.0f){
					dutyCycle = 0.0f;
					continue;
				}
				dutyCycle -= 0.05f;
			}
			else{
				continue;
			}
			UART_Printf("Set BRIGHTNESS to: %d%%\r\n", (int)(dutyCycle*100));
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (int)((float)(pulse)*dutyCycle));
			HAL_Delay(200);
			continue;
		}
		*/
		else if(mode == 3){
			if(ckr2 == 0){
				setTCAChannel(1);
				AS7343_Init();
				setTCAChannel(2);
				AS7343_Init();
				setTCAChannel(3);
				AS7343_Init();
				setTCAChannel(4);
				AS7343_Init();
				setTCAChannel(5);
				AS7343_Init();
				setTCAChannel(6);
				AS7343_Init();
				HAL_Delay(1000);
				UART_Printf("ONE TIME DETECTION...\r\n");
				HAL_Delay(measuringTime*2);
				ckr2 = 1;
				continue;
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == 1 || HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == 1){
				AS7343_ReadSpectralData();

				UART_Printf("| %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-12s | %-10s | %-10s | %-10s | %-10s |\r\n","F1 (405nm)","F2 (425nm)","FZ (450nm)","F3 (475nm)","F4 (515nm)","FY (555nm)","F5 (550nm)","FXL (595nm)","F6 (640nm)","F7 (690nm)","F8 (745nm)", "NIR");
				UART_Printf("| %-10d | %-10d | %-10d | %-10d | %-10d | %-10d | %-10d | %-12d | %-10d | %-10d | %-10d | %-10d |\r\n\n",channel_data[8],channel_data[4],channel_data[0],channel_data[5],channel_data[6],channel_data[1],channel_data[11],channel_data[2],channel_data[7],channel_data[9],channel_data[10],channel_data[3]);
				UART_Printf("---------------------------------------------------------------------------------------------------------------------------------------------------------------\r\n\n");

				HAL_Delay(measuringTime);
			}
			continue;
		}
		else if(mode == 4){
			if(ckr2 == 1 || ckr2 == 0){
				setTCAChannel(1);
				AS7343_Init();
				setTCAChannel(2);
				AS7343_Init();
				setTCAChannel(3);
				AS7343_Init();
				setTCAChannel(4);
				AS7343_Init();
				setTCAChannel(5);
				AS7343_Init();
				setTCAChannel(6);
				AS7343_Init();
				HAL_Delay(1000);
				UART_Printf("CONTINUOUS DETECTION...\r\n");
				HAL_Delay(measuringTime*2);
				ckr2 = 2;
				continue;
			}
			Get_Time(time_str);

			uint32_t currentTime = HAL_GetTick();
			sprintf(time_out, "%s", time_str);
			lcd_send_cmd (0x80|0x0F);
			lcd_send_string(time_out);
			//uint32_t seconds = (currentTime / 1000) % 60;

			char str[15];

			if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == 1 || HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == 1) && cFlb == 0){
				lcd_send_cmd (0x80|0x00);
				lcd_send_string("               ");
				sprintf(str, "GAIN:%u", gain_values[gain_value]);
				lcd_send_cmd (0x80|0x00);
				lcd_send_string(strcat(str,"x"));
				cFlb = 1;
			}
			else if ((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == 1 || HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == 1) && cFlb == 1){
				lcd_send_cmd (0x80|0x00);
				lcd_send_string("               ");
				sprintf(str, "TINT:%.0f", desiredTime);
				lcd_send_cmd (0x80|0x00);
				lcd_send_string(strcat(str,"ms"));
				cFlb = 2;
			}
			else if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == 1 || HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == 1) && cFlb == 2){
				lcd_send_cmd (0x80|0x00);
				lcd_send_string("               ");
				sprintf(str, "WL:%s", sorted_wl[wl_value]);
				lcd_send_cmd (0x80|0x00);
				lcd_send_string(str);
				cFlb = 0;
			}


			if ((currentTime - lastPrintTime) >= measuringTime){
				lastPrintTime = currentTime; // Update last print time
			}
			else{
				continue;
			}


			//HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

			setTCAChannel(1);
			AS7343_ReadSpectralData();

			UART_Printf("| %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-12s | %-10s | %-10s | %-10s | %-10s |\r\n","F1 (405nm)","F2 (425nm)","FZ (450nm)","F3 (475nm)","F4 (515nm)","FY (555nm)","F5 (550nm)","FXL (595nm)","F6 (640nm)","F7 (690nm)","F8 (745nm)", "NIR");
			UART_Printf("| %-10d | %-10d | %-10d | %-10d | %-10d | %-10d | %-10d | %-12d | %-10d | %-10d | %-10d | %-10d |\r\n\n",channel_data[8],channel_data[4],channel_data[0],channel_data[5],channel_data[6],channel_data[1],channel_data[11],channel_data[2],channel_data[7],channel_data[9],channel_data[10],channel_data[3]);
			UART_Printf("---------------------------------------------------------------------------------------------------------------------------------------------------------------\r\n\n");

			uint16_t cdata1_updated[14] = {channel_data[8],channel_data[4],channel_data[0],channel_data[5],channel_data[6],channel_data[1],channel_data[11],channel_data[2],channel_data[7],channel_data[9],channel_data[10],channel_data[3]};

			setTCAChannel(2);
			AS7343_ReadSpectralData();
			/*
			UART_Printf("| %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-12s | %-10s | %-10s | %-10s | %-10s |\r\n","F1 (405nm)","F2 (425nm)","FZ (450nm)","F3 (475nm)","F4 (515nm)","FY (555nm)","F5 (550nm)","FXL (595nm)","F6 (640nm)","F7 (690nm)","F8 (745nm)", "NIR");
			UART_Printf("| %-10d | %-10d | %-10d | %-10d | %-10d | %-10d | %-10d | %-12d | %-10d | %-10d | %-10d | %-10d |\r\n\n",channel_data[8],channel_data[4],channel_data[0],channel_data[5],channel_data[6],channel_data[1],channel_data[11],channel_data[2],channel_data[7],channel_data[9],channel_data[10],channel_data[3]);
			UART_Printf("---------------------------------------------------------------------------------------------------------------------------------------------------------------\r\n\n");
			*/

			uint16_t cdata2_updated[14] = {channel_data[8],channel_data[4],channel_data[0],channel_data[5],channel_data[6],channel_data[1],channel_data[11],channel_data[2],channel_data[7],channel_data[9],channel_data[10],channel_data[3]};

			setTCAChannel(3);
			AS7343_ReadSpectralData();
			/*
			UART_Printf("| %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-12s | %-10s | %-10s | %-10s | %-10s |\r\n","F1 (405nm)","F2 (425nm)","FZ (450nm)","F3 (475nm)","F4 (515nm)","FY (555nm)","F5 (550nm)","FXL (595nm)","F6 (640nm)","F7 (690nm)","F8 (745nm)", "NIR");
			UART_Printf("| %-10d | %-10d | %-10d | %-10d | %-10d | %-10d | %-10d | %-12d | %-10d | %-10d | %-10d | %-10d |\r\n\n",channel_data[8],channel_data[4],channel_data[0],channel_data[5],channel_data[6],channel_data[1],channel_data[11],channel_data[2],channel_data[7],channel_data[9],channel_data[10],channel_data[3]);
			UART_Printf("---------------------------------------------------------------------------------------------------------------------------------------------------------------\r\n\n");
			*/

			uint16_t cdata3_updated[14] = {channel_data[8],channel_data[4],channel_data[0],channel_data[5],channel_data[6],channel_data[1],channel_data[11],channel_data[2],channel_data[7],channel_data[9],channel_data[10],channel_data[3]};

			setTCAChannel(4);
			AS7343_ReadSpectralData();
			/*
			UART_Printf("| %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-12s | %-10s | %-10s | %-10s | %-10s |\r\n","F1 (405nm)","F2 (425nm)","FZ (450nm)","F3 (475nm)","F4 (515nm)","FY (555nm)","F5 (550nm)","FXL (595nm)","F6 (640nm)","F7 (690nm)","F8 (745nm)", "NIR");
			UART_Printf("| %-10d | %-10d | %-10d | %-10d | %-10d | %-10d | %-10d | %-12d | %-10d | %-10d | %-10d | %-10d |\r\n\n",channel_data[8],channel_data[4],channel_data[0],channel_data[5],channel_data[6],channel_data[1],channel_data[11],channel_data[2],channel_data[7],channel_data[9],channel_data[10],channel_data[3]);
			UART_Printf("---------------------------------------------------------------------------------------------------------------------------------------------------------------\r\n\n");
			*/

			uint16_t cdata4_updated[14] = {channel_data[8],channel_data[4],channel_data[0],channel_data[5],channel_data[6],channel_data[1],channel_data[11],channel_data[2],channel_data[7],channel_data[9],channel_data[10],channel_data[3]};

			setTCAChannel(5);
			AS7343_ReadSpectralData();
			/*
			UART_Printf("| %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-12s | %-10s | %-10s | %-10s | %-10s |\r\n","F1 (405nm)","F2 (425nm)","FZ (450nm)","F3 (475nm)","F4 (515nm)","FY (555nm)","F5 (550nm)","FXL (595nm)","F6 (640nm)","F7 (690nm)","F8 (745nm)", "NIR");
			UART_Printf("| %-10d | %-10d | %-10d | %-10d | %-10d | %-10d | %-10d | %-12d | %-10d | %-10d | %-10d | %-10d |\r\n\n",channel_data[8],channel_data[4],channel_data[0],channel_data[5],channel_data[6],channel_data[1],channel_data[11],channel_data[2],channel_data[7],channel_data[9],channel_data[10],channel_data[3]);
			UART_Printf("---------------------------------------------------------------------------------------------------------------------------------------------------------------\r\n\n");
			*/

			uint16_t cdata5_updated[14] = {channel_data[8],channel_data[4],channel_data[0],channel_data[5],channel_data[6],channel_data[1],channel_data[11],channel_data[2],channel_data[7],channel_data[9],channel_data[10],channel_data[3]};

			setTCAChannel(6);
			AS7343_ReadSpectralData();
			/*
			UART_Printf("| %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-12s | %-10s | %-10s | %-10s | %-10s |\r\n","F1 (405nm)","F2 (425nm)","FZ (450nm)","F3 (475nm)","F4 (515nm)","FY (555nm)","F5 (550nm)","FXL (595nm)","F6 (640nm)","F7 (690nm)","F8 (745nm)", "NIR");
			UART_Printf("| %-10d | %-10d | %-10d | %-10d | %-10d | %-10d | %-10d | %-12d | %-10d | %-10d | %-10d | %-10d |\r\n\n",channel_data[8],channel_data[4],channel_data[0],channel_data[5],channel_data[6],channel_data[1],channel_data[11],channel_data[2],channel_data[7],channel_data[9],channel_data[10],channel_data[3]);
			UART_Printf("---------------------------------------------------------------------------------------------------------------------------------------------------------------\r\n\n");
			*/

			uint16_t cdata6_updated[14] = {channel_data[8],channel_data[4],channel_data[0],channel_data[5],channel_data[6],channel_data[1],channel_data[11],channel_data[2],channel_data[7],channel_data[9],channel_data[10],channel_data[3]};


			lcd_send_cmd (0x80|0x40);
			lcd_send_string("S1:       ");
			lcd_send_cmd (0x80|0x4C);
			lcd_send_string("S2:     ");
			lcd_send_cmd (0x80|0x14);
			lcd_send_string("S3:       ");
			lcd_send_cmd (0x80|0x20);
			lcd_send_string("S4:     ");
			lcd_send_cmd (0x80|0x54);
			lcd_send_string("S5:       ");
			lcd_send_cmd (0x80|0x60);
			lcd_send_string("S6:     ");

			char str1[6],str2[6],str3[6],str4[6],str5[6],str6[6];
			sprintf(str1, "%u", cdata1_updated[wl_value]);  // sample 1
			sprintf(str2, "%u", cdata2_updated[wl_value]);  // sample 2
			sprintf(str3, "%u", cdata3_updated[wl_value]);  // sample 3
			sprintf(str4, "%u", cdata4_updated[wl_value]);  // sample 4
			sprintf(str5, "%u", cdata5_updated[wl_value]);  // sample 5
			sprintf(str6, "%u", cdata6_updated[wl_value]);  // sample 6

			lcd_send_cmd (0x80|0x43);
			lcd_send_string(str1);
			lcd_send_cmd (0x80|0x4F);
			lcd_send_string(str2);
			lcd_send_cmd (0x80|0x17);
			lcd_send_string(str3);
			lcd_send_cmd (0x80|0x23);
			lcd_send_string(str4);
			lcd_send_cmd (0x80|0x57);
			lcd_send_string(str5);
			lcd_send_cmd (0x80|0x63);
			lcd_send_string(str6);




			//HAL_Delay(20);
			//HAL_Delay(measuringTime);
			continue;
		}
		else{
			continue;
		}
		UART_Printf("mode is %d", mode);
		HAL_Delay(1000);
    }
}

void find_timestep(void){
	float time;
	for (uint16_t i = 0x01; i <= 0xFF; i++){
		time = (float)(i+1) * 2.78f * (float)(astep+1) *0.001f;
		if(time >= desiredTime){
			atime = i;
			break;
		}
		if (i == 0xFF && astep == 10){
			astep = 999;
			d4Val = 0xE7;
			d5Val = 0x03;
			i = 0x01;
		}
		if (i == 0xFF && astep == 999){
			astep = 17999;
			d4Val = 0x0F;
			d5Val = 0x46;
			i = 0x01;
		}
	}
}

void UART_Printf(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void setTCAChannel(uint8_t i){
  uint8_t channel = 1 << i;
  HAL_I2C_Master_Transmit(&hi2c1, (0x70 << 1), &channel, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef AS7343_WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(&hi2c1, AS7343_I2C_ADDRESS, data, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef AS7343_ReadRegister(uint8_t reg, uint8_t *value) {
    if (HAL_I2C_Master_Transmit(&hi2c1, AS7343_I2C_ADDRESS, &reg, 1, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_I2C_Master_Receive(&hi2c1, AS7343_I2C_ADDRESS, value, 1, HAL_MAX_DELAY);
}




void AS7343_Init(void) {
    // Power on the sensor
    AS7343_WriteRegister(0x80, 0x01);
    HAL_Delay(10);  // Delay to ensure initialization

    // Enable spectral measurement
    AS7343_WriteRegister(0x80, 0x03);

    AS7343_WriteRegister(0xD6, 0x60);

    // Set gain and integration time


    //desiredTime = 1000.0f;
    find_timestep();

    AS7343_WriteRegister(0xC6, gain_value);  // AGAIN = 64x
    AS7343_WriteRegister(0x81, atime);  // ATIME = 40 steps (~111ms

    AS7343_WriteRegister(0xD4, d4Val);  // Lower byte (0x0F from 17999)
    AS7343_WriteRegister(0xD5, d5Val);  // Upper byte (0x46 from 17999)

    float tempVal = (float)(astep + 1) * (float)(atime + 1) * 2.78f * 0.001f;
    measuringTime = (int)(tempVal * 3.02);
    //UART_Printf("astep and atime: %d, %d\r\n\n", astep, atime);

    //UART_Printf("%d %d %d\r\n\n", tempVal, atime, astep);



    //UART_Printf("%.5f %d\r\n\n", tempVal, measuringTime);

    //UART_Printf("Gain Value: %d\r\n\n", gain_values[gain_value]);
    //UART_Printf("Integ Time: %.5f ms\r\n\n", tempVal);
    UART_Printf("\n\n\nCURRENT SETTINGS:\r\n");
    UART_Printf("GAIN: %dx\r\nINTEGRATION TIME: %.2fms\r\nASTEP: %d\r\nATIME: %d\r\n", gain_values[gain_value], tempVal, astep, atime);
    UART_Printf("ESTIMATED MEASUREMENT TIME: %dms\r\nLED BRIGHTNESS: %d%%\r\nCURRENT: 20mA\r\n", measuringTime, (int)(dutyCycle*100));
    //UART_Printf("AS7343 initialized successfully.\r\n");
}

void AS7343_ReadSpectralData(void) {
    uint8_t low_byte, high_byte;
    /*
    char *string_array[14] = {
    		3"FZ - 450nm", 0
			5"FY - 555nm", 1
			7"FXL - 600nm",2
    		11"NIR - 855nm",3
    		2"F2 - 425nm", 4
			3"F3 - 475nm", 5
			4"F4 - 515nm", 6
    		8"F6 - 640nm", 7
    		1"F1 - 405nm", 8
			9"F7 - 690nm", 9
			10"F8 - 745nm", 10
    		6"F5 - 550nm", 11
			"2xVIS",
			"FD"
    };
    */
    AS7343_ReadRegister(0x95, &low_byte);
    AS7343_ReadRegister(0x96, &high_byte);
    channel_data[0] = (high_byte << 8) | low_byte;

    AS7343_ReadRegister(0x97, &low_byte);
    AS7343_ReadRegister(0x98, &high_byte);
    channel_data[1] = (high_byte << 8) | low_byte;

    AS7343_ReadRegister(0x99, &low_byte);
    AS7343_ReadRegister(0x9A, &high_byte);
    channel_data[2] = (high_byte << 8) | low_byte;

    AS7343_ReadRegister(0x9B, &low_byte);
    AS7343_ReadRegister(0x9C, &high_byte);
    channel_data[3] = (high_byte << 8) | low_byte;

    AS7343_ReadRegister(0xA1, &low_byte);
    AS7343_ReadRegister(0xA2, &high_byte);
    channel_data[4] = (high_byte << 8) | low_byte;

    AS7343_ReadRegister(0xA3, &low_byte);
    AS7343_ReadRegister(0xA4, &high_byte);
    channel_data[5] = (high_byte << 8) | low_byte;

    AS7343_ReadRegister(0xA5, &low_byte);
    AS7343_ReadRegister(0xA6, &high_byte);
    channel_data[6] = (high_byte << 8) | low_byte;

    AS7343_ReadRegister(0xA7, &low_byte);
    AS7343_ReadRegister(0xA8, &high_byte);
    channel_data[7] = (high_byte << 8) | low_byte;

    AS7343_ReadRegister(0xAD, &low_byte);
    AS7343_ReadRegister(0xAE, &high_byte);
    channel_data[8] = (high_byte << 8) | low_byte;

    AS7343_ReadRegister(0xAF, &low_byte);
    AS7343_ReadRegister(0xB0, &high_byte);
    channel_data[9] = (high_byte << 8) | low_byte;

    AS7343_ReadRegister(0xB1, &low_byte);
    AS7343_ReadRegister(0xB2, &high_byte);
    channel_data[10] = (high_byte << 8) | low_byte;

    AS7343_ReadRegister(0xB3, &low_byte);
    AS7343_ReadRegister(0xB4, &high_byte);
    channel_data[11] = (high_byte << 8) | low_byte;

    AS7343_ReadRegister(0xB5, &low_byte);
    AS7343_ReadRegister(0xB6, &high_byte);
    channel_data[12] = (high_byte << 8) | low_byte;

    AS7343_ReadRegister(0xB7, &low_byte);
    AS7343_ReadRegister(0xB8, &high_byte);
    channel_data[13] = (high_byte << 8) | low_byte;



    //UART_Printf("Spectral Data:\r\n");
    //for (uint8_t i = 0; i < 14; i++) {
    //    UART_Printf("(%s): %d\r\n",string_array[i], channel_data[i]);
    //}
}

// Initialize I2C1 peripheral
static void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000; // Reduced for stability
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        while (1);
    }
}

uint32_t exCess;

void Get_Time(char *buffer)
{
	if(cTime == 0){
		exCess = HAL_GetTick();
		cTime = 1;
	}
    uint32_t millis = HAL_GetTick();  // Get system uptime in milliseconds
    millis = millis-exCess;
    //uint32_t ms = millis % 1000;
    uint32_t seconds = (millis / 1000) % 60;
    uint32_t minutes = (millis / 60000);
    //uint32_t minutes = (millis / 60000) % 60;
    //uint32_t hours = (millis / 3600000) % 24;

    sprintf(buffer, "%02lu:%02lu", minutes, seconds);
}


//THESE FOLLOWING FUNCTIONS BELOW ARE USEFUL FOR THE HEATING PART

// Read Thermistor Temperature
double readTemperature(void) {
    float adc_sum = 0;

	for (int i = 0; i < 100; i++) {
		HAL_ADC_Start(&hadc1);
	    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	    adc_sum += HAL_ADC_GetValue(&hadc1);
	    HAL_Delay(2);
	}
	float adc_value = adc_sum / 100.0f;



    float voltage = (adc_value * V_REF) / 4095.0f;
    float resistance = ((R_REF*V_REF)/voltage)-R_REF;
    float temperature = 1.0 / (A + B * log(resistance) + C * (log(resistance) * log(resistance) * log(resistance)));
    return temperature - 273.15f;  // Convert to Celsius
}

// Compute PID Output
double computePID(double temp) {
    double error = setpoint - temp;
    integral += error * 0.2;
    double derivative = (error - lastError) / 0.2;
    lastError = error;

    double output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    if (output > 100) output = 100; // Limit to 100% PWM
    if (output < 0) output = 0;     // Prevent negative values

    return output;
}

static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}


// Initialize GPIO
static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9; // Changed to PB8 (SCL) and PB9 (SDA)
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	//GPIO_InitStruct.Pin = GPIO_PIN_13;
	//GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	//GPIO_InitStruct.Pull = GPIO_NOPULL;
	//HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : PC13 PC12 */
  	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_12;
  	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  	GPIO_InitStruct.Pull = GPIO_NOPULL;
  	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PC10 PC11 */
	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


// Initialize USART2 UART
static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        while (1);
    }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


static void MX_TIM3_Init(void)
{


  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;  // 1 MHz Timer Clock
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 41999;   // 2 kHz PWM Frequency
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 41999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);

}

static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = psc;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = arr;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = arr;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

// System Clock Configuration
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while (1);
    }

    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                   RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        while (1);
    }
}
