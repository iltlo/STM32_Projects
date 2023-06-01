/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include "string.h"

#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include  "CAN_receive.h"
#include "bsp_can.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//pid_type_def motor_pid;           // declare PID structure
extern PID_TypeDef drive_motor_pid[4];    // declare PID structure //array defined in pid.c, use "extern"

const motor_measure_t *motor_data[4]; // declare a pointer to the motor data structure

int set_spd = 0;                    // target speed
//const fp32 PID[3] = { 10, 0.02, 0}; // P I D


uint32_t MsgTimer = 0; // time counter
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USART6_IRQHandler(void)
{
    volatile uint8_t receive;
    //receive interrupt
    if(huart6.Instance->SR & UART_FLAG_RXNE)
    {
        receive = huart6.Instance->DR;
        HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    }
        //idle interrupt
    else if(huart6.Instance->SR & UART_FLAG_IDLE)
    {
        receive = huart6.Instance->DR;
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    //enable receive interrupt and idle interrupt
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);  //receive interrupt
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);  //idle interrupt

    can_filter_init();

    //const fp32 PID[3] = {5,2,2 }; // P I D
    //const fp32 PID[3] = {0.9,0.25,0.01 }; // P I D
    const fp32 PID[3] = {10,5,1 }; // P I D

    // initialize pid values of all motors
    for(int i=0;i<4;i++){
        pid_init(&drive_motor_pid[i]);  //把结构体里的函数指针赋值，三个函数
        // motor_pid[i].f_param_init(&motor_pid[i],PID_Speed,16384,5000,10,0,8000,0,1.5,0.1,0);
//        drive_motor_pid[i].f_param_init(&drive_motor_pid[i],PID_Speed,16384,5000,10,0,8000,0,PID[0],PID[1],PID[2]);
//        drive_motor_pid[i].f_param_init(&drive_motor_pid[i],PID_Position,300,200,10,0,800,0,PID[0],PID[1],PID[2]);
        drive_motor_pid[i].f_param_init(&drive_motor_pid[i],PID_Position,10000,200,10,0,800,0,PID[0],PID[1],PID[2]);
        //确定结构体内的参数，幅值，死区大小，PID系数
    }
    // get motor data
    motor_data[0] = get_chassis_motor_measure_point(0); // get the pointer of the motor (id 1) data
    motor_data[1] = get_chassis_motor_measure_point(1);
    motor_data[2] = get_chassis_motor_measure_point(2);
    motor_data[3] = get_chassis_motor_measure_point(3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // set all the target rotation speed (unit: rpm)
    drive_motor_pid[0].target = 3000;    // v can be negative
    drive_motor_pid[1].target = 0;
    drive_motor_pid[2].target = 0;   // v can be negative
    drive_motor_pid[3].target = motor_data[2]->ecd;

    drive_motor_pid[0].f_cal_pid(&drive_motor_pid[0],motor_data[0]->speed_rpm); // calculate motor 1 pid current
    drive_motor_pid[1].f_cal_pid(&drive_motor_pid[1],motor_data[0]->speed_rpm);
    drive_motor_pid[2].f_cal_pid(&drive_motor_pid[2],motor_data[2]->ecd);
    drive_motor_pid[3].f_cal_pid(&drive_motor_pid[3],motor_data[3]->ecd);

    CAN_cmd_chassis(drive_motor_pid[0].output,drive_motor_pid[1].output,0,drive_motor_pid[3].output);
    HAL_Delay(2);

//      CAN_cmd_chassis(4000, 3000, 1000, 1000);
//      HAL_Delay(10);

      if (HAL_GetTick() - MsgTimer > 100) {
          char info1[10];
          char info2[10];
          char info3[10];
          char info4[10];
          int temp = (int) drive_motor_pid[3].output;
          itoa(motor_data[1]->speed_rpm, info1, 10);
          itoa(motor_data[2]->ecd, info2, 10);
          itoa(motor_data[3]->ecd, info3, 10);
          itoa(temp, info4, 10);
//          strcat(info1, "\r\n");
//          strcat(info2, "\r\n");
//          strcat(info3, "\r\n");
//          strcat(info4, "\r\n");

//          HAL_UART_Transmit(&huart6, "(1)rpm:", 8, 100);
//          HAL_UART_Transmit(&huart6, info1, 11, 100);
          HAL_UART_Transmit(&huart6, "(2)ecd:", 7, 3);
          HAL_UART_Transmit(&huart6, info2, 12, 3);
          HAL_UART_Transmit(&huart6, "\r\n", 2, 3);

          HAL_UART_Transmit(&huart6, "(3)ecd:", 7, 3);
          HAL_UART_Transmit(&huart6, info3, 12, 3);
          HAL_UART_Transmit(&huart6, "\r\n", 2, 3);

          HAL_UART_Transmit(&huart6, "(3)output:", 10, 3);
          HAL_UART_Transmit(&huart6, info4, 12, 3);
          HAL_UART_Transmit(&huart6, "\r\n", 2, 3);
          HAL_Delay(1);
            MsgTimer = HAL_GetTick();
      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
