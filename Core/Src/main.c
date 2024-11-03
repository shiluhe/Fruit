/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "RoboMain.h"
#include "string.h" //ʹ��strlen��Ҫ


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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


#ifdef __cplusplus
extern "C" {
#endif
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM7_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM9_Init();
  MX_UART4_Init();
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  //Init

    HAL_Delay(5000); //�ӳ�5����Ϊ���ȶ��ߵ��Ķ���???

    __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE); //�رս�������¼��ж�


    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//4�����
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);//2�����ֶ��
    //PWM TIM1 ||1 PE9||2 PE11||3 PE13||4 PE14||

    HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1|TIM_CHANNEL_2);//PA15 PB3//PA0 PA1Ϊ��λ��������
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1|TIM_CHANNEL_2);//PA6 PA7
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1|TIM_CHANNEL_2);//PD12 PD13
    HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_1|TIM_CHANNEL_2);//PC6 PC7


    HAL_TIM_Base_Start_IT(&htim7);  //ÿ��0.5ms����pid


    RobotInit();              //��ʼ��PID
    RobotInitServo();         //��ʼ��PWM���
    HAL_Delay(500);    //��ʼ��ʱ��
    RobotBeginVoice(0, (uint8_t *)"[v12][m0][t5]����ζ�����׼����ʼ����");
    HAL_Delay(4500);//��������ʱ��


//    RoboAllMove();
//    RobotMoveSpeed(50);
//      RobotSpinNinety(false);
//    RobotArmMiddle();
//    RobotGrabRightUp();
//    RobotArmMiddle();
//    RobotGrabLeftUp();
//    RobotMoveForward(50, 40);//ǰ�ֵ�������8cm
//    HAL_Delay(1000);
//          RobotSpinNinety(false);
//    RobotMoveForward(50, 40);//ǰ�ֵ�������8cm
//    HAL_Delay(1000);
//    RobotMoveForward(50, 40);//ǰ�ֵ�������8cm
//    HAL_Delay(1000);

///////////////////////////////////////////��ʼ��ʼ��ʼ////////////////////////////////////////////////////////////////////////

///////////////////////////////AAAAAAAAAAAAAAA��////////////////////////////////////////////
    RobotMoveForward(40, 48);//ǰ�ֵ�������8cm
    RobotArmMiddle();
    RobotGrabRightGround();
    RobotArmMiddle();
    RobotGrabLeftUp();
    RobotArmMiddle();

    RobotMoveForward(40, 100);
    RobotArmMiddle();
    RobotGrabRightUp();
    RobotArmMiddle();
    RobotGrabLeftGround();
    RobotArmMiddle();

    RobotMoveForward(40, 100);
    RobotArmMiddle();
    RobotGrabRightGround();
    RobotArmMiddle();
    RobotGrabLeftUp();
    RobotArmMiddleBehind();

    RobotMoveForward(40, 58);  //58=40+18
    RobotSpinNinety(false);
    //���½Ƕ�
    ///////////////////////////////////BBBBBBBBBBBBBBBBBBBBBBBBBBB����/////////////////////////////////////////////////

    RobotMoveForward(40, 99);///////////////�ص��
    RobotSpinNinety(false);

    RobotMoveForward(40, 22);  //22=40-18
    RobotArmMiddle();
    RobotGrabRightGround();
    RobotArmMiddle();
    RobotGrabLeftUp();
    RobotArmMiddle();

    RobotMoveForward(40, 100);
    RobotArmMiddle();
    RobotGrabRightUp();
    RobotArmMiddle();
    RobotGrabLeftGround();
    RobotArmMiddle();

    RobotMoveForward(40, 100);
    RobotArmMiddle();
    RobotGrabRightGround();
    RobotArmMiddle();
    RobotGrabLeftUp();
    RobotArmMiddleBehind();

    RobotMoveForward(40, 58);
    RobotSpinNinety(true);
    /////////////////////////////////CCCCCCCCCCCCCCCCCCCCCCCCC����////////////////////////////////////////////////////////


    RobotMoveForward(40, 125.5); //B����C��         ///////////////�ص��
    RobotSpinNinety(true);

    RobotMoveForward(40, 44);  //C����㵽C���߲˴�
    RobotArmMiddle();
    RobotCGrabLeft();
    RobotArmMiddle();
    RobotCGrabRight();
    RobotArmMiddle();

    RobotMoveForward(40, 40);
    RobotCGrabLeft();
    RobotArmMiddle();
    RobotCGrabRight();
    RobotArmMiddle();

    RobotMoveForward(40, 40);
    RobotCGrabLeft();
    RobotArmMiddle();
    RobotCGrabRight();
    RobotArmMiddle();

    RobotMoveForward(40, 40);
    RobotCGrabLeft();
    RobotArmMiddle();
    RobotCGrabRight();
    RobotArmMiddle();

    RobotMoveForward(40, 40);
    RobotCGrabLeft();
    RobotArmMiddle();
    RobotCGrabRight();
    RobotArmMiddle();
    RobotArmMiddleBehind();

    RobotMoveForward(40, 82);  //Cһ�����һ���Ϲϵ�ʮ�ֵľ���
    RobotSpinNinety(false);
    //���½Ƕ�

    RobotMoveForward(40, 105.5);//Cһ����C����֮��ľ���////////////////////////////�ص��
    RobotSpinNinety(false);

    RobotMoveForward(40, 45);  //C������һ���Ϲϵ�ʮ�ֵľ���
    RobotCGrabLeft();
    RobotArmMiddle();
    RobotCGrabRight();
    RobotArmMiddle();

    RobotMoveForward(40, 40);
    RobotCGrabLeft();
    RobotArmMiddle();
    RobotCGrabRight();
    RobotArmMiddle();

    RobotMoveForward(40, 40);
    RobotCGrabLeft();
    RobotArmMiddle();
    RobotCGrabRight();
    RobotArmMiddle();

    RobotMoveForward(40, 40);
    RobotCGrabLeft();
    RobotArmMiddle();
    RobotCGrabRight();
    RobotArmMiddle();

    RobotMoveForward(40, 40);
    RobotCGrabLeft();
    RobotArmMiddle();
    RobotCGrabRight();
    RobotArmMiddle();
//////////////////////////////////////////////������������//////////////////////////////////////////////////////////////

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//      VofaTransmitInc();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM7) {
        RoboTick();
        //RoboTest();
    }

    if(htim->Instance == TIM6)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
}
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
        while (1) {
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
