//
// Created by vitaliy on 03.04.2022.
//

#include "main.h"
#include "Interrupt.hpp"
#include "../logger/Logger.hpp"
#include "stm32f4xx_it.h"
#include "../Inc/preference.h"
#include "Interval.h"


extern Logger mLog;
extern Interval arrIP;  // массив данных об интервалах


/** Проверка лога внутри прерывания */
uint16_t pauseItCount = 0;
uint8_t itMess[] = "======InterruptTestMessage=====\r\n";
void logItTest(void){
  if(pauseItCount > 99){
    mLog.messageLogIT(itMess, sizeof(itMess) - 1);
    pauseItCount = 0;
  }
  pauseItCount += 1;
}

/** Отслеживание переполнения энкодера M0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM1) //check if the interrupt comes from TIM1
  {
//    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
}

/** Прерывание от системного таймера */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  arrIP.interruptOnSystemTick();

  /* USER CODE END SysTick_IRQn 1 */
}

//void TIM4_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM4_IRQn 0 */
//
//  /* USER CODE END TIM4_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim4);
//  /* USER CODE BEGIN TIM4_IRQn 1 */
//
//  /* USER CODE END TIM4_IRQn 1 */
//}

/** Прерывание по захвату таймера */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

  if (htim->Instance == TIM4)
  {
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      TIM2->CNT = 0;

//      period = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);

    }
  }
}



