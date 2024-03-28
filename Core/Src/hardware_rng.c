/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hardware_rng.c
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    14-April-2017
  * @brief   mbedtls alternate entropy data function.
  *          the mbedtls_hardware_poll() is customized to use the STM32 RNG
  *          to generate random data, required for TLS encryption algorithms.
  *
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
#include "hardware_rng.h"
#include "mbedtls/mbedtls_config.h"
#include "main.h"
#include "string.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#ifdef MBEDTLS_ENTROPY_HARDWARE_ALT

#include "entropy_poll.h"

extern RNG_HandleTypeDef hrng;

uint32_t randomValue_arr[32];
QueueHandle_t RandomNumGen_Q;

void SaveRandomValue(uint32_t val);
uint32_t UseRandomBackupValue(void);

void RandomNumberGeneratorTask(void * arg)
{
  HAL_StatusTypeDef xResult;
  RandomNumGen_Q = xQueueCreate(64,sizeof(uint32_t));
  uint32_t Random_number;
  while(1)
  {    
      xResult = HAL_ERROR;
      xResult = HAL_RNG_GenerateRandomNumber( &hrng, &Random_number );
      SaveRandomValue(Random_number);
      if( xResult == HAL_OK )
    	{
          xQueueSendToBack(RandomNumGen_Q,&Random_number,100);
        }
      else
        {
          
        }
        vTaskDelay(50/portTICK_PERIOD_MS);
  }
}


  void GetRandomValue(uint32_t val, char index)
{
  randomValue_arr[index] = val;
}
  
 BaseType_t xApplicationGetRandomNumber( uint32_t *pulValue )
 {
    HAL_StatusTypeDef xResult;
    BaseType_t xReturn;
    uint32_t ulValue;
    static uint8_t random_Val_index;
    	//xResult = HAL_RNG_GenerateRandomNumber( &hrng, &ulValue );
    if(uxQueueMessagesWaiting(RandomNumGen_Q) != 0)
    { 
      xQueueReceive(RandomNumGen_Q,&ulValue,300);
    	if( ulValue != 0 )
    	{
    		xReturn = pdPASS;
    		*pulValue = ulValue;
               /* if(random_Val_index >=32)
                {
                  random_Val_index = 0;
                }*/
                //GetRandomValue(ulValue,random_Val_index);
               // random_Val_index++;
    	}
    	else
    	{
    		xReturn = pdFAIL;
    	}
    }
    else
    {
      xReturn = pdPASS;
      ulValue = UseRandomBackupValue();
      *pulValue = ulValue;
    }
    	return xReturn;
 }


/*-------Save Backp val ---*/
uint32_t UseRandomBackupValue(void)
{
  static uint8_t random_Val_index;
  uint32_t tickcount, new_random_gen;
        if(random_Val_index >=32)
                {
                  random_Val_index = 0;
                }
        tickcount = xTaskGetTickCount();
        new_random_gen = randomValue_arr[random_Val_index] ^tickcount;
        random_Val_index++;
        return  new_random_gen;
}
  
void SaveRandomValue(uint32_t val)
{
  static uint8_t random_Val_index;
  
        if(random_Val_index >=32)
                {
                  random_Val_index = 0;
                }
        randomValue_arr[random_Val_index] = val;
        random_Val_index++;
}

uint32_t RandomValGEn(void)
{
  uint32_t my_randomValue;
  BaseType_t xReturn = xApplicationGetRandomNumber(&my_randomValue);
   if(xReturn == pdPASS)
  {
     return my_randomValue;
  }
  else {
      return xTaskGetTickCount();
    }
}
  

int mbedtls_hardware_poll( void *Data, unsigned char *Output, size_t Len, size_t *oLen )
{
  uint8_t index;
  uint32_t randomValue;
  uint32_t test_tick;
  Data = NULL;
  for (index = 0; index < Len/4;index++)
  {
    if(xQueueReceive(RandomNumGen_Q,&randomValue,300))
    {   
      if (randomValue != 0)
      {
         *oLen += 4;
         memcpy(&(Output[index * 4]), &randomValue, 4);
      }
    }
    else 
    {
      
      randomValue = UseRandomBackupValue();
      *oLen += 4;
      memcpy(&(Output[index * 4]), &randomValue, 4);
    }

  }
      FreeHeapSize_t = xPortGetFreeHeapSize();

  return 0;
}

#endif /*MBEDTLS_ENTROPY_HARDWARE_ALT*/

