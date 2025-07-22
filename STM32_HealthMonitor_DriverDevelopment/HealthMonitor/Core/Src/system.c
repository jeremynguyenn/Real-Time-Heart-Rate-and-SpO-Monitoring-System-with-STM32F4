/*
 * system.c
 *
 *      Author: Nguyennhan
 */

#include "system.h"
#include "stm32f4xx_hal.h"

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void errorHandler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

// Get number of elapsed milliseconds
uint32_t millis(void)
{
	return HAL_GetTick();
}
