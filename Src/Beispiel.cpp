/*
 * main.cpp
 *
 *  Created on: Dec 27, 2022
 *      Author: Carst
 */

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stm32f0xx.h>
#include <stm32f0xx_ll_gpio.h>
#include "Hardware.h"
#include "timer.h"

Beispiel::Hardware hw;

int main()
{
  CarstenKeller::Timer t_led = CarstenKeller::Timer(500U, true);
  __enable_irq();
  while(1)
  {
    if(t_led.elapsed())
    {
      LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_4);
    }
    hw.run();
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
extern "C" void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  __asm volatile ("bkpt #0");
  printf("Wrong parameters value: file %s on line %" PRIu32 "\r\n", (char*)file, line);
  while(1);
}
#endif /* USE_FULL_ASSERT */
