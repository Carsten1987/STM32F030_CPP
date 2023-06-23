/*
 * Hardware.cpp
 *
 *  Created on: Dec 27, 2022
 *      Author: Carst
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stm32f0xx.h>
#include <stm32f0xx_ll_bus.h>
#include <stm32f0xx_ll_cortex.h>
#include <stm32f0xx_ll_gpio.h>
#include <stm32f0xx_ll_rcc.h>
#include <stm32f0xx_ll_system.h>
#include <stm32f0xx_ll_usart.h>
#include <stm32f0xx_ll_utils.h>
#include "Hardware.h"

extern std::uint32_t _estack;

extern "C" void assert_failed(uint8_t* file, uint32_t line);
extern "C" void ResetHandler(void);

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void WWDG_IRQHandler(void);
void RTC_IRQHandler(void);
void FLASH_IRQHandler(void);
void RCC_IRQHandler(void);
void EXTI0_1_IRQHandler(void);
void EXTI2_3_IRQHandler(void);
void EXTI4_15_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel2_3_IRQHandler(void);
void DMA1_Channel4_5_IRQHandler(void);
void ADC1_IRQHandler(void);
void TIM1_BRK_UP_TRG_COM_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM14_IRQHandler(void);
void TIM16_IRQHandler(void);
void TIM17_IRQHandler(void);
void I2C1_IRQHandler(void);
void SPI1_IRQHandler(void);
void USART1_IRQHandler(void);

const std::uintptr_t interruptVectorTable[] __attribute__((section(".isr_vector")))
{
  // Stack Ptr initialization
  reinterpret_cast<std::uintptr_t>(&_estack),
  // Entry point
  reinterpret_cast<std::uintptr_t>(ResetHandler),
  // Exceptions
  reinterpret_cast<std::uintptr_t>(NMI_Handler),                       /* NMI_Handler */
  reinterpret_cast<std::uintptr_t>(HardFault_Handler),                 /* HardFault_Handler */
  reinterpret_cast<std::uintptr_t>(MemManage_Handler),                 /* MemManage_Handler */
  reinterpret_cast<std::uintptr_t>(BusFault_Handler),                  /* BusFault_Handler */
  reinterpret_cast<std::uintptr_t>(UsageFault_Handler),                /* UsageFault_Handler */
  reinterpret_cast<std::uintptr_t>(nullptr),                           /* 0 */
  reinterpret_cast<std::uintptr_t>(nullptr),                           /* 0 */
  reinterpret_cast<std::uintptr_t>(nullptr),                           /* 0 */
  reinterpret_cast<std::uintptr_t>(nullptr),                           /* 0 */
  reinterpret_cast<std::uintptr_t>(SVC_Handler),                       /* SVC_Handler */
  reinterpret_cast<std::uintptr_t>(DebugMon_Handler),                  /* DebugMon_Handler */
  reinterpret_cast<std::uintptr_t>(nullptr),                           /* 0 */
  reinterpret_cast<std::uintptr_t>(PendSV_Handler),                    /* PendSV_Handler */
  reinterpret_cast<std::uintptr_t>(SysTick_Handler),                   /* SysTick_Handler */
  // External Interrupts
  reinterpret_cast<std::uintptr_t>(WWDG_IRQHandler),                   /* Window WatchDog              */
  reinterpret_cast<std::uintptr_t>(nullptr),                                 /* Reserved                     */
  reinterpret_cast<std::uintptr_t>(RTC_IRQHandler),                    /* RTC through the EXTI line    */
  reinterpret_cast<std::uintptr_t>(FLASH_IRQHandler),                  /* FLASH                        */
  reinterpret_cast<std::uintptr_t>(RCC_IRQHandler),                    /* RCC                          */
  reinterpret_cast<std::uintptr_t>(EXTI0_1_IRQHandler),                /* EXTI Line 0 and 1            */
  reinterpret_cast<std::uintptr_t>(EXTI2_3_IRQHandler),                /* EXTI Line 2 and 3            */
  reinterpret_cast<std::uintptr_t>(EXTI4_15_IRQHandler),               /* EXTI Line 4 to 15            */
  reinterpret_cast<std::uintptr_t>(nullptr),                                 /* Reserved                     */
  reinterpret_cast<std::uintptr_t>(DMA1_Channel1_IRQHandler),          /* DMA1 Channel 1               */
  reinterpret_cast<std::uintptr_t>(DMA1_Channel2_3_IRQHandler),        /* DMA1 Channel 2 and Channel 3 */
  reinterpret_cast<std::uintptr_t>(DMA1_Channel4_5_IRQHandler),        /* DMA1 Channel 4 and Channel 5 */
  reinterpret_cast<std::uintptr_t>(ADC1_IRQHandler),                   /* ADC1                         */
  reinterpret_cast<std::uintptr_t>(TIM1_BRK_UP_TRG_COM_IRQHandler),    /* TIM1 Break, Update, Trigger and Commutation */
  reinterpret_cast<std::uintptr_t>(TIM1_CC_IRQHandler),                /* TIM1 Capture Compare         */
  reinterpret_cast<std::uintptr_t>(nullptr),                                 /* Reserved                     */
  reinterpret_cast<std::uintptr_t>(TIM3_IRQHandler),                   /* TIM3                         */
  reinterpret_cast<std::uintptr_t>(nullptr),                                 /* Reserved                     */
  reinterpret_cast<std::uintptr_t>(nullptr),                                 /* Reserved                     */
  reinterpret_cast<std::uintptr_t>(TIM14_IRQHandler),                  /* TIM14                        */
  reinterpret_cast<std::uintptr_t>(nullptr),                                 /* Reserved                     */
  reinterpret_cast<std::uintptr_t>(TIM16_IRQHandler),                  /* TIM16                        */
  reinterpret_cast<std::uintptr_t>(TIM17_IRQHandler),                  /* TIM17                        */
  reinterpret_cast<std::uintptr_t>(I2C1_IRQHandler),                   /* I2C1                         */
  reinterpret_cast<std::uintptr_t>(nullptr),                                 /* Reserved                     */
  reinterpret_cast<std::uintptr_t>(SPI1_IRQHandler),                   /* SPI1                         */
  reinterpret_cast<std::uintptr_t>(nullptr),                                 /* Reserved                     */
  reinterpret_cast<std::uintptr_t>(USART1_IRQHandler),                 /* USART1                       */
  reinterpret_cast<std::uintptr_t>(nullptr),                                 /* Reserved                     */
  reinterpret_cast<std::uintptr_t>(nullptr),                                 /* Reserved                     */
  reinterpret_cast<std::uintptr_t>(nullptr),                                 /* Reserved                     */
  reinterpret_cast<std::uintptr_t>(nullptr),                                 /* Reserved                     */
};

extern "C" void ResetHandler(void)
{

  // Initialize data section
  extern std::uint8_t _sdata;
  extern std::uint8_t _edata;
  extern std::uint8_t _sidata;
  std::size_t size = static_cast<size_t>(&_edata - &_sdata);
  std::copy(&_sidata, &_sidata + size, &_sdata);

  // Initialize bss section
  extern std::uint8_t _sbss;
  extern std::uint8_t _ebss;
  std::fill(&_sbss, &_ebss, UINT8_C(0x00));

  SystemInit();

  // Initialize static objects by calling their constructors
  typedef void (*function_t)();
  extern function_t __init_array_start;
  extern function_t __init_array_end;
  std::for_each(&__init_array_start, &__init_array_end, [](const function_t pfn) {
      pfn();
  });

  // Jump to main
  asm ("bl main");
}
/* This variable is updated in three ways:
    1) by calling CMSIS function SystemCoreClockUpdate()
    2) by calling HAL API function HAL_RCC_GetHCLKFreq()
    3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
       Note: If you use this function to configure the system clock; then there
             is no need to call the 2 first functions listed above, since SystemCoreClock
             variable is updated automatically.
*/
uint32_t SystemCoreClock = 8000000;

const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

void SystemInit(void)
{
  /* Configure the system clock */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_6);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(48000000);
  LL_SYSTICK_EnableIT();
  LL_SetSystemCoreClock(48000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_SYSCLK);
}

void NMI_Handler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void HardFault_Handler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void MemManage_Handler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void BusFault_Handler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void UsageFault_Handler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void SVC_Handler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void DebugMon_Handler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void PendSV_Handler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}

void WWDG_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void RTC_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void FLASH_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void RCC_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void EXTI0_1_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void EXTI2_3_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void EXTI4_15_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void DMA1_Channel1_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void DMA1_Channel2_3_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void DMA1_Channel4_5_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void ADC1_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void TIM1_CC_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void TIM3_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void TIM14_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void TIM16_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void TIM17_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void I2C1_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void SPI1_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}
void USART1_IRQHandler(void)
{
  assert_failed((std::uint8_t*)__FILE__, (std::uint32_t)__LINE__);
}

namespace Beispiel
{
  Hardware::Hardware()
  {
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, 3);

    /* GPIO Ports Clock Enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    /* Peripheral clock enable */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);

    /**/
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

    /**LED-Pin*/
    LL_GPIO_InitTypeDef GPIO_InitStruct =
    {
        .Pin = LL_GPIO_PIN_4,
        .Mode = LL_GPIO_MODE_OUTPUT,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_NO,
        .Alternate = 0U,
    };
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /**USART1 GPIO Configuration
    PA2   ------> USART1_TX
    PA3   ------> USART1_RX
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    LL_USART_InitTypeDef USART_InitStruct = {
        .BaudRate = 115200,
        .DataWidth = LL_USART_DATAWIDTH_8B,
        .StopBits = LL_USART_STOPBITS_1,
        .Parity = LL_USART_PARITY_NONE,
        .TransferDirection = LL_USART_DIRECTION_TX_RX,
        .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
        .OverSampling = LL_USART_OVERSAMPLING_16,
    };
    LL_USART_Init(USART1, &USART_InitStruct);
    LL_USART_DisableIT_CTS(USART1);
    LL_USART_ConfigAsyncMode(USART1);
    LL_USART_Enable(USART1);
  }

  Hardware::~Hardware()
  {
    // TODO Auto-generated destructor stub
  }

  void Hardware::run(void)
  {

  }
}


