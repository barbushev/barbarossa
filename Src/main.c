#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"
#include "BA_tasks.h"
#include "barbarossa_config.h"
#include "L6470.h"
#include "file_operations.h"
#include "RTC.h"
#include "Mixer_Control.h"
#include "Door_Sensor.h"
#include "Door_Latch.h"
#include "Illumination_Control.h"
#include "WatchDog.h"


CRC_HandleTypeDef hcrc;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);

void BKPSRAM_Init(void)
{
	//enable access to Power Controller PWR APB1 interface clock
	__HAL_RCC_PWR_CLK_ENABLE();  // Enable PWR clock

	__HAL_RCC_BKPSRAM_CLK_ENABLE(); // Enable backup SRAM Clock

	HAL_PWR_EnableBkUpAccess(); // Allow access to backup domain

	*(__IO uint32_t *) CSR_BRE_BB = (uint32_t)ENABLE;  /// Enable the Backup SRAM low power Regulator. This will allow data to stay when using VBat mode.

	/* Wait for backup regulator to be ready  */
	while (!(PWR->CSR & (PWR_FLAG_BRR)));
}

/*
 * This function is called from SystemInit() in system_stm32f4xx.c during bootup.
 * It checks if a flag has been set to activate DFU mode.
 * If the flag is set, it activates DFU mode.
 */
void JumpToBootLoader(void)
{
	 BKPSRAM_Init();  //intialize the backup SRAM

	 if  (BKPSRAM_Read8(BA_FIRMWARE_UPDATE_ADDR) != BA_FIRMWARE_UPDATE_FLAG) return;  //check the register for a flag

	 BKPSRAM_Write8(BA_FIRMWARE_UPDATE_ADDR, 0x00);		//clear the flag

	 void (*SysMemBootJump)(void);

	 // Set system memory address. -  For STM32F429, system memory is on 0x1FFF 0000
     volatile uint32_t addr = 0x1FFF0000;

	 // Disable RCC, set it to default (after reset) settings Internal clock, no PLL, etc.
	 HAL_RCC_DeInit();

	 // Disable systick timer and reset it to default values
	 SysTick->CTRL = 0;
	 SysTick->LOAD = 0;
	 SysTick->VAL = 0;

	 // Disable all interrupts
	 //__disable_irq();

	 // Remap system memory to address 0x0000 0000 in address space
     __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

     // Set jump memory location for system memory. Use address with 4 bytes offset which specifies jump location where program starts.
     SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));

	 // Set main stack pointer. This step must be done last otherwise local variables in this function
	 // don't have proper value since stack pointer is located on different position. Set direct address location which specifies stack pointer in SRAM location.

	 __set_MSP(*(uint32_t *)addr);

	 // Actually call our function to jump to set location. This will start system memory execution
     SysMemBootJump();
     while(1);
}


int main(void)
 {
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  //activate independent watch dog
  WatchDog_Init();

 //turn on GPIO clocks
  MX_GPIO_Init();

  RTC_Init();

  MX_CRC_Init();

  if (Mixer_Control_Init() != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_HARDWARE_INIT, 0);

  Door_Sensor_Init();
  Door_Latch_Init();
  Illumination_Control_Init();
  BA_LEDs_Init();

  ACTUATOR_INIT();

  if (FLUID_CAL_INIT() != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_HARDWARE_INIT, 0);

  if (Fluid_Sensor_Init() != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_HARDWARE_INIT, 0);

  if (L6470_InitSPI() != HAL_OK) BA_ERROR_HANLDER(BA_ERROR_HARDWARE_INIT, 0);

  RTOS_Start();
  
  /* We should never get here as control is now taken by the scheduler */

  while (1)
  {
	  //this would only execute if for some reason the RTOS failed to start
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    BA_ERROR_HANLDER(BA_ERROR_HARDWARE_INIT, 0);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
	  BA_ERROR_HANLDER(BA_ERROR_HARDWARE_INIT, 0);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
	  BA_ERROR_HANLDER(BA_ERROR_HARDWARE_INIT, 0);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
	  BA_ERROR_HANLDER(BA_ERROR_HARDWARE_INIT, 0);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if (htim->Instance == TIM3) { HAL_IncTick();}						//OS tick
 //if (htim->Instance == TIM14) {BA_IncDebugTick(); }    //Debug Tick
}


#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif
