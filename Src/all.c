/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "usbd_desc.h"

#include "xinput.h"
#include "stm32_xinput.h"
#include "usb_xinput.h"





/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

extern USBD_HandleTypeDef  *hUsbDevice_0;
uint32_t ADC_buffer[6];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */
uint8_t USB_RX_Buffer[8] = {0,0,0,0,0,0,0,0};
uint8_t USB_TX_Buffer[8] = {0,0,0,0,0,0,0,0};


extern uint8_t USBD_CUSTOM_HID_SendReport     (USBD_HandleTypeDef  *pdev, 
                                 uint8_t *report,
                                 uint16_t len);

/* USER CODE END 0 */


int sendtag = 1;

int main(void)
{
	
	
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

	HAL_DMA_Init(&hdma_adc1); 
	HAL_ADC_Start_DMA( &hadc1, ADC_buffer, 6 );
	HAL_ADC_Start_IT( &hadc1 );
	
	// Declare port and pins
	declareButtonPins();
	declareAnalogPins();
	declareEncoderPins();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		// Read and update buttons, dpad and adc handbrake
		readButtons();
		
		// Potentiometers is used to triggers and sticks
		
		// Read ADC values
		readAdcValues();
				
		// Update values of sticks
		updateSticks();
		
		// Update values of triggres
		// updateTriggers();
		
		sendtag = HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_9 );
 
		
		// Send commands to PC
		//USBD_LL_Transmit( hUsbDevice_0, XINPUT_TX_ENDPOINT, TXData ,XINPUT_TX_SIZE);
		
		if (sendtag == 1){

		USBD_CUSTOM_HID_SendReport( hUsbDevice_0, TXData, XINPUT_TX_SIZE );
			
		
		}
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);				// Led to calculate speed transmission


		
  /* USER CODE END WHILE */
				
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  HAL_ADC_Init(&hadc1);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	  /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();


  /*Configure GPIO pins : PA7 PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_15|GPIO_PIN_9; 
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11
                           PB12 PB13 PB14 PB15
                           PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);



}

/* USER CODE BEGIN 4 */

/**	Encoder ISR
*		Handles the encoder that control the steering wheel position
**/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	/* Handls encoder of the x360 racing wheel
	if( GPIO_Pin ==  encoderPins[0].pin ){
		if( HAL_GPIO_ReadPin( encoderPins[1].port, encoderPins[1].pin ) == SET ){
			wheelEncoderValue++;
		}else{
			wheelEncoderValue--;
		}
	}	
	*/
}

/**	ADC ISR
*		Handles the values from ADC after the conversion finished
*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	
	if( adcValueReady == 0 ){
		
		yLeftStickValue_ADC= ADC_buffer[0];
		xLeftStickValue_ADC= ADC_buffer[1];
		leftTriggerValue_ADC= ADC_buffer[2];
		rightTriggerValue_ADC= ADC_buffer[3];
		yRightStickValue_ADC= ADC_buffer[5];
		xRightStickValue_ADC= ADC_buffer[4];
		
		
		adcValueReady = 1;
	}
	
}

/* USER CODE END 4 */

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

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern DMA_HandleTypeDef hdma_adc1;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 channel1 global interrupt.
*/
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
* @brief This function handles USB low priority or CAN RX0 interrupts.
*/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
#include "usb_xinput.h"
#include <string.h>    // for memcpy()

/*
//Function receives packets from the RX endpoint
//We will use this for receiving LED commands
int usb_xinput_recv(void *buffer, uint32_t timeout)
{
	usb_packet_t *rx_packet;
	uint32_t begin = millis();

	while (1) {
		if (!usb_configuration) return -1;
		rx_packet = usb_rx(XINPUT_RX_ENDPOINT);
		if (rx_packet) break;
		if (millis() - begin > timeout || !timeout) return 0;
		yield();
	}
	memcpy(buffer, rx_packet->buf, XINPUT_RX_SIZE);
	usb_free(rx_packet);
	return XINPUT_RX_SIZE;
}

//Function to check if packets are available
//to be received on the RX endpoint
int usb_xinput_available(void)
{
	uint32_t count;

	if (!usb_configuration) return 0;
	count = usb_rx_byte_count(XINPUT_RX_ENDPOINT);
	return count;
}

// Maximum number of transmit packets to queue so we don't starve other endpoints for memory
#define TX_PACKET_LIMIT 3

//Function used to send packets out of the TX endpoint
//This is used to send button reports
int usb_xinput_send(const void *buffer, uint32_t timeout)
{
	usb_packet_t *tx_packet;
	uint32_t begin = millis();

	while (1) {
		if (!usb_configuration) return -1;
		if (usb_tx_packet_count(XINPUT_TX_ENDPOINT) < TX_PACKET_LIMIT) {
			tx_packet = usb_malloc();
			if (tx_packet) break;
		}
		if (millis() - begin > timeout) return 0;
		yield();
	}
	memcpy(tx_packet->buf, buffer, XINPUT_TX_SIZE);
	tx_packet->len = XINPUT_TX_SIZE;
	usb_tx(XINPUT_TX_ENDPOINT, tx_packet);
	return XINPUT_TX_SIZE;
}
*/
/*
	Mechanical Squid Factory presents to you:
    XINPUT Controller library for TeensyLC, ported for STM32F103C8T6
    Compatible w/ PC
    
    Developer: Zachery Littell
    Email: zlittell@gmail.com
    www.zlittell.com
	
	This tricks the computer into loading the xbox 360 controller driver.
    Then it sends and receives reports in the same way as the xbox 360 controller.
	
	Credit where credit is due:
    Paul Stoffregen - for the awesome teensy and all the awesome examples he has included
    Hamaluik.com - for allowing me to not murder the arduino "IDE" out of frustration of hidden "magic"
    BeyondLogic.org - for a great resource on all things USB protocol related
    BrandonW - I contacted him a long time ago for a different project to get log files from his
             - beagle usb 12 between the 360 and controller.  I used them again for verification
             - and understanding during this project. (brandonw.net)
    free60.org - for their page on the x360 gamepad and its lusb output plus the explanations of the descriptors
    Microsoft - Windows Message Analyzer.  It wouldn't have been possible at times without this awesome message
              - analyzer capturing USB packets.  Debugged many issues with enumerating the device using this.
			  
	Also one final shoutout to Microsoft... basically **** you for creating xinput and not using HID to do so.
    XINPUT makes signing drivers necessary again, which means paying you.  Also you have ZERO openly available
    documentation on the XUSB device standard and I hate you for that.
*/	

#include "xinput.h"

//Private variables and functions

//Data
uint8_t TXData[20] = {0x00, 0x14, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  //Holds USB transmit packet data// @NOTE 
uint8_t RXData[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  //Holds USB receive packet data

//LED Patterns
uint8_t patternAllOff[10] = {0,0,0,0,0,0,0,0,0,0};
uint8_t patternBlinkRotate[10] = {1,0,1,0,1,0,1,0,1,0};
uint8_t patternPlayer1[10] = {1,0,0,0,0,0,0,0,0,0};
uint8_t patternPlayer2[10] = {1,0,1,0,0,0,0,0,0,0};
uint8_t patternPlayer3[10] = {1,0,1,0,1,0,0,0,0,0};
uint8_t patternPlayer4[10] = {1,0,1,0,1,0,1,0,0,0};
uint8_t patternCurrent[10] = {0,0,0,0,0,0,0,0,0,0};	//Variabled to hold the current pattern selected by the host

uint8_t rumbleValues[2] = {0x00,0x00};	//Array to hold values for rumble motors. rumbleValues[0] is big weight rumbleValues[1] is small weight
uint8_t currentPlayer = 0;	//Variable to access the current controller number attached to this device.  0 is no controller number assigned by host yet


//LED Pattern Tracking
uint8_t _modeLED = 0;			//Track LED mode
struct _pin _pinLED;			//Track LED pin
uint8_t _LEDState = 0;		//used to set the pin for the LED
uint32_t _previousMS = 0; //used to store the last time LED was updated
uint8_t _LEDtracker = 0;	//used as an index to step through a pattern on interval

void LEDPatternSelect(uint8_t rxPattern);

/* Initialize controller 
*
*/
void XINPUT_init( uint8_t LEDMode, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin ){
	XINPUT_setLEDMode( LEDMode, GPIOx, GPIO_Pin );
}

/* 	Update button in packet
*	 	Buttons L3,R3,START,BACK are in Packet 1
*		Buttons A,B,X,Y,LB,RB,LOGO are in Packet 2
*/
void XINPUT_buttonUpdate(uint8_t button, uint8_t buttonState)// @NOTE 
{
	//BUTTON_A
	if (button == BUTTON_A)
	{
		if(buttonState)
		{
			TXData[BUTTON_PACKET_2] |= A_MASK_ON;// @NOTE 
		}
		else
		{
			TXData[BUTTON_PACKET_2] &= A_MASK_OFF;
		}
	}
	//BUTTON_B
	else if(button == BUTTON_B)
	{
		if(buttonState)
		{
			TXData[BUTTON_PACKET_2] |= B_MASK_ON;// @NOTE 
		}
		else
		{
			TXData[BUTTON_PACKET_2] &= B_MASK_OFF;
		}
	}
	//BUTTON_X
	else if(button == BUTTON_X)
	{
		if(buttonState)
		{
			TXData[BUTTON_PACKET_2] |= X_MASK_ON;
		}
		else
		{
			TXData[BUTTON_PACKET_2] &= X_MASK_OFF;
		}
	}
	//BUTTON_Y
	else if(button == BUTTON_Y)
	{
		if(buttonState)
		{
			TXData[BUTTON_PACKET_2] |= Y_MASK_ON;
		}
		else
		{
			TXData[BUTTON_PACKET_2] &= Y_MASK_OFF;
		}
	}
	//BUTTON_LB
	else if(button == BUTTON_LB)
	{
		if(buttonState)
		{
			TXData[BUTTON_PACKET_2] |= LB_MASK_ON;
		}
		else
		{
			TXData[BUTTON_PACKET_2] &= LB_MASK_OFF;
		}
		
	}
	//BUTTON_RB
	else if(button == BUTTON_RB)
	{
		if(buttonState)
		{
			TXData[BUTTON_PACKET_2] |= RB_MASK_ON;
		}
		else
		{
			TXData[BUTTON_PACKET_2] &= RB_MASK_OFF;
		}
	}
	//BUTTON_L3
	else if(button == BUTTON_L3)
	{
		if(buttonState)
		{
			TXData[BUTTON_PACKET_1] |= L3_MASK_ON;
		}
		else
		{
			TXData[BUTTON_PACKET_1] &= L3_MASK_OFF;
		}
	}
	//BUTTON_R3
	else if(button == BUTTON_R3)
	{
		if(buttonState)
		{
			TXData[BUTTON_PACKET_1] |= R3_MASK_ON;
		}
		else
		{
			TXData[BUTTON_PACKET_1] &= R3_MASK_OFF;
		}
	}
	//BUTTON_START
	else if(button == BUTTON_START)
	{
		if(buttonState)
		{
			TXData[BUTTON_PACKET_1] |= START_MASK_ON;
		}
		else
		{
			TXData[BUTTON_PACKET_1] &= START_MASK_OFF;
		}
	}
	//BUTTON_BACK
	else if(button == BUTTON_BACK)
	{
		if(buttonState)
		{
			TXData[BUTTON_PACKET_1] |= BACK_MASK_ON;
		}
		else
		{
			TXData[BUTTON_PACKET_1] &= BACK_MASK_OFF;
		}
	}
	//BUTTON_LOGO
	else if(button == BUTTON_LOGO)
	{
		if(buttonState)
		{
			TXData[BUTTON_PACKET_2] |= LOGO_MASK_ON;
		}
		else
		{
			TXData[BUTTON_PACKET_2] &= LOGO_MASK_OFF;
		}
	}
	//Unknown Button
	else {}
}

/*	Update all buttons with a single array
*		Order is as follows A,B,X,Y,LB,RB,L3,R3,START,BACK,LOGO
*		11 buttons 0-10 in the array
*/
void XINPUT_buttonArrayUpdate(uint8_t buttonArray[11])
{
	//BUTTON_A
	if (buttonArray[0]){TXData[BUTTON_PACKET_2] |= A_MASK_ON;}// @NOTE 
	else{TXData[BUTTON_PACKET_2] &= A_MASK_OFF;}
	//BUTTON_B
	if(buttonArray[1]){TXData[BUTTON_PACKET_2] |= B_MASK_ON;}
	else{TXData[BUTTON_PACKET_2] &= B_MASK_OFF;}
	//BUTTON_X
	if(buttonArray[2]){TXData[BUTTON_PACKET_2] |= X_MASK_ON;}
	else{TXData[BUTTON_PACKET_2] &= X_MASK_OFF;}
	//BUTTON_Y
	if(buttonArray[3]){TXData[BUTTON_PACKET_2] |= Y_MASK_ON;}
	else{TXData[BUTTON_PACKET_2] &= Y_MASK_OFF;}
	//BUTTON_LB
	if(buttonArray[4]){TXData[BUTTON_PACKET_2] |= LB_MASK_ON;}
	else{TXData[BUTTON_PACKET_2] &= LB_MASK_OFF;}
	//BUTTON_RB
	if(buttonArray[5]){TXData[BUTTON_PACKET_2] |= RB_MASK_ON;}
	else{TXData[BUTTON_PACKET_2] &= RB_MASK_OFF;}
	//BUTTON_L3
	if(buttonArray[6]){TXData[BUTTON_PACKET_1] |= L3_MASK_ON;}
	else{TXData[BUTTON_PACKET_1] &= L3_MASK_OFF;}
	//BUTTON_R3
	if(buttonArray[7]){TXData[BUTTON_PACKET_1] |= R3_MASK_ON;}
	else{TXData[BUTTON_PACKET_1] &= R3_MASK_OFF;}
	//BUTTON_START
	if(buttonArray[8]){TXData[BUTTON_PACKET_1] |= START_MASK_ON;}
	else{TXData[BUTTON_PACKET_1] &= START_MASK_OFF;}
	//BUTTON_BACK
	if(buttonArray[9]){TXData[BUTTON_PACKET_1] |= BACK_MASK_ON;}
	else{TXData[BUTTON_PACKET_1] &= BACK_MASK_OFF;}
	//BUTTON_LOGO
	if(buttonArray[10]){TXData[BUTTON_PACKET_2] |= LOGO_MASK_ON;}
	else{TXData[BUTTON_PACKET_2] &= LOGO_MASK_OFF;}
}

/* 	Update dpad values in the packet
*		SOCD cleaner included
*		Programmed behavior is UP+DOWN=UP and LEFT+RIGHT=NEUTRAL
*		SOCD makes fightsticks tournament legal and helps prevent erroneous states 
*/
void XINPUT_dpadUpdate(uint8_t dpadUP, uint8_t dpadDOWN, uint8_t dpadLEFT, uint8_t dpadRIGHT)
{
	//Clear DPAD
	TXData[BUTTON_PACKET_1] &= DPAD_MASK_OFF;
	//DPAD Up
	if (dpadUP) {TXData[BUTTON_PACKET_1] |= DPAD_UP_MASK_ON;}
	//DPAD Down
	if (dpadDOWN && !dpadUP) {TXData[BUTTON_PACKET_1] |= DPAD_DOWN_MASK_ON;}
	//DPAD Left
	if (dpadLEFT && !dpadRIGHT) {TXData[BUTTON_PACKET_1] |= DPAD_LEFT_MASK_ON;}
	//DPAD Right
	if (dpadRIGHT && !dpadLEFT) {TXData[BUTTON_PACKET_1] |= DPAD_RIGHT_MASK_ON;}
}

/*	Update the trigger values in the packet		
*		0x00 to 0xFF
*/
void XINPUT_triggerUpdate(uint8_t triggerLeftValue, uint8_t triggerRightValue)
{
	TXData[LEFT_TRIGGER_PACKET] = triggerLeftValue;// @NOTE 
	TXData[RIGHT_TRIGGER_PACKET] = triggerRightValue;
}

/*	Update a single trigger value in the packet		
*		0x00 to 0xFF
*/
void XINPUT_singleTriggerUpdate(uint8_t trigger, uint8_t triggerValue)
{
	if (trigger == TRIGGER_LEFT)
	{
		TXData[LEFT_TRIGGER_PACKET] = triggerValue;
	}
	else if (trigger == TRIGGER_RIGHT)
	{
		TXData[RIGHT_TRIGGER_PACKET] = triggerValue;
	}
	else{/*invalid parameter*/}
}

/*	Analog Sticks
*		Each axis is a signed 16 bit integer
*		-32,768 to 32,767 is the range of value
*/
void XINPUT_stickUpdate(uint8_t analogStick, int16_t stickXDirValue, int16_t stickYDirValue)
{
	if (analogStick == STICK_LEFT)
	{
		//Left Stick X Axis
		TXData[LEFT_STICK_X_PACKET_LSB] = LOBYTE(stickXDirValue);		// (CONFERIR)// @NOTE 
		TXData[LEFT_STICK_X_PACKET_MSB] = HIBYTE(stickXDirValue);
		//Left Stick Y Axis
		TXData[LEFT_STICK_Y_PACKET_LSB] = LOBYTE(stickYDirValue);
		TXData[LEFT_STICK_Y_PACKET_MSB] = HIBYTE(stickYDirValue);
	}
	else if(analogStick == STICK_RIGHT)
	{
		//Right Stick X Axis
		TXData[RIGHT_STICK_X_PACKET_LSB] = LOBYTE(stickXDirValue);
		TXData[RIGHT_STICK_X_PACKET_MSB] = HIBYTE(stickXDirValue);
		//Right Stick Y Axis
		TXData[RIGHT_STICK_Y_PACKET_LSB] = LOBYTE(stickYDirValue);
		TXData[RIGHT_STICK_Y_PACKET_MSB] = HIBYTE(stickYDirValue);
	}
	else{/*invalid parameter*/}
}

/* Send an update packet to the PC
*
*/
void XINPUT_sendXinput()
{
	//Send TXData
	//XInputUSB.send(TXData, USB_TIMEOUT);		//(ALTERAR)
	
	//Zero out data
	//Start at 2 so that you can keep the message type and packet size
	//Then fill the rest with 0x00's
	for (int i=2; i<13; i++) {TXData[i] = 0x00;}
}

/*
*		(ALTERAR)
*/
uint8_t XINPUT_receiveXinput()
{
	/*
	//Check if packet available
	if (XInputUSB.available() > 0)
	{
		//Grab packet and store it in RXData array
		XInputUSB.recv(RXData, USB_TIMEOUT);
		
		//If the data is a rumble command parse it
		//Rumble Command
		//8bytes for this command. In hex looks like this
		//000800bbll000000
		//the bb is speed of the motor with the big weight
		//the ll is the speed of the motor with the small weight
		//0x00 to 0xFF (0-255) is the range of these values
		if ((RXData[0] == 0x00) & (RXData[1] == 0x08))
		{
			//Process Rumble
			rumbleValues[0] = RXData[3];	//Big weight
			rumbleValues[1] = RXData[4];  //Small weight
			return 1;
		}
		
		//If the data is an LED command parse it
		else if (RXData[0] == 1)
		{
			LEDPatternSelect(RXData[2]);	//Call LED Pattern Select and pass the pattern byte to it
			return 2;
		}
		
		//Some other command we don't parse came through
		else{return 3;}
	}
	//Packet not available return 0
	else{return 0;}
	*/
}

/* Set the LED mode and pin settings
*
*/
void XINPUT_setLEDMode(uint8_t LEDMode, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin ){
	//Check LED mode
	if (LEDMode == LED_ENABLED)
	{
		/** STM32CUBE DO THIS */
		//LED ENABLED
		//pinMode(LEDPin, OUTPUT);		//Set pin output mode
		//digitalWrite(LEDPin, LOW);	//Set pin low initially to turn light off
		
		_modeLED = LED_ENABLED;				//Set LED Mode
		_pinLED.port = GPIOx;					//Set LED Pin
		_pinLED.pin = GPIO_Pin;
	}
	else
	{
		//Invalid entry or No Led
		_modeLED = NO_LED;		//Clear LED Mode
		_pinLED.port = NULL;	//Clear LED Pin
		_pinLED.pin = 0;
	}
}

/*	Process and update the current LED Pattern
*		(ALTERAR)
*/
void XINPUT_LEDUpdate()
{
	/*
	if (_modeLED == LED_ENABLED)
	{
		//Grab the current time in mS that the program has been running
		uint32_t currentMS = millis();
	
		//subtract the previous update time from the current time and see if interval has passed
		if ((currentMS - _previousMS)>interval)
		{
			//Set the led state correctly according to next part of pattern
			_LEDState = patternCurrent[_LEDtracker];
			//update the previous time
			_previousMS = currentMS;
			//increment the pattern tracker
			_LEDtracker++;
			//write the state to the led
			digitalWrite(_pinLED, _LEDState);
		}
	
		//if we increased ledtracker to 10, it needs to rollover
		if (_LEDtracker==10) {_LEDtracker=0;}
	}
	else{LED mode is NO_LED}

	*/
}

/*	Select the correct LED pattern according to received packets
*
*/
void XINPUT_LEDPatternSelect(uint8_t rxPattern){
	/*
	Process the LED Pattern
	0x00 OFF
	0x01 All Blinking
	0x02 1 Flashes, then on
	0x03 2 Flashes, then on
	0x04 3 Flashes, then on
	0x05 4 Flashes, then on
	0x06 1 on
	0x07 2 on
	0x08 3 on
	0x09 4 on
	0x0A Rotating (1-2-4-3)
	0x0B Blinking*
	0x0C Slow Blinking*
	0x0D Alternating (1+4-2+3)*
	*Does Pattern and then goes back to previous
	*/
	//All blinking or rotating
	if((rxPattern==ALLBLINKING)||(rxPattern==ROTATING))
	{
		//Copy the pattern array into the current pattern
		memcpy(patternCurrent, patternBlinkRotate, 10);
		//Reset the index to beginning of pattern
		_LEDtracker = 0;
		//Set the current player to 0 to indicate not being handshaked completely yet
		currentPlayer = 0;
	}
	//Device is player 1
	else if ((rxPattern==FLASHON1)||(rxPattern==ON1))
	{
		//Copy the pattern array into the current pattern
		memcpy(patternCurrent, patternPlayer1, 10);
		//Reset the index to beginning of pattern
		_LEDtracker = 0;
		//Set the current player to 1
		currentPlayer = 1;
	}
	//Device is player 2
	else if ((rxPattern==FLASHON2)||(rxPattern==ON2))
	{
		//Copy the pattern array into the current pattern
		memcpy(patternCurrent, patternPlayer2, 10);
		//Reset the index to beginning of pattern
		_LEDtracker = 0;
		//Set the current player to 2
		currentPlayer = 2;
	}
	//Device is player 3
	else if ((rxPattern==FLASHON3)||(rxPattern==ON3))
	{
		//Copy the pattern array into the current pattern
		memcpy(patternCurrent, patternPlayer3, 10);
		//Reset the index to beginning of pattern
		_LEDtracker = 0;
		//Set the current player to 3
		currentPlayer = 3;
	}
	//Device is player 4
	else if ((rxPattern==FLASHON4)||(rxPattern==ON4))
	{
		//Copy the pattern array into the current pattern
		memcpy(patternCurrent, patternPlayer4, 10);
		//Reset the index to beginning of pattern
		_LEDtracker = 0;
		//Set the current player to 4
		currentPlayer = 4;
	}
	//If pattern is not specified perform no pattern
	else
	{
		//Copy the pattern array into the current pattern
		memcpy(patternCurrent, patternAllOff, 10);
		//Pattern is all 0's so we don't care where LEDtracker is at
	}
}
/*
	Mechanical Squid Factory presents to you:
    XINPUT Controller library for TeensyLC, ported for STM32F103C8T6
    Compatible w/ PC
    
    Developer: Zachery Littell
    Email: zlittell@gmail.com
    www.zlittell.com
	
	This tricks the computer into loading the xbox 360 controller driver.
    Then it sends and receives reports in the same way as the xbox 360 controller.
	
	Credit where credit is due:
    Paul Stoffregen - for the awesome teensy and all the awesome examples he has included
    Hamaluik.com - for allowing me to not murder the arduino "IDE" out of frustration of hidden "magic"
    BeyondLogic.org - for a great resource on all things USB protocol related
    BrandonW - I contacted him a long time ago for a different project to get log files from his
             - beagle usb 12 between the 360 and controller.  I used them again for verification
             - and understanding during this project. (brandonw.net)
    free60.org - for their page on the x360 gamepad and its lusb output plus the explanations of the descriptors
    Microsoft - Windows Message Analyzer.  It wouldn't have been possible at times without this awesome message
              - analyzer capturing USB packets.  Debugged many issues with enumerating the device using this.
			  
	Also one final shoutout to Microsoft... basically **** you for creating xinput and not using HID to do so.
    XINPUT makes signing drivers necessary again, which means paying you.  Also you have ZERO openly available
    documentation on the XUSB device standard and I hate you for that.
*/	

#ifndef __xinput_H
#define __xinput_H

//Includes
#include "stdint.h"

#include "usb_xinput.h"
#include "stm32_xinput.h"
#include "usbd_def.h"

//Defines
//----------------------------------------------

//General Declarations
#define interval 150	//interval in milliseconds to update LED
#define USB_TIMEOUT 12840	//packet timeout for USB

//LED Pattern Defines
#define ALLOFF 0x00
#define ALLBLINKING 0x01
#define FLASHON1 0x02
#define FLASHON2 0x03
#define FLASHON3 0x04
#define FLASHON4 0x05
#define ON1  0x06
#define ON2  0x07
#define ON3  0x08
#define ON4  0x09
#define ROTATING 0x0A
#define BLINK	 0x0B
#define SLOWBLINK 0x0C
#define ALTERNATE 0x0D

//LED STYLE DEFINES
#define NO_LED 0
#define LED_ENABLED 1

//BUTTON MASK DEFINES
#define R3_MASK_ON 0x80
#define R3_MASK_OFF 0x7F
#define L3_MASK_ON 0x40
#define L3_MASK_OFF 0xBF
#define BACK_MASK_ON 0x20
#define BACK_MASK_OFF 0xDF
#define START_MASK_ON 0x10
#define START_MASK_OFF 0xEF
#define DPAD_RIGHT_MASK_ON 0x08
#define DPAD_RIGHT_MASK_OFF 0xF7
#define DPAD_LEFT_MASK_ON 0x04
#define DPAD_LEFT_MASK_OFF 0xFB
#define DPAD_DOWN_MASK_ON 0x02
#define DPAD_DOWN_MASK_OFF 0xFD
#define DPAD_UP_MASK_ON 0x01
#define DPAD_UP_MASK_OFF 0xFE
#define Y_MASK_ON 0x80
#define Y_MASK_OFF 0x7F
#define X_MASK_ON 0x40
#define X_MASK_OFF 0xBF
#define B_MASK_ON 0x20
#define B_MASK_OFF 0xDF
#define A_MASK_ON 0x10
#define A_MASK_OFF 0xEF
#define LOGO_MASK_ON 0x04
#define LOGO_MASK_OFF 0xFB
#define RB_MASK_ON 0x02
#define RB_MASK_OFF 0xFD
#define LB_MASK_ON 0x01
#define LB_MASK_OFF 0xFE
#define DPAD_MASK_OFF 0xF0

//Byte location Definitions
#define BUTTON_PACKET_1 2
#define BUTTON_PACKET_2 3// @NOTE 
#define LEFT_TRIGGER_PACKET 4
#define RIGHT_TRIGGER_PACKET 5
#define LEFT_STICK_X_PACKET_LSB 6// @NOTE 
#define LEFT_STICK_X_PACKET_MSB 7
#define LEFT_STICK_Y_PACKET_LSB 8
#define LEFT_STICK_Y_PACKET_MSB 9
#define RIGHT_STICK_X_PACKET_LSB 10
#define RIGHT_STICK_X_PACKET_MSB 11
#define RIGHT_STICK_Y_PACKET_LSB 12
#define RIGHT_STICK_Y_PACKET_MSB 13

//Classification numbers for updating controller items
#define BUTTON_A 0x01
#define BUTTON_B 0x02
#define BUTTON_X 0x03
#define BUTTON_Y 0x04
#define BUTTON_LB 0x05
#define BUTTON_RB 0x06
#define BUTTON_L3 0x07
#define BUTTON_R3 0x08
#define BUTTON_START 0x09
#define BUTTON_BACK 0x0a
#define BUTTON_LOGO 0x0b
#define DPAD_UP 0x0c
#define DPAD_DOWN 0x0d
#define DPAD_LEFT 0x0e
#define DPAD_RIGHT 0x0f
#define TRIGGER_LEFT 0x10
#define TRIGGER_RIGHT 0x11
#define STICK_LEFT 0x12
#define STICK_RIGHT 0x13

//Data 
extern uint8_t TXData[20];  //Holds USB transmit packet data
extern uint8_t RXData[8]; //Holds USB receive packet data


extern uint8_t rumbleValues[2];	//Array to hold values for rumble motors. rumbleValues[0] is big weight rumbleValues[1] is small weight
extern uint8_t currentPlayer;	//Variable to access the current controller number attached to this device.  0 is no controller number assigned by host yet

//Functions
void XINPUT_init( uint8_t LEDMode, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin );
void XINPUT_buttonUpdate(uint8_t button, uint8_t buttonState);
void XINPUT_buttonArrayUpdate(uint8_t buttonArray[11]);
void XINPUT_dpadUpdate(uint8_t dpadUP, uint8_t dpadDOWN, uint8_t dpadLEFT, uint8_t dpadRIGHT);
void XINPUT_triggerUpdate(uint8_t triggerLeftValue, uint8_t triggerRightValue);
void XINPUT_singleTriggerUpdate(uint8_t trigger, uint8_t triggerValue);
void XINPUT_stickUpdate(uint8_t analogStick, int16_t stickXDirValue, int16_t stickYDirValue);
void XINPUT_sendXinput(void);
uint8_t XINPUT_receiveXinput(void);
void XINPUT_setLEDMode(uint8_t LEDMode, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin );
void XINPUT_LEDUpdate(void);

#endif
/*
	Interface between Racing wheel and XINPUT Controller library
  Compatible w/ PC
  
	Developer: Daniel Nesvera
	
	WTFPL lincense

*/

#ifndef __stm32_xinput_H
#define __stm32_xinput_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f1xx_hal.h"
	 
#define NUM_BUTTONS 15
#define NUM_ANALOG 	7
	 
#define ENCODER_PRECISION 1
#define STEERING_DEGREES 200
#define MAX_ADC_VALUE 4033
#define MIN_HANDBRAKE_POS 200
#define MAX_HANDBRAKE_POS	800
#define MID_HANDBRAKE_POS ((MAX_HANDBRAKE_POS-MIN_HANDBRAKE_POS)/2)
	 
#define MAX_THROTTLE 4020				// values from my pedals
#define MAX_BRAKE 3370
	 
struct _pin{
	GPIO_TypeDef* port;
	uint16_t pin;
	GPIO_PinState state;			/// OFF: 0(RESET)		ON: 1(SET)
};

extern int16_t wheelEncoderValue;
extern uint16_t handbrakeValue;
extern int16_t leftTriggerValue_ADC;
extern int16_t rightTriggerValue_ADC;
extern int16_t xLeftStickValue_ADC;				// value comes from wheelEncoderValue
extern int16_t yLeftStickValue_ADC;				// Not used
extern int16_t xRightStickValue_ADC;
extern int16_t yRightStickValue_ADC;
extern struct _pin digitalArray[NUM_BUTTONS];
extern struct _pin analogArray[NUM_ANALOG];
extern struct _pin encoderPins[2];

extern int8_t adcValueReady;
extern int16_t leftTriggerValue;
extern int16_t rightTriggerValue;
extern int16_t xLeftStickValue;
extern int16_t yLeftStickValue;				
extern int16_t xRightStickValue;
extern int16_t yRightStickValue;

void declareButtonPins();
void declareAnalogPins();
void declareEncoderPins();
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
int32_t restrict(int32_t x, int32_t max, int32_t middle);
void readButtons();
void updateTriggers();
void updateSticks();
void readAdcValues();
uint8_t readTag(uint8_t tag);

#ifdef __cplusplus
}
#endif
#endif /*__stm32_xinput_H */
#ifndef __usb_xinput_H
#define __usb_xinput_H

#include <inttypes.h>
#include "stdint.h"

int usb_xinput_recv(void *buffer, uint32_t timeout);
int usb_xinput_available(void);
int usb_xinput_send(const void *buffer, uint32_t timeout);


#endif // USBxinput_h_
/*
	Interface between Racing wheel and XINPUT Controller library
  Compatible w/ PC
  
	Developer: Daniel Nesvera
	
	WTFPL lincense

*/

#include "stm32_xinput.h"
#include "xinput.h"
#include "math.h"

#define LeftStickVer_Pin GPIO_PIN_0
#define LeftStickVer_GPIO_Port GPIOA
#define LeftStickHori_Pin GPIO_PIN_1
#define LeftStickHori_GPIO_Port GPIOA
#define LeftTrigger_Pin GPIO_PIN_2
#define LeftTrigger_GPIO_Port GPIOA
#define RightTrigger_Pin GPIO_PIN_3
#define RightTrigger_GPIO_Port GPIOA
#define RightStickHori_Pin GPIO_PIN_4
#define RightStickHori_GPIO_Port GPIOA
#define RightStickVer_Pin GPIO_PIN_5
#define RightStickVer_GPIO_Port GPIOA
#define A_Pin GPIO_PIN_7
#define A_GPIO_Port GPIOA
#define B_Pin GPIO_PIN_0
#define B_GPIO_Port GPIOB
#define X_Pin GPIO_PIN_1
#define X_GPIO_Port GPIOB
#define Y_Pin GPIO_PIN_10
#define Y_GPIO_Port GPIOB
#define RB_Pin GPIO_PIN_11
#define RB_GPIO_Port GPIOB
#define Dright_Pin GPIO_PIN_12
#define Dright_GPIO_Port GPIOB
#define LB_Pin GPIO_PIN_13
#define LB_GPIO_Port GPIOB
#define Dup_Pin GPIO_PIN_14
#define Dup_GPIO_Port GPIOB
#define Ddown_Pin GPIO_PIN_15
#define Ddown_GPIO_Port GPIOB
#define Dleft_Pin GPIO_PIN_8
#define Dleft_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_9
#define LED_GPIO_Port GPIOA
#define L3_Pin GPIO_PIN_15
#define L3_GPIO_Port GPIOA
#define SELECT_Pin GPIO_PIN_3
#define SELECT_GPIO_Port GPIOB
#define START_Pin GPIO_PIN_4
#define START_GPIO_Port GPIOB
#define R3_Pin GPIO_PIN_5
#define R3_GPIO_Port GPIOB



// Initiate 
int16_t wheelEncoderValue = 0;
uint16_t handbrakeValue = 0;
int16_t leftTriggerValue_ADC = 0;
int16_t rightTriggerValue_ADC = 0;
int16_t xLeftStickValue_ADC = 0;
int16_t yLeftStickValue_ADC = 0;
int16_t xRightStickValue_ADC = 0;
int16_t yRightStickValue_ADC = 0;

int8_t adcValueReady = 0;
int16_t leftTriggerValue = 0;
int16_t rightTriggerValue = 0;
int16_t xLeftStickValue = 0;
int16_t yLeftStickValue = 0;
int16_t xRightStickValue = 0;
int16_t yRightStickValue = 0;

struct _pin digitalArray[NUM_BUTTONS];
struct _pin analogArray[NUM_ANALOG];
struct _pin encoderPins[2];

/* Declare port, pin and state of all buttons
*
*/
void declareButtonPins(){
	digitalArray[0].port  = A_GPIO_Port;		digitalArray[0].pin  = A_Pin;		digitalArray[0].state  = 0;		// A
	digitalArray[1].port  = B_GPIO_Port;		digitalArray[1].pin  = B_Pin;		digitalArray[1].state  = 0;		// B
	digitalArray[2].port  = X_GPIO_Port;		digitalArray[2].pin  = X_Pin;		digitalArray[2].state  = 0;		// X
	digitalArray[3].port  = Y_GPIO_Port;		digitalArray[3].pin  = Y_Pin;		digitalArray[3].state  = 0;		// Y
	digitalArray[4].port  = LB_GPIO_Port;		digitalArray[4].pin  = LB_Pin;		digitalArray[4].state  = 0;		// LB	
	digitalArray[5].port  = RB_GPIO_Port;		digitalArray[5].pin  = RB_Pin;		digitalArray[5].state  = 0;		// RB
	digitalArray[6].port  = L3_GPIO_Port;		digitalArray[6].pin  = L3_Pin;		digitalArray[6].state  = 0;		// L3
	digitalArray[7].port  = R3_GPIO_Port;		digitalArray[7].pin  = R3_Pin;		digitalArray[7].state  = 0;		// R3
	digitalArray[8].port  = START_GPIO_Port;		digitalArray[8].pin  = START_Pin;		digitalArray[8].state  = 0;		// START
	digitalArray[9].port  = SELECT_GPIO_Port;		digitalArray[9].pin  = SELECT_Pin;		digitalArray[9].state  = 0;		// BACK
	digitalArray[10].port = NULL;		digitalArray[10].pin = NULL;		digitalArray[10].state = 0;		// XBOX-LOGO
	digitalArray[11].port = Dup_GPIO_Port;		digitalArray[11].pin = Dup_Pin;		digitalArray[11].state = 0;		// D-UP	
	digitalArray[12].port = Ddown_GPIO_Port;		digitalArray[12].pin = Ddown_Pin;		digitalArray[12].state = 0;		// D-DOWN
	digitalArray[13].port = Dleft_GPIO_Port;		digitalArray[13].pin = Dleft_Pin;		digitalArray[13].state = 0;		// D-LEFT
	digitalArray[14].port = Dright_GPIO_Port;		digitalArray[14].pin = Dright_Pin;		digitalArray[14].state = 0;		// D-RIGHT
}

/* Declare port, pin of the analog inputs
*
*/
void declareAnalogPins(){
	analogArray[0].port = NULL;		analogArray[1].pin = NULL;		analogArray[1].state = 0;			// LEFT TRIGGER
	analogArray[1].port = NULL;		analogArray[2].pin = NULL;		analogArray[2].state = 0;			// RIGHT TRIGGER
	analogArray[2].port = RightStickHori_GPIO_Port;		analogArray[5].pin = RightStickHori_Pin;		analogArray[5].state = 0;			// X RIGHT STICK
	analogArray[3].port = RightStickVer_GPIO_Port;		analogArray[6].pin = RightStickVer_Pin;		analogArray[6].state = 0;			// Y RIGHT STICK
	analogArray[4].port = LeftStickHori_GPIO_Port;		analogArray[3].pin = LeftStickHori_Pin;		analogArray[3].state = 0;			// X LEFT STICK		
	analogArray[5].port = LeftStickVer_GPIO_Port;		analogArray[4].pin = LeftStickVer_Pin;		analogArray[4].state = 0;			// Y LEFT STICK		
	
}

/*	Declare port, pin of the encoder
*
*/
void declareEncoderPins(){
	encoderPins[0].port = NULL;		encoderPins[0].pin = NULL;		encoderPins[0].state = 0;			// Encoder input A - interrupt pin
	encoderPins[1].port = NULL;		encoderPins[1].pin = NULL;		encoderPins[1].state = 0;			// Encoder input B - normal input
}

/*	Read/update buttons and potentiometer of the handrake
*
*/
void readButtons(){
	int i = 0;
	int state = 1;
	uint8_t buttonArray[11] = {0,0,0,0,0,0,0,0,0,0,0};				// initialize array of buttons
	uint8_t dpadArray[4] = {0,0,0,0};													// initialize array from dpad
		
	while( i < NUM_BUTTONS ){
		
		if( digitalArray[i].port != NULL ){
			
			// 0 = PRESSED		1 = NOT PRESSED
			state = HAL_GPIO_ReadPin( digitalArray[i].port, digitalArray[i].pin );	// read buttons, button is active-low
			
			if( i <= 10 ){		// Buttons
				buttonArray[i] = !state;
				
			}else{					// D-PAD
				//dpadArray[i-11] = state;
			}
		}
		
		i++;
	}
			
	XINPUT_buttonArrayUpdate( buttonArray );																					// update buttons
	
	dpadArray[0] = !HAL_GPIO_ReadPin( digitalArray[11].port, digitalArray[11].pin );
	dpadArray[1] = !HAL_GPIO_ReadPin( digitalArray[12].port, digitalArray[12].pin );
	dpadArray[2] = !HAL_GPIO_ReadPin( digitalArray[13].port, digitalArray[13].pin );
	dpadArray[3] = !HAL_GPIO_ReadPin( digitalArray[14].port, digitalArray[14].pin );
	
	XINPUT_dpadUpdate( dpadArray[0], dpadArray[1], dpadArray[2], dpadArray[3] );			// update dpad

}

/*	Read input values from sticks and triggers
*
*/

#define STICK_16_MAX 50000
#define STICK_16_MIN -50000

#define INT16MAX 32767
#define INT16MIN -32767

void readAdcValues(){
	
	if( adcValueReady == 1 ){
		rightTriggerValue = (uint8_t)map( 0, 0, 4040, 0, UINT8_MAX );
		leftTriggerValue 	= (uint8_t)map( 0, 0, 4040, 0, UINT8_MAX );
		
		if (xLeftStickValue_ADC<2020) {
			int32_t tmp = restrict(xLeftStickValue_ADC, 752, 2020);
			xLeftStickValue = (int16_t)map( tmp, 752, 2020, INT16MIN, 0 );
		}else {
			int32_t tmp = restrict(xLeftStickValue_ADC, 3261, 2020);
			xLeftStickValue = (int16_t)map( tmp, 2020, 3261, 0, INT16MAX );
		}
		
		if (yLeftStickValue_ADC<2020) {
			int32_t tmp = restrict(yLeftStickValue_ADC, 895, 2020);
			yLeftStickValue = (int16_t)map( tmp, 895, 2020, INT16MAX, 0 );
		}else {
			int32_t tmp = restrict(yLeftStickValue_ADC, 3211, 2020);
			yLeftStickValue = (int16_t)map( tmp, 2020, 3211, 0, INT16MIN );
		}
		
		if (xRightStickValue_ADC<2020) {
			int32_t tmp = restrict(xRightStickValue_ADC, 816, 2020);
			xRightStickValue = (int16_t)map( tmp, 816, 2020, INT16MAX, 0 );
		}else {
			int32_t tmp = restrict(xRightStickValue_ADC, 3225, 2020);
			xRightStickValue = (int16_t)map( tmp, 2020, 3225, 0, INT16MIN );
		}
		
		if (yRightStickValue_ADC<2020) {
			int32_t tmp = restrict(yRightStickValue_ADC, 922, 2020);
			yRightStickValue = (int16_t)map( tmp, 922, 2020, INT16MIN, 0 );
		}else {
			int32_t tmp = restrict(yRightStickValue_ADC, 3185, 2020);
			yRightStickValue = (int16_t)map( tmp, 2020, 3185, 0, INT16MAX );
		}
		
		
		adcValueReady = 0;
	}
	
}

/*	Update brake and throttle triggers
*
*/
void updateTriggers(){
		
	XINPUT_triggerUpdate(leftTriggerValue, rightTriggerValue);
}

/*	Update right and left sticks
*			LeftStick X-axis comes from steering wheel
*			RightStick comes from potentiometers
*/
void updateSticks(){

	XINPUT_stickUpdate(STICK_LEFT, xLeftStickValue, yLeftStickValue);
	XINPUT_stickUpdate(STICK_RIGHT, xRightStickValue, yRightStickValue );
	
}

/*	Re-maps a number from one range to another
*
*/
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int32_t restrict(int32_t x, int32_t max, int32_t middle) {
	if (max>middle) {
		if (x>max) {return max;}
		else {return x;}
	} else {
	if (x<max) {return max;}
		else {return x;}
	}
}
/**
  ******************************************************************************
  * File Name          : stm32f1xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

extern DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_AFIO_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    /**NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
    */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC1 GPIO Configuration    
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_adc1);

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }

}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(hadc->DMA_Handle);
  }
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_hid_if.h"
/* USER CODE BEGIN INCLUDE */

#include "usbd_desc.h"
#include "usbd_ctlreq.h"

/* USER CODE END INCLUDE */
/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CUSTOM_HID 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */ 
/**
  * @}
  */ 

/** @defgroup USBD_CUSTOM_HID_Private_Defines
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */
  
/**
  * @}
  */ 

/** @defgroup USBD_CUSTOM_HID_Private_Macros
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */ 

/** @defgroup USBD_AUDIO_IF_Private_Variables
 * @{
 */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  /* USER CODE BEGIN 0 */ 
  0x00, 
  /* USER CODE END 0 */ 
  0xC0    /*     END_COLLECTION	             */
   
}; 
/* USB handler declaration */
/* Handle for USB Full Speed IP */
  USBD_HandleTypeDef  *hUsbDevice_0;

/* USER CODE BEGIN PRIVATE_VARIABLES */

extern uint8_t USB_RX_Buffer[8];

/* USER CODE END PRIVATE_VARIABLES */
/**
  * @}
  */ 
  
/** @defgroup USBD_CUSTOM_HID_IF_Exported_Variables
  * @{
  */ 
  extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @{
  */
static int8_t CUSTOM_HID_Init_FS     (void);
static int8_t CUSTOM_HID_DeInit_FS   (void);
static int8_t CUSTOM_HID_OutEvent_FS (uint8_t event_idx, uint8_t state);
 

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS = 
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS,
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  CUSTOM_HID_Init_FS
  *         Initializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  hUsbDevice_0 = &hUsbDeviceFS;
  /* USER CODE BEGIN 4 */ 
  return (0);
  /* USER CODE END 4 */ 
}

/**
  * @brief  CUSTOM_HID_DeInit_FS
  *         DeInitializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */ 
  return (0);
  /* USER CODE END 5 */ 
}

/**
  * @brief  CUSTOM_HID_OutEvent_FS
  *         Manage the CUSTOM HID class events       
  * @param  event_idx: event index
  * @param  state: event state
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS  (uint8_t event_idx, uint8_t state)
{ 
  /* USER CODE BEGIN 6 */ 
	
	
	USBD_CUSTOM_HID_HandleTypeDef *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDevice_0->pClassData;
	
	for( uint8_t i = 0 ; i < 8 ; i++ ){
		USB_RX_Buffer[i] = hhid->Report_buf[i];
	}
	
  return (0);
  /* USER CODE END 6 */ 
}

/* USER CODE BEGIN 7 */ 
/**
  * @brief  USBD_CUSTOM_HID_SendReport_FS
  *         Send the report to the Host       
  * @param  report: the report to be sent
  * @param  len: the report length
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
/*  
static int8_t USBD_CUSTOM_HID_SendReport_FS ( uint8_t *report,uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(hUsbDevice_0, report, len); 
}
*/
/* USER CODE END 7 */ 

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */ 

/**
  * @}
  */  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/**
  ******************************************************************************
  * @file           : USB_DEVICE  
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device 
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"

/* USB Device Core handle declaration */
USBD_HandleTypeDef hUsbDeviceFS;

/* init function */				        
void MX_USB_DEVICE_Init(void)
{
  /* Init Device Library,Add Supported Class and Start the library*/
  USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
	
  USBD_RegisterClass(&hUsbDeviceFS, &USBD_CUSTOM_HID);

  USBD_CUSTOM_HID_RegisterInterface(&hUsbDeviceFS, &USBD_CustomHID_fops_FS);
	
	USBD_CUSTOM_HID_HandleTypeDef *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;
	
  USBD_Start(&hUsbDeviceFS);
	
	
	
	
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/**
  ******************************************************************************
  * @file           : usbd_desc.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device descriptors
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_DESC 
  * @brief USBD descriptors module
  * @{
  */ 

/** @defgroup USBD_DESC_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Private_Defines
  * @{
  */
	
// (ALTERAR)
/** @defgroup USBD_DESC_Private_Defines
  * @{
  */ 
#define USBD_VID     1118
#define USBD_LANGID_STRING     1033
#define USBD_MANUFACTURER_STRING     "@Microsoft"
#define USBD_PID_FS     654
#define USBD_PRODUCT_STRING_FS     "Controller"
#define USBD_SERIALNUMBER_STRING_FS     "00000000001A"
#define USBD_CONFIGURATION_STRING_FS     "Custom HID Config"
#define USBD_INTERFACE_STRING_FS     "\xB2\x03\x58\x00\x62\x00\x6F\x00\x78\x00\x20\x00\x53\x00\x65\x00\x63\x00\x75\x00\x72\x00\x69\x00\x74\x00\x79\x00\x20\x00\x4D\x00\x65\x00\x74\x00\x68\x00\x6F\x00\x64\x00\x20\x00\x33\x00\x2C\x00\x20\x00\x56\x00\x65\x00\x72\x00\x73\x00\x69\x00\x6F\x00\x6E\x00\x20\x00\x31\x00\x2E\x00\x30\x00\x30\x00\x2C\x00\x20\x00\xA9\x00\x20\x00\x32\x00\x30\x00\x30\x00\x35\x00\x20\x00\x4D\x00\x69\x00\x63\x00\x72\x00\x6F\x00\x73\x00\x6F\x00\x66\x00\x74\x00\x20\x00\x43\x00\x6F\x00\x72\x00\x70\x00\x6F\x00\x72\x00\x61\x00\x74\x00\x69\x00\x6F\x00\x6E\x00\x2E\x00\x20\x00\x41\x00\x6C\x00\x6C\x00\x20\x00\x72\x00\x69\x00\x67\x00\x68\x00\x74\x00\x73\x00\x20\x00\x72\x00\x65\x00\x73\x00\x65\x00\x72\x00\x76\x00\x65\x00\x64\x00\x2E\x00"				// (ALTERAR)

/* USER CODE BEGIN 0 */

/* USER CODE END 0*/
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Private_Variables
  * @{
  */ 
uint8_t *     USBD_FS_DeviceDescriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_FS_LangIDStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_FS_ManufacturerStrDescriptor ( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_FS_ProductStrDescriptor ( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_FS_SerialStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_FS_ConfigStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_FS_InterfaceStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length);

#ifdef USB_SUPPORT_USER_STRING_DESC
uint8_t *     USBD_FS_USRStringDesc (USBD_SpeedTypeDef speed, uint8_t idx , uint16_t *length);  
#endif /* USB_SUPPORT_USER_STRING_DESC */  

USBD_DescriptorsTypeDef FS_Desc =
{
  USBD_FS_DeviceDescriptor,
  USBD_FS_LangIDStrDescriptor, 
  USBD_FS_ManufacturerStrDescriptor,
  USBD_FS_ProductStrDescriptor,
  USBD_FS_SerialStrDescriptor,
  USBD_FS_ConfigStrDescriptor,
  USBD_FS_InterfaceStrDescriptor,
};


// (ALTERAR)
// USB Device Descriptor.  The USB host reads this first, to learn
// what type of device is connected.
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4   
#endif
/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
  {
    18,                       /*bLength */
    1,       									/*bDescriptorType*/
    0x00,                     /* bcdUSB */  
    0x02,
    DEVICE_CLASS,             /*bDeviceClass*/
    DEVICE_SUBCLASS,          /*bDeviceSubClass*/
    DEVICE_PROTOCOL,          /*bDeviceProtocol*/
    USB_MAX_EP0_SIZE,         /*bMaxPacketSize*/			// --> EP0_SIZE dando problema
    LOBYTE(VENDOR_ID),        /*idVendor*/
    HIBYTE(VENDOR_ID),        /*idVendor*/
    LOBYTE(PRODUCT_ID),       /*idVendor*/
    HIBYTE(PRODUCT_ID),       /*idVendor*/
    LOBYTE(DEVICE_VERSION),   /*bcdDevice rel. 2.00*/
    HIBYTE(DEVICE_VERSION),
    1,           							/*Index of manufacturer  string*/
    2,								       	/*Index of product string*/
    3,        								/*Index of serial number string*/
    1  												/*bNumConfigurations*/
  } ; 
/* USB_DeviceDescriptor */

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4   
#endif

/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
     USB_LEN_LANGID_STR_DESC,         
     USB_DESC_TYPE_STRING,       
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING), 
};

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4   
#endif
__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Private_FunctionPrototypes
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Private_Functions
  * @{
  */ 

/**
* @brief  USBD_FS_DeviceDescriptor 
*         return the device descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_FS_DeviceDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  *length = sizeof(USBD_FS_DeviceDesc);
  return USBD_FS_DeviceDesc;
}

/**
* @brief  USBD_FS_LangIDStrDescriptor 
*         return the LangID string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_FS_LangIDStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  *length =  sizeof(USBD_LangIDDesc);  
  return USBD_LangIDDesc;
}

/**
* @brief  USBD_FS_ProductStrDescriptor 
*         return the product string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_FS_ProductStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  if(speed == 0)
  {   
    USBD_GetString (USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString (USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);    
  }
  return USBD_StrDesc;
}

/**
* @brief  USBD_FS_ManufacturerStrDescriptor 
*         return the manufacturer string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_FS_ManufacturerStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  USBD_GetString (USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

/**
* @brief  USBD_FS_SerialStrDescriptor 
*         return the serial number string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_FS_SerialStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  if(speed  == USBD_SPEED_HIGH)
  {    
    USBD_GetString (USBD_SERIALNUMBER_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString (USBD_SERIALNUMBER_STRING_FS, USBD_StrDesc, length);    
  }
  return USBD_StrDesc;
}

/**
* @brief  USBD_FS_ConfigStrDescriptor 
*         return the configuration string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_FS_ConfigStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  if(speed  == USBD_SPEED_HIGH)
  {  
    USBD_GetString (USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString (USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length); 
  }
  return USBD_StrDesc;  
}

/**
* @brief  USBD_HS_InterfaceStrDescriptor 
*         return the interface string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_FS_InterfaceStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString (USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString (USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;  
}
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/**
  ******************************************************************************
  * @file           : usbd_conf.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the board support package for the USB device library
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "usbd_def.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_customhid.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
void HAL_PCDEx_SetConnectionState(PCD_HandleTypeDef *hpcd, uint8_t state);

/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/
/* MSP Init */

void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)
{
  if(hpcd->Instance==USB)
  {
  /* USER CODE BEGIN USB_MspInit 0 */

  /* USER CODE END USB_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USB_CLK_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN USB_MspInit 1 */

  /* USER CODE END USB_MspInit 1 */
  }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef* hpcd)
{
  if(hpcd->Instance==USB)
  {
  /* USER CODE BEGIN USB_MspDeInit 0 */

  /* USER CODE END USB_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USB_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

  /* USER CODE BEGIN USB_MspDeInit 1 */

  /* USER CODE END USB_MspDeInit 1 */
  }
}

/**
  * @brief  Setup Stage callback
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SetupStage((USBD_HandleTypeDef*)hpcd->pData, (uint8_t *)hpcd->Setup);
}

/**
  * @brief  Data Out Stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataOutStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

/**
  * @brief  Data In Stage callback..
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataInStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

/**
  * @brief  SOF callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SOF((USBD_HandleTypeDef*)hpcd->pData);
}

/**
  * @brief  Reset callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{ 
  USBD_SpeedTypeDef speed = USBD_SPEED_FULL;

  /*Set USB Current Speed*/
  switch (hpcd->Init.speed)
  {
  case PCD_SPEED_FULL:
    speed = USBD_SPEED_FULL;    
    break;
	
  default:
    speed = USBD_SPEED_FULL;    
    break;    
  }
  USBD_LL_SetSpeed((USBD_HandleTypeDef*)hpcd->pData, speed);  
  
  /*Reset Device*/
  USBD_LL_Reset((USBD_HandleTypeDef*)hpcd->pData);
}

/**
  * @brief  Suspend callback.
  * When Low power mode is enabled the debug cannot be used (IAR, Keil doesn't support it)
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
  /* Inform USB library that core enters in suspend Mode */
  USBD_LL_Suspend((USBD_HandleTypeDef*)hpcd->pData);
  /*Enter in STOP mode */
  /* USER CODE BEGIN 2 */  
  if (hpcd->Init.low_power_enable)
  {
    /* Set SLEEPDEEP bit and SleepOnExit of Cortex System Control Register */
    SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
  }
  /* USER CODE END 2 */
}

/**
  * @brief  Resume callback.
  * When Low power mode is enabled the debug cannot be used (IAR, Keil doesn't support it)
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
  USBD_LL_Resume((USBD_HandleTypeDef*)hpcd->pData);
  
}

/**
  * @brief  ISOOUTIncomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum);
}

/**
  * @brief  ISOINIncomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoINIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum);
}

/**
  * @brief  ConnectCallback callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevConnected((USBD_HandleTypeDef*)hpcd->pData);
}

/**
  * @brief  Disconnect callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevDisconnected((USBD_HandleTypeDef*)hpcd->pData);
}

/*******************************************************************************
                       LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/
/**
  * @brief  Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_Init (USBD_HandleTypeDef *pdev)
{ 
  /* Init USB_IP */
  /* Link The driver to the stack */
  hpcd_USB_FS.pData = pdev;
  pdev->pData = &hpcd_USB_FS;

  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.ep0_mps = DEP0CTL_MPS_8;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  HAL_PCD_Init(&hpcd_USB_FS);

  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x00 , PCD_SNG_BUF, 0x18);
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x80 , PCD_SNG_BUF, 0x58);
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x81 , PCD_SNG_BUF, 0x98);  
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , CUSTOM_HID_EPOUT_ADDR , PCD_SNG_BUF, 0xD8); 
	
	HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x02 , PCD_SNG_BUF, 0x118);											// endpoint that receives led
	HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x83 , PCD_SNG_BUF, 0x158);
	HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x04 , PCD_SNG_BUF, 0x158);
	HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x85 , PCD_SNG_BUF, 0x198);
	HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x05 , PCD_SNG_BUF, 0x1D8);
	HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x86 , PCD_SNG_BUF, 0x218);
	
	
	//HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , XINPUT_RX_ENDPOINT , PCD_SNG_BUF, 0x98);  
	//HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , XINPUT_TX_ENDPOINT , PCD_SNG_BUF, 0xD8); 

/*
HAL_StatusTypeDef  HAL_PCDEx_PMAConfig(PCD_HandleTypeDef *hpcd, 
                                       uint16_t ep_addr,
                                       uint16_t ep_kind,
                                       uint32_t pmaadress)

	*/	
  return USBD_OK;
}

/**
  * @brief  De-Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_DeInit (USBD_HandleTypeDef *pdev)
{
  HAL_PCD_DeInit((PCD_HandleTypeDef*)pdev->pData);
  return USBD_OK;
}

/**
  * @brief  Starts the Low Level portion of the Device driver. 
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Start((PCD_HandleTypeDef*)pdev->pData);
  return USBD_OK;
}

/**
  * @brief  Stops the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_Stop (USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Stop((PCD_HandleTypeDef*) pdev->pData);
  return USBD_OK;
}

/**
  * @brief  Opens an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  ep_type: Endpoint Type
  * @param  ep_mps: Endpoint Max Packet Size
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_OpenEP  (USBD_HandleTypeDef *pdev, 
                                      uint8_t  ep_addr,                                      
                                      uint8_t  ep_type,
                                      uint16_t ep_mps)
{
  HAL_PCD_EP_Open((PCD_HandleTypeDef*) pdev->pData,
                  ep_addr,
                  ep_mps,
                  ep_type);
  
  return USBD_OK; 
}

/**
  * @brief  Closes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_CloseEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  HAL_PCD_EP_Close((PCD_HandleTypeDef*) pdev->pData, ep_addr);
  return USBD_OK;
}

/**
  * @brief  Flushes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_FlushEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  HAL_PCD_EP_Flush((PCD_HandleTypeDef*) pdev->pData, ep_addr);
  return USBD_OK;
}

/**
  * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_StallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  HAL_PCD_EP_SetStall((PCD_HandleTypeDef*) pdev->pData, ep_addr);
  return USBD_OK;
}

/**
  * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_ClearStallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  HAL_PCD_EP_ClrStall((PCD_HandleTypeDef*) pdev->pData, ep_addr);
  return USBD_OK; 
}

/**
  * @brief  Returns Stall condition.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Stall (1: Yes, 0: No)
  */
uint8_t USBD_LL_IsStallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*) pdev->pData;
  
  if((ep_addr & 0x80) == 0x80)
  {
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall; 
  }
  else
  {
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall; 
  }
}
/**
  * @brief  Assigns a USB address to the device.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_SetUSBAddress (USBD_HandleTypeDef *pdev, uint8_t dev_addr)   
{
  HAL_PCD_SetAddress((PCD_HandleTypeDef*) pdev->pData, dev_addr);
  return USBD_OK; 
}

/**
  * @brief  Transmits data over an endpoint.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be sent
  * @param  size: Data size    
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_Transmit (USBD_HandleTypeDef *pdev, 
                                      uint8_t  ep_addr,                                      
                                      uint8_t  *pbuf,
                                      uint16_t  size)
{
  HAL_PCD_EP_Transmit((PCD_HandleTypeDef*) pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

/**
  * @brief  Prepares an endpoint for reception.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be received
  * @param  size: Data size
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, 
                                           uint8_t  ep_addr,                                      
                                           uint8_t  *pbuf,
                                           uint16_t  size)
{
  HAL_PCD_EP_Receive((PCD_HandleTypeDef*) pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

/**
  * @brief  Returns the last transfered packet size.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Recived Data Size
  */
uint32_t USBD_LL_GetRxDataSize  (USBD_HandleTypeDef *pdev, uint8_t  ep_addr)  
{
  return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef*) pdev->pData, ep_addr);
}

/**
  * @brief  Delays routine for the USB Device Library.
  * @param  Delay: Delay in ms
  * @retval None
  */
void  USBD_LL_Delay (uint32_t Delay)
{
  HAL_Delay(Delay);  
}

/**
  * @brief  static single allocation.
  * @param  size: size of allocated memory
  * @retval None
  */
void *USBD_static_malloc(uint32_t size)
{
  static uint32_t mem[(sizeof(USBD_CUSTOM_HID_HandleTypeDef)/4+1)];//On 32-bit boundary
  return mem;
}

/**
  * @brief  Dummy memory free
  * @param  *p pointer to allocated  memory address
  * @retval None
  */
void USBD_static_free(void *p)
{
}

/**
* @brief Software Device Connection
* @param hpcd: PCD handle
* @param state: connection state (0 : disconnected / 1: connected) 
* @retval None
*/
void HAL_PCDEx_SetConnectionState(PCD_HandleTypeDef *hpcd, uint8_t state)
{
/* USER CODE BEGIN 5 */
  if (state == 1)
  {
    /* Configure Low Connection State */
	
  }
  else
  {
    /* Configure High Connection State */
   
  } 
/* USER CODE END 5 */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
