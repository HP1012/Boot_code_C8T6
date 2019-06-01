#include "main.h"
#include "usb_device.h"

#include "usbd_cdc.h"
#include "usbd_cdc_if.h"                                     // y: phan hoi ok, v: phan hoi bi loi
//a																														//s: nhan	lenh start, r: lenh run
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#define adrr_read_program 0x08005000  /* as example */
#define adrr_program 0x00008000
#define size_page_flash 400
uint32_t adrr_base_start = 0x08000000 ;


#define IS_AF(c)  ((c >= 'A') && (c <= 'F'))
#define IS_af(c)  ((c >= 'a') && (c <= 'f'))
#define IS_09(c)  ((c >= '0') && (c <= '9'))
#define ISVALIDHEX(c)  IS_AF(c) || IS_af(c) || IS_09(c)
#define ISVALIDDEC(c)  IS_09(c)
#define CONVERTDEC(c)  (c - '0')

#define CONVERTHEX_alpha(c)  (IS_AF(c) ? (c - 'A'+10) : (c - 'a'+10))
#define CONVERTHEX(c)   (IS_09(c) ? (c - '0') : CONVERTHEX_alpha(c))


UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
#define APP_RX_DATA_SIZE  1000
#define APP_TX_DATA_SIZE  1000
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
//----------------usb--------------------------------
static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};
//-----------------------------end usb--------------
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
uint32_t Str2Int(uint8_t *inputstr, int32_t *intnum);


typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;

uint64_t waiting=0;
uint8_t p,write_app =0,data_rece[45]={0},dem_rece=0,adrr_update_program_array[11],data_update_program_array[11], fl_start_up=0,FL_BOOT=0;
int32_t USER_APPLICATION_BASE_ADDRESS = 0xffffffff, adrr_update_program=0,data_update_program=0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void printf_usb(const char *args, ... )
{
  char StrBuff[100];
  
  va_list ap;
  va_start(ap, args);
  char len = vsnprintf(StrBuff, sizeof(StrBuff), args, ap);
  va_end(ap);
	CDC_Transmit_FS((uint8_t *)StrBuff,strlen(StrBuff));
}

void printf_uart_1(const char *args, ... )
{
  char StrBuff[100];
  
  va_list ap;
  va_start(ap, args);
  char len = vsnprintf(StrBuff, sizeof(StrBuff), args, ap);
  va_end(ap);
	HAL_UART_Transmit(&huart1,(uint8_t *)StrBuff,strlen(StrBuff),100);
}



uint32_t Read_Flash(uint32_t adr)
	{
		uint32_t*Pntr = (uint32_t*)adr;
		return (*Pntr);
	}
	
	void Erase_Flash(uint32_t adr)
{
		FLASH->CR |= (1<<1);
		FLASH->AR = adr;
	
		FLASH->CR |= FLASH_CR_STRT;
		while( (FLASH->SR&0x00000001));
		FLASH->CR &= ~(1<<1);
}

void Write_Flash(uint32_t adr, uint32_t data)
{
	//FLASH->CR |= 0x00000001;
	FLASH->CR |= FLASH_CR_PG;
	//FLASH->AR=adr;
	//FLASH->CR |= (1<<9);
	uint16_t tamp = data >>16;
	*(__IO uint16_t*)(adr+2) = tamp;
	*(__IO uint16_t*)(adr) = data;
	
		while( (FLASH->SR&0x00000001) );
	FLASH->CR &= ~FLASH_CR_PG;
	
//	FLASH->CR |= FLASH_CR_PG;
//	*(__IO uint16_t*)(adr ) = data;
//	
//	while( (FLASH->SR&0x00000001) );
//	FLASH->CR &= ~FLASH_CR_PG;

}
void KIN1_SoftwareReset(void)
{
  SCB->AIRCR = (0x5FA<<SCB_AIRCR_VECTKEY_Pos)|SCB_AIRCR_SYSRESETREQ_Msk;
  for(;;) {
    /* wait until reset */
  }
}
void __svc( 0 ) EnablePrivilegedMode( void ) ;



static void BootJump( uint32_t *Address )
{
//1.Make sure, the CPU is in privileged mode.
  if( CONTROL_nPRIV_Msk & __get_CONTROL( ) )
  {  /* not in privileged mode */
    EnablePrivilegedMode( ) ;
  }

//2.Disable all enabled interrupts in NVIC.
NVIC->ICER[ 0 ] = 0xFFFFFFFF ;
NVIC->ICER[ 1 ] = 0xFFFFFFFF ;
NVIC->ICER[ 2 ] = 0xFFFFFFFF ;
NVIC->ICER[ 3 ] = 0xFFFFFFFF ;
NVIC->ICER[ 4 ] = 0xFFFFFFFF ;
NVIC->ICER[ 5 ] = 0xFFFFFFFF ;
NVIC->ICER[ 6 ] = 0xFFFFFFFF ;
NVIC->ICER[ 7 ] = 0xFFFFFFFF ;

//3.Clear all pending interrupt requests in NVIC.
NVIC->ICPR[ 0 ] = 0xFFFFFFFF ;
NVIC->ICPR[ 1 ] = 0xFFFFFFFF ;
NVIC->ICPR[ 2 ] = 0xFFFFFFFF ;
NVIC->ICPR[ 3 ] = 0xFFFFFFFF ;
NVIC->ICPR[ 4 ] = 0xFFFFFFFF ;
NVIC->ICPR[ 5 ] = 0xFFFFFFFF ;
NVIC->ICPR[ 6 ] = 0xFFFFFFFF ;
NVIC->ICPR[ 7 ] = 0xFFFFFFFF ;
//4.Disable SysTick and clear its exception pending bit, if it is used in the bootloader, e. g. by the RTX.
SysTick->CTRL = 0 ;
SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk ;
//5.Disable individual fault handlers if the bootloader used them.
SCB->SHCSR &= ~( SCB_SHCSR_USGFAULTENA_Msk |SCB_SHCSR_BUSFAULTENA_Msk |SCB_SHCSR_MEMFAULTENA_Msk ) ;
//6.Activate the MSP, if the core is found to currently run with the PSP.
if( CONTROL_SPSEL_Msk & __get_CONTROL( ) )
{  /* MSP is not active */
  __set_CONTROL( __get_CONTROL( ) & ~CONTROL_SPSEL_Msk ) ;
}
//7.Load the vector table address of the user application into SCB->VTOR register. Make sure the address meets the alignment requirements.
SCB->VTOR = ( uint32_t )Address ;

	__set_MSP( Address[ 0 ] ) ;
//8.Set the PC to the reset vector value of the user application via a function call.
	( ( void ( * )( void ) )Address[ 1 ] )( ) ;


}

void USART1_IRQHandler(void)
{
	
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
	HAL_UART_Receive_IT(&huart1,&p,1);

	switch (p) 
		{
			case 'u':  // lenh update
			{
				FL_BOOT=1;
				break;
			}
		case 's':  // lenh update
			{
				fl_start_up=1;
				printf_uart_1("y"); // ok
				break;
			}
		case 'r': // run
			{
				if(fl_start_up==1)
					{
						fl_start_up=2;
						dem_rece=0;
						write_app=1;
					}
				
				break;
			}
		case 'z':  // ket thuc
			{
				write_app =0;
				Erase_Flash(adrr_read_program);
				Write_Flash(adrr_read_program,0x08008000);
				KIN1_SoftwareReset();
				break;
			}
		default:
		{
			if(write_app ==1)
			{
				if(p=='\r' || p == '\n')  // /r/n
				{
					if( data_rece[0] != ':' && (data_rece[2] != '0' || data_rece[2] != '8' ) ) printf_uart_1("v");
					else if(data_rece[0] == ':' && data_rece[2] == '2'&& data_rece[12] == '0' ) adrr_base_start=0x08000000;
					else if(data_rece[0] == ':' && data_rece[2] == '2'&& data_rece[12] == '1' ) adrr_base_start=0x08010000;
					else
					{
						//adrr_update_program_array[5]=data_rece[2];
						adrr_update_program_array[6]=data_rece[3];adrr_update_program_array[7]=data_rece[4];
						adrr_update_program_array[8]=data_rece[5];adrr_update_program_array[9]=data_rece[6];  // lay dia chi nap code
						Str2Int(adrr_update_program_array,&adrr_update_program);  // convect string to int adrr
						adrr_update_program = adrr_update_program + adrr_base_start; 

						
						uint8_t adrr=0;
						for(uint8_t tamp_nho=9;tamp_nho <37; tamp_nho+=8)
						{
							if(data_rece[tamp_nho+7]==0x00) break;
							data_update_program_array[2] = data_rece[tamp_nho+6];data_update_program_array[3] = data_rece[tamp_nho+7];
							data_update_program_array[4] = data_rece[tamp_nho+4];data_update_program_array[5] = data_rece[tamp_nho+5];
							data_update_program_array[6] = data_rece[tamp_nho+2];data_update_program_array[7] = data_rece[tamp_nho+3];
							data_update_program_array[8] = data_rece[tamp_nho+0];data_update_program_array[9] = data_rece[tamp_nho+1];
							Str2Int(data_update_program_array,&data_update_program);
							if(adrr_update_program >= 0x08008000)
							{
								Write_Flash(adrr_update_program+adrr ,data_update_program) ;
							}
							adrr += 4;
						}
					}
						
					dem_rece=0;
					printf_uart_1("y"); // ok
					for(int i=0;i<45;i++) data_rece[i]=0x00;
				}
				else
				{
					data_rece[dem_rece] = p;
					dem_rece++;
				}
			}
			break;
		}
		
		}
	
	
}

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)  // nap code bang usb
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	
	//-------------------------------------------------------------------
	switch (Buf[0]) 
		{
			case 'u':  // lenh update
			{
				FL_BOOT=1;
				break;
			}
			case 's':  // lenh update
			{
				fl_start_up=1;
				printf_usb("y"); // ok
				break;
			}
		case 'r': // run
			{
				if(fl_start_up==1)
					{
						fl_start_up=2;
						write_app=1;
						dem_rece=0;
					}
				break;
			}
		case 'z':  // ket thuc
			{
				write_app =0;
				Erase_Flash(adrr_read_program);
				Write_Flash(adrr_read_program,0x08008000);
				KIN1_SoftwareReset();
				break;
			}
		}
		//--------------------------
			if(write_app ==1)
			{
					if( Buf[0] != ':' && (Buf[2] != '0' || Buf[2] != '8' ) ) printf_usb("v");
					else if(Buf[0] == ':' && Buf[2] == '2'&& Buf[12] == '0' ) adrr_base_start=0x08000000;
					else if(Buf[0] == ':' && Buf[2] == '2'&& Buf[12] == '1' ) adrr_base_start=0x08010000;
					else
					{
						//adrr_update_program_array[5]=data_rece[2];
						adrr_update_program_array[6]=Buf[3];adrr_update_program_array[7]=Buf[4];
						adrr_update_program_array[8]=Buf[5];adrr_update_program_array[9]=Buf[6];  // lay dia chi nap code
						Str2Int(adrr_update_program_array,&adrr_update_program);  // convect string to int adrr
						adrr_update_program = adrr_update_program + adrr_base_start; 
					/////////////get data up to flash
						uint8_t adrr=0;
						for(uint8_t tamp_nho=9;tamp_nho <37; tamp_nho+=8)
						{
							if(Buf[tamp_nho+7]==0x00) break;
							data_update_program_array[2] = Buf[tamp_nho+6];data_update_program_array[3] = Buf[tamp_nho+7];
							data_update_program_array[4] = Buf[tamp_nho+4];data_update_program_array[5] = Buf[tamp_nho+5];
							data_update_program_array[6] = Buf[tamp_nho+2];data_update_program_array[7] = Buf[tamp_nho+3];
							data_update_program_array[8] = Buf[tamp_nho+0];data_update_program_array[9] = Buf[tamp_nho+1];
							Str2Int(data_update_program_array,&data_update_program);
							if(adrr_update_program >= 0x08008000)
							{
								Write_Flash(adrr_update_program+adrr ,data_update_program) ;
							}
							adrr += 4;
						}
					}
					printf_usb("y"); // ok
					for(int i=0;i<45;i++) Buf[i]=0x00;
			}
		
		
		
	
	//-------------------------------------------------------------------
 
  return (USBD_OK);
}

int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
	MX_USB_DEVICE_Init();
	HAL_UART_Receive_IT(&huart1,&p,1);	
	//------------------------
	HAL_FLASH_Unlock();
//	HAL_FLASH_OB_Unlock();
//	FLASH_OB_RDP_LevelConfig(OB_RDP_LEVEL_1);
//	HAL_FLASH_OB_Lock();
	//------------------------
	RCC->APB1ENR |= (1<< 27);
	PWR->CR |= (1<<8);
	//------------------------
	
	
	HAL_Delay(500);
	printf_usb("Press the BOOT button to enter setup\r\n");
	printf_uart_1("Press the BOOT button to enter setup\r\n");
	while(waiting < 10000000)
	{
			waiting++;
			if(FL_BOOT==1) break;
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == 0) break;
			if(BKP->DR10 == 0x1234){waiting=10000000;break;}
	}
	if(waiting <10000000)
	{
		BKP->DR10 = 0x1234;
		printf_usb("-----AIGREEN CODE V-1.0 (12-2018)---\r\n+-----------------------Boot start----------------------+\r\n");
		
		printf_uart_1("-----AIGREEN CODE V-1.0 (12-2018)---\r\n+-----------------------Boot start---------------------+\r\n");		
		adrr_update_program_array[0]='0';adrr_update_program_array[1]='x';
		adrr_update_program_array[2]='0';adrr_update_program_array[3]='0';
		adrr_update_program_array[4]='0';adrr_update_program_array[5]='0';
		
		data_update_program_array[0] ='0';data_update_program_array[1] ='x';
		for(int i=0;i<45;i++) data_rece[i]=0xff;
	}
	else
	{
		BKP->DR10 = 0x1234;
		printf_usb("start...\r\n");
		printf_uart_1("start...\r\n");
		USER_APPLICATION_BASE_ADDRESS = Read_Flash(adrr_read_program);
		if(USER_APPLICATION_BASE_ADDRESS != 0xffffffff)
		{
			printf_usb("Program found in Flash adrr : %X\r\n",USER_APPLICATION_BASE_ADDRESS);
			printf_uart_1("Program found in Flash adrr : %X\r\n",USER_APPLICATION_BASE_ADDRESS);
			HAL_Delay(100);
			BootJump((uint32_t * )adrr_program);
		}
		else
		{
			printf_usb("Program not found in Flash\r\n");
			printf_uart_1("Program not found in Flash\r\n");		
		}
	}
	  
	//-----------------------------------------
  while (1)
  {
		
		if(fl_start_up ==2)  // xoa flash
		{
			uint8_t dem=0;
			for(uint32_t i = 0x08008000; i <= 0x0801FC00;i = i+ 1024)
			{
				Erase_Flash(i);
				HAL_Delay(10);
				dem++;
				if(dem==3){HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);dem=0;}
			}
			fl_start_up=0;
			printf_uart_1("y");
			printf_usb("y");
		}
		if(write_app ==1){
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		HAL_Delay(15);
		}
  }
	//-----------------------------------------
}


uint32_t Str2Int(uint8_t *inputstr, int32_t *intnum)
{
  uint32_t i = 0, res = 0;
  uint32_t val = 0;

  if (inputstr[0] == '0' && (inputstr[1] == 'x' || inputstr[1] == 'X'))
  {
    if (inputstr[2] == '\0')
    {
      return 0;
    }
    for (i = 2; i < 11; i++)
    {
      if (inputstr[i] == '\0')
      {
        *intnum = val;
        /* return 1; */
        res = 1;
        break;
      }
      if (ISVALIDHEX(inputstr[i]))
      {
        val = (val << 4) + CONVERTHEX(inputstr[i]);
      }
      else
      {
        /* return 0, Invalid input */
        res = 0;
        break;
      }
    }
    /* over 8 digit hex --invalid */
    if (i >= 11)
    {
      res = 0;
    }
  }
  else /* max 10-digit decimal input */
  {
    for (i = 0;i < 11;i++)
    {
      if (inputstr[i] == '\0')
      {
        *intnum = val;
        /* return 1 */
        res = 1;
        break;
      }
      else if ((inputstr[i] == 'k' || inputstr[i] == 'K') && (i > 0))
      {
        val = val << 10;
        *intnum = val;
        res = 1;
        break;
      }
      else if ((inputstr[i] == 'm' || inputstr[i] == 'M') && (i > 0))
      {
        val = val << 20;
        *intnum = val;
        res = 1;
        break;
      }
      else if (ISVALIDDEC(inputstr[i]))
      {
        val = val * 10 + CONVERTDEC(inputstr[i]);
      }
      else
      {
        /* return 0, Invalid input */
        res = 0;
        break;
      }
    }
    /* Over 10 digit decimal --invalid */
    if (i >= 11)
    {
      res = 0;
    }
  }

  return res;
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	/*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */



uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
