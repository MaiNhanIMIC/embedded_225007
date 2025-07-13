/*
 * main.c
 *
 *  Created on: Mar 23, 2025
 *      Author: Dell
 */
#include "main.h"
#include <string.h>
#define GPIOD_BASE_ADDR  0x40020C00
int cnt = 1000;
void LedInit()
{
	__HAL_RCC_GPIOD_CLK_ENABLE();
	uint32_t* GPIOD_MODER = (uint32_t*)(GPIOD_BASE_ADDR + 0x00);
	*GPIOD_MODER &= ~(0xff << 24);
	*GPIOD_MODER |= (0b01 << 24)| (0b01 << 26)|(0b01 << 28)|(0b01 << 30);
}
#define GPIOA_BASE_ADDR  0x40020000

void ButtonInit()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
	*GPIOA_MODER &= ~(0b11 << 0);
}
typedef enum {
	LED_GREEN = 12,
	LED_ORANGE,
	LED_RED,
	LED_BLUE
}LED_t;

void LedCtrl(LED_t led, int on_off)
{
#if 0
	uint32_t* GPIOD_ODR = (uint32_t*)(GPIOD_BASE_ADDR + 0x14);
	if(on_off == 1)
	{
		*GPIOD_ODR |= (1 << led);
	}
	else
	{
		*GPIOD_ODR &= ~(1<<led);
	}
#else
	uint32_t* GPIOD_BSRR = (uint32_t*)(GPIOD_BASE_ADDR + 0x18);
	if(on_off == 1)
	{
		*GPIOD_BSRR |= (1<<led);
	}
	else
	{
		*GPIOD_BSRR |= (1<<(led+16));
	}
#endif
}

char ButtonState()
{
	uint32_t* GPIOA_IDR = (uint32_t*)(GPIOA_BASE_ADDR + 0x10);
	return (*GPIOA_IDR >> 0) & 1;
}

void function()
{
	if(ButtonState())
		LedCtrl(LED_RED, 1);
	else
		LedCtrl(LED_RED, 0);
}

#define EXTI_BASE_ADDR	0x40013C00
void EXTI0Init()
{
	/*
	- Làm sao để EXTI0 gửi interrupt signal lên ARM?
		+ chọn cạnh (rising/falling/cả hai)
			+ set trong thanh ghi EXTI_RTSR và EXTI_FTSR
		+ enable exti0(set mark)
			+ set trong thanh ghi EXTI_IMR
	- ARM (NVIC) phải chấp nhận interrupt signal từ EXTI gửi lên?
		+ bước 1: xác định EXTI0 nằm ở position bao nhiêu trong vector table? (mở vector table ở chapter "10: interrupts and events" trong reference manual) --> 6
		+ bước 2: enable interrupt cho position 6
	*/
	uint32_t* EXTI_RTSR = (uint32_t*)(EXTI_BASE_ADDR + 0x08);
	*EXTI_RTSR |= (1<<0);
	uint32_t* EXTI_FTSR = (uint32_t*)(EXTI_BASE_ADDR + 0x0C);
	*EXTI_FTSR |= (1<<0);
	uint32_t* EXTI_IMR = (uint32_t*)(EXTI_BASE_ADDR + 0x00);
	*EXTI_IMR |= (1<<0);

	uint32_t* NVIC_ISER0 = (uint32_t*)0xE000E100;
	*NVIC_ISER0 |= 1<<6;

	// Move vector table lên RAM (0x20000000)
	uint8_t* src = 0;
	uint8_t* dis = (uint8_t*)0x20000000;
	for(int i = 0; i < 0x198; i++)
	{
		//dis[i] = src[i];
		*(dis+i) = *(src+i);
	}
	//Báo ARM vector table đã được offset lên RAM
	uint32_t* VTOR = (uint32_t*)0xE000ED08;
	*VTOR = 0x20000000;
	//
	int* ptr;
	ptr= (int*)0x20000058;
	*ptr = (int)function;


}

void EXTI0_IRQHandler()
{
	if(ButtonState())
		LedCtrl(LED_RED, 1);
	else
		LedCtrl(LED_RED, 0);

	//clear interrupt flag
	uint32_t* EXTI_PR = (uint32_t*)(EXTI_BASE_ADDR + 0x14);
	*EXTI_PR |= (1<<0);
}
#define DMA2_BASE_ADDR	0x40026400

char rx_buf[3268];
void DMA_Init()
{
	__HAL_RCC_DMA2_CLK_ENABLE();
	// use DMA2, stream 2, channel 4 for UART1_Rx

	// địa chỉ người nhận -> thanh ghi nào?
	uint32_t* DMA_S2M0AR = (uint32_t*)(DMA2_BASE_ADDR + 0x1C + 0x18 *2);
	*DMA_S2M0AR = rx_buf;

	// địa chỉ người gởi -> thanh ghi nào?
	uint32_t* DMA_S2PAR = (uint32_t*)(DMA2_BASE_ADDR + 0x18 + 0x18 * 2);
	*DMA_S2PAR = 0x40011004;

	// kích thước gói data -> thanh ghi nào?
	uint32_t* DMA_S2NDTR = (uint32_t*)(DMA2_BASE_ADDR + 0x14 + 0x18 * 2);
	*DMA_S2NDTR = sizeof(rx_buf);

	// enable DMA2, stream 2, channel 4
	uint32_t* DMA_S2CR = (uint32_t*)(DMA2_BASE_ADDR + 0x10 + 0x18 * 2);
	*DMA_S2CR &= ~(0b111<< 25);
	*DMA_S2CR |= (0b100 << 25); 	// select channel 4 for stream 2
	*DMA_S2CR |= (1 << 10);			// enable Memory increment mode
	*DMA_S2CR |= (1 << 8);
	*DMA_S2CR |= (1 << 4);			// enable transfer complete interrupt
	*DMA_S2CR |= (1 << 0); 			// enable stream 2

	int32_t* ISER1 = (uint32_t*)(0xE000E104);
	*ISER1 |= 1 << 26;
}
int update_firmware;
void DMA2_Stream2_IRQHandler()
{
	uint32_t* DMA_LIFCR = (uint32_t*)(DMA2_BASE_ADDR + 0x08);
	*DMA_LIFCR |= 1 << 21;
	update_firmware = 1;

}
#define UART1_BASE_ADDR	0x40011000
#define GPIOB_BASE_ADDR	0x40020400
void UART_Init()
{

	/* 	CONFIG GPIOB
		set PB6 as UART1_Tx, PB7 as UART1_Rx
		PB6 alternate function 07
		PB7 alternate function 07
	*/
	__HAL_RCC_GPIOB_CLK_ENABLE();
	uint32_t* GPIOB_MODER = (uint32_t*)(GPIOB_BASE_ADDR + 0x00);
	*GPIOB_MODER &= ~(0b1111 << 12);
	*GPIOB_MODER |= (0b10 << 12) | (0b10 << 14);

	uint32_t* GPIOB_AFLR = (uint32_t*)(GPIOB_BASE_ADDR + 0x20);
	*GPIOB_AFLR &= ~(0xff << 24);
	*GPIOB_AFLR |= (0b0111 << 24) | (0b0111 << 28);

	/* 	CONFIG UART
		- set baud rate: 9600 (bps)
		- data frame:
			+ data size: 8bit	-> CR1 bit 12 M =
			+ parity: even		-> CR1 bit 10 PCE = 1 (parity control enable)
						-> CR1 bit 9  PS = 0 (parity selection)
		- Enable transmitter, receiver	-> CR1 bit 3 TE, bit 2 RE
		- Enable UART			-> CR1 bit 13 UE
	*/
	__HAL_RCC_USART1_CLK_ENABLE();	// 16Mhz
	uint32_t* BRR = (uint32_t*)(UART1_BASE_ADDR + 0x08);
	*BRR = (104 << 4) | (3 << 0);

	uint32_t* CR1 = (uint32_t*)(UART1_BASE_ADDR + 0x0C);
	*CR1 |= (1 << 12) | (1 << 10) | (1 << 3) | (1 << 2) | (1 << 13);
#if  0
	/* enable interrupt */
	*CR1 |= (1 << 5);
	uint32_t* NVIC_ISER1 = (uint32_t*)0xE000E104;
	*NVIC_ISER1 |= 1<<5;
#else
	uint32_t* CR3 = (uint32_t*)(UART1_BASE_ADDR + 0x14);
	*CR3 |= (1 << 6);
	DMA_Init();
#endif
}



void UART_Transmit(uint8_t data)
{
	uint32_t* DR = (uint32_t*)(UART1_BASE_ADDR + 0x04);
	uint32_t* SR = (uint32_t*)(UART1_BASE_ADDR + 0x00);
	while(((*SR >> 7) & 1) == 0);
	*DR = data;
	while(((*SR >> 6) & 1) == 0);
}

void UART_Print_Log(char* msg)
{
	int msg_len = strlen(msg);
	for(int i = 0; i < msg_len; i++)
	{
		UART_Transmit((uint8_t)msg[i]);
	}
}

char UART_Receive()
{
	uint32_t* DR = (uint32_t*)(UART1_BASE_ADDR + 0x04);
	uint32_t* SR = (uint32_t*)(UART1_BASE_ADDR + 0x00);
	while(((*SR >> 5) & 1) == 0);
	char data = *DR;
	return data;
}

char storeData[100];
int idx;

void USART1_IRQHandler(){
	storeData[idx++] = UART_Receive();
	if(strstr(storeData, "\n"))
	{
		if(strstr(storeData, "blue led on"))
		{
			LedCtrl(LED_BLUE, 1);
			UART_Print_Log("--> ON LED OK\n");
		}
		else if(strstr(storeData, "blue led off"))
		{
			LedCtrl(LED_BLUE, 0);
			UART_Print_Log("--> OFF LED OK\n");
		}
		else if(strstr(storeData, "red led on"))
		{
			LedCtrl(LED_RED, 1);
			UART_Print_Log("--> OFF LED OK\n");
		}
		else if(strstr(storeData, "red led off"))
		{
			LedCtrl(LED_RED, 0);
			UART_Print_Log("--> OFF LED OK\n");
		}
		else
		{
			UART_Print_Log("--> COMMAND NOT FOUND\n");
		}

		memset(storeData, 0,  sizeof(storeData));
		idx = 0;
	}
}

__attribute__((section(".RamFunc"))) void Flash_Erase_Sector(int sec_num)
{
	uint32_t* FLASH_SR = (uint32_t*)(0x40023C00 + 0x0C);
	uint32_t* FLASH_CR = (uint32_t*)(0x40023C00 + 0x10);
	if (((*FLASH_SR >> 16) & 1) == 0) {
		if (((*FLASH_CR >> 31) & 1) == 1) {
			uint32_t* FLASH_KEYR = (uint32_t*)(0x40023C00 + 0x04);
			*FLASH_KEYR = 0x45670123;
			*FLASH_KEYR = 0xCDEF89AB;
		}
		*FLASH_CR |= (0b01 << 1);
		*FLASH_CR |= (sec_num << 3);
		*FLASH_CR |= (0b01 << 16);
		while(((*FLASH_SR >> 16) & 1) == 1);
	}
}

__attribute__((section(".RamFunc"))) void Flash_Program(uint8_t* addr, uint8_t* buf, int size)
{
	uint32_t* FLASH_SR = (uint32_t*)(0x40023C00 + 0x0C);
	uint32_t* FLASH_CR = (uint32_t*)(0x40023C00 + 0x10);
	if (((*FLASH_SR >> 16) & 1) == 0) {
		if (((*FLASH_CR >> 31) & 1) == 1) {
			uint32_t* FLASH_KEYR = (uint32_t*)(0x40023C00 + 0x04);
			*FLASH_KEYR = 0x45670123;
			*FLASH_KEYR = 0xCDEF89AB;
		}
		*FLASH_CR |= (0b01 << 0);
		*FLASH_CR |= (0b00 << 9);
		for(int i = 0; i < size; i++)
		{
			addr[i] = buf[i];
		}

		while(((*FLASH_SR >> 16) & 1) == 1);
	}
}

__attribute__((section(".RamFunc"))) void update()
{
	Flash_Erase_Sector(0);
	Flash_Program(0x08000000, rx_buf, sizeof(rx_buf));
	uint32_t* AIRCR = (uint32_t*)0xE000ED0C;
	*AIRCR |= (0x5FA << 16) | (1 << 2);
}
#define I2C_BASE_ADDR 0x40005400
void I2C_Init()
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	uint32_t* GPIOB_MODER = (uint32_t*)(GPIOB_BASE_ADDR);
	*GPIOB_MODER &= ~(0b11 << 12);
	*GPIOB_MODER |= (0b10 << 12);
	*GPIOB_MODER &= ~(0b11 << 18);
	*GPIOB_MODER |= (0b10 << 18);
	uint32_t* GPIOB_AFRL = (uint32_t*)(GPIOB_BASE_ADDR + 0x20);
	*GPIOB_AFRL &= ~(0b1111 << 24);
	*GPIOB_AFRL |= (0b0100 << 24);
	uint32_t* GPIOB_AFRH = (uint32_t*)(GPIOB_BASE_ADDR + 0x24);
	*GPIOB_AFRH &= ~(0b1111 << 4);
	*GPIOB_AFRH |= (0b0100 << 4);
	__HAL_RCC_I2C1_CLK_ENABLE();
	uint32_t* I2C_CR2 = (uint32_t*)(I2C_BASE_ADDR + 0x04);
	*I2C_CR2 &= ~(0b11111 << 0);
	*I2C_CR2 |= (16 << 0);
	uint32_t* I2C_CCR = (uint32_t*)(I2C_BASE_ADDR + 0x1C);
	*I2C_CCR &= (0xFFF << 0);
	*I2C_CCR |= (160 << 0);
	uint32_t* I2C_CR1 = (uint32_t*)(I2C_BASE_ADDR);
	*I2C_CR1 |= (1 << 0);
}

void I2C_Write_Data(uint8_t slave_reg_addr, uint8_t slave_reg_val)
{
	//- Generate start bit
	uint32_t* I2C_CR1 = (uint32_t*)(I2C_BASE_ADDR);
	*I2C_CR1 |= (1 << 8);
	// wait SB in SR1
	uint32_t* I2C_SR1 = (uint32_t*)(I2C_BASE_ADDR + 0x14);
	while(((*I2C_SR1 >> 0) &1)== 0);
	//- Send 7 slave add + 1 bit write (0b0011001 << 1 | 0)
	uint32_t* I2C_DATA = (uint32_t*)(I2C_BASE_ADDR + 0x10);
	*I2C_DATA = 0b00110010;
	while (((*I2C_SR1 >> 1) &1) == 0);
	uint32_t* I2C_SR2 = (uint32_t*)(I2C_BASE_ADDR + 0x18);
	uint32_t temp = *I2C_SR2;
	//- Check ACK
	while (((*I2C_SR1 >> 10) &1) == 1);
	//- Send command frame.		 (0x1F)
	*I2C_DATA = slave_reg_addr;
	while(((*I2C_SR1 >> 2) &1) == 0);
	//- check ACK
	while (((*I2C_SR1 >> 10) &1) == 1);
	//- Send write data		 (0b11000000)
	*I2C_DATA = 0b11000000;
	while(((*I2C_SR1 >> 2) &1) == 0);
	//- Generate stop bit
	*I2C_CR1 |= (1 << 9);
}

uint8_t I2C_Read_Data(uint8_t slave_reg_addr)
{
//	- Generate start bit
	uint32_t* I2C_CR1 = (uint32_t*)(I2C_BASE_ADDR);
	*I2C_CR1 |= (1 << 8);
	uint32_t* I2C_SR1 = (uint32_t*)(I2C_BASE_ADDR + 0x14);
	while(((*I2C_SR1 >> 0) &1)== 0);
//	- Send 7 slave add + 1 bit write
	uint32_t* I2C_DATA = (uint32_t*)(I2C_BASE_ADDR + 0x10);
	*I2C_DATA = 0b00110010;
	while (((*I2C_SR1 >> 1) &1) == 0);
	uint32_t* I2C_SR2 = (uint32_t*)(I2C_BASE_ADDR + 0x18);
	uint32_t temp = *I2C_SR2;
//	- Check ACK
	while (((*I2C_SR1 >> 10) &1) == 1);
//	- Send command frame.
	*I2C_DATA = slave_reg_addr;
	while(((*I2C_SR1 >> 2) &1) == 0);
//	- Check ACK
	while (((*I2C_SR1 >> 10) &1) == 1);
//	- Generate start bit
	*I2C_CR1 |= (1 << 8);
	while(((*I2C_SR1 >> 0) &1)== 0);
//	- Send 7 slave add + 1 bit read
	*I2C_DATA = 0b00110011;
	while (((*I2C_SR1 >> 1) &1) == 0);
	temp = *I2C_SR2;
//	- Check ACK
	while (((*I2C_SR1 >> 10) &1) == 1);
//	- Read data
	while(((*I2C_SR1 >> 6) &1) ==0);
	uint8_t data = *I2C_DATA;
//	- Generate stop bit
	*I2C_CR1 |= (1 << 9);
	return data;
}

#define GPIOE_BASE_ADDR		0x40021000
#define SPI1_BASE_ADDR		0x40013000
void spi_init()
{
	// map pin, PA5 - SPI1_SCK, PA6 - SPI1_MISO, PA7 - SPI1_MOSI
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
	*GPIOA_MODER &= ~(0b111111 << 10);
	*GPIOA_MODER |= (0b10 << 10) | (0b10 << 12) | (0b10 << 14);

	uint32_t* GPIOA_AFRL = (uint32_t*)(GPIOA_BASE_ADDR + 0x20);
	*GPIOA_AFRL &= ~(0xfff << 20);
	*GPIOA_AFRL |= (5 << 20) | (5 << 24) | (5 << 28);
	// PE3 set as GPIO_OUTPUT
	__HAL_RCC_GPIOE_CLK_ENABLE();
	uint32_t* GPIOE_MODER = (uint32_t*)(GPIOE_BASE_ADDR + 0x00);
	*GPIOE_MODER &= ~(0b11 << 6);
	*GPIOE_MODER |= (0b01 << 6);

	__HAL_RCC_SPI1_CLK_ENABLE();	// 16Mhz
	// stm32 as master, speed of SCLK = 1Mhz, use software slave management, enable SPI
	uint16_t* SPI1_CR1 = (uint16_t*)(SPI1_BASE_ADDR + 0x00);
	*SPI1_CR1 |= (1 << 2);
	*SPI1_CR1 |= (0b011 << 3);
	*SPI1_CR1 |= (1 << 9) | (1 << 8);
	*SPI1_CR1 |= (1 << 6);
}

char spi_read(char reg_addr)
{
	uint32_t* GPIOE_ODR = (uint32_t*)(GPIOE_BASE_ADDR + 0x14);
	*GPIOE_ODR &= ~(1<<3);	// set PE3 to LOW to active slave

	uint16_t* SPI1_DR = (uint16_t*)(SPI1_BASE_ADDR + 0x0C);
	uint16_t* SPI1_SR = (uint16_t*)(SPI1_BASE_ADDR + 0x08);

	while(((*SPI1_SR >> 7)&1)==1);
	*SPI1_DR = reg_addr | (1 << 7);
	while(((*SPI1_SR >> 1)&1)==0);
	while(((*SPI1_SR >> 7)&1)==1);

	while(((*SPI1_SR >> 0)&1)==0);
	char data = *SPI1_DR;


	while(((*SPI1_SR >> 7)&1)==1);
	*SPI1_DR = 0xff;
	while(((*SPI1_SR >> 1)&1)==0);
	while(((*SPI1_SR >> 7)&1)==1);

	while(((*SPI1_SR >> 0)&1)==0);
	data = *SPI1_DR;

	*GPIOE_ODR |= (1<<3);	// set PE3 to HIGH to inactive slave

	return data;

}

void spi_write(char reg_addr, char reg_val)
{
	uint32_t* GPIOE_ODR = (uint32_t*)(GPIOE_BASE_ADDR + 0x14);
	*GPIOE_ODR &= ~(1<<3);	// set PE3 to LOW to active slave
	uint16_t* SPI1_DR = (uint16_t*)(SPI1_BASE_ADDR + 0x0C);
	uint16_t* SPI1_SR = (uint16_t*)(SPI1_BASE_ADDR + 0x08);

	while(((*SPI1_SR >> 7)&1)==1);
	*SPI1_DR = reg_addr;
	while(((*SPI1_SR >> 1)&1)==0);
	while(((*SPI1_SR >> 7)&1)==1);

	while(((*SPI1_SR >> 0)&1)==0);
	char data = *SPI1_DR;


	while(((*SPI1_SR >> 7)&1)==1);
	*SPI1_DR = reg_val;
	while(((*SPI1_SR >> 1)&1)==0);
	while(((*SPI1_SR >> 7)&1)==1);

	while(((*SPI1_SR >> 0)&1)==0);
	data = *SPI1_DR;

	*GPIOE_ODR |= (1<<3);	// set PE3 to HIGH to inactive slave
}

#define TIM1_BASE_ADDR	0x40010000
void time_init()
{
	__HAL_RCC_TIM1_CLK_ENABLE();
	// 1s, PSC = 16000, ARR = 1000
	uint16_t* TIM1_PSC = (uint16_t*)(TIM1_BASE_ADDR + 0x28);
	uint16_t* TIM1_ARR = (uint16_t*)(TIM1_BASE_ADDR + 0x2C);
	uint16_t* TIM1_CR1 = (uint16_t*)(TIM1_BASE_ADDR + 0x00);


	*TIM1_PSC = 16 - 1;
	*TIM1_ARR = 1000;

	uint16_t* TIM1_DIER = (uint16_t*)(TIM1_BASE_ADDR + 0x0C);
	*TIM1_DIER |= (1<<0);

	uint32_t* ISER0 = (uint32_t*)(0xe000e100);
	*ISER0 |= (1<<25);

	*TIM1_CR1 |= (1 << 0);	// count enable
}

int time_cnt = 10;
void TIM1_UP_TIM10_IRQHandler()
{
	time_cnt++;
	uint16_t* TIM1_SR = (uint16_t*)(TIM1_BASE_ADDR + 0x10);
	*TIM1_SR &= ~(1<<0);
}
#define ADC1_BASE_ADDR	0x40012000
void Temp_Sensor_Init()
{
	__HAL_RCC_ADC1_CLK_ENABLE();
	uint32_t* ADC_JSQR = (uint32_t*)(ADC1_BASE_ADDR + 0x38);
	*ADC_JSQR = 16 << 15;
	uint32_t* ADC_CCR = (uint32_t*)(ADC1_BASE_ADDR + 0x300 + 0x04);
	*ADC_CCR |= (1 << 23);
	uint32_t* ADC_SMPR1 = (uint32_t*)(ADC1_BASE_ADDR + 0x0C);
	*ADC_SMPR1 |= (0b111 << 18);
	uint32_t* ADC_CR2 = (uint32_t*)(ADC1_BASE_ADDR + 0x08);
	*ADC_CR2 |= (1 << 0);
}

float Temp_Sensor_Read()
{
	uint32_t* ADC_CR2 = (uint32_t*)(ADC1_BASE_ADDR + 0x08);
	*ADC_CR2 |= (1 << 22);
	uint32_t* ADC_SR = (uint32_t*)(ADC1_BASE_ADDR + 0x00);
	while(((*ADC_SR >> 2)&1)==0);
	*ADC_SR &= ~(1<<2);
	uint32_t* ADC_JDR1 = (uint32_t*)(ADC1_BASE_ADDR + 0x3C);
	uint16_t adc_raw = *ADC_JDR1;
	float vin =  (adc_raw * 3.0)/4095;
	float temperature = ((vin - 0.76) / 0.0025) + 25;
	return temperature;

}
void delay(int time)
{
	time_cnt = 0;
	while(time_cnt < time);
}
float temp;
int main()
{
	LedInit();
	ButtonInit();
	EXTI0Init();
	UART_Init();
	Temp_Sensor_Init();
	time_init();
	while(1)
	{
		LedCtrl(LED_RED, 1);
		delay(1000);
		LedCtrl(LED_RED, 0);
		delay(1000);
		Temp_Sensor_Read();
	}
	return 0;
}
