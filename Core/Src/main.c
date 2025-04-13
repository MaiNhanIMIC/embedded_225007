/*
 * main.c
 *
 *  Created on: Mar 23, 2025
 *      Author: Dell
 */
#include "main.h"
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
	uint8_t* dis = 0x20000000;
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
	ptr= 0x20000058;
	*ptr = function;


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



int main()
{
	HAL_Init();
	LedInit();
	ButtonInit();
	EXTI0Init();

	while(1)
	{
		LedCtrl(LED_GREEN, 1);
		HAL_Delay(cnt);
		LedCtrl(LED_GREEN, 0);
		HAL_Delay(1000);
	}
	return 0;
}
