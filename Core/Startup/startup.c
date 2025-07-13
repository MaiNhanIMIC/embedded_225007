/*
 * startup.c
 *
 *  Created on: Jul 13, 2025
 *      Author: Dell
 */

int main();
void TIM1_UP_TIM10_IRQHandler();

typedef void(*handler_t)();




extern int _estack;

extern int _sbss;
extern int _ebss;

extern int _sidata;
extern int _sdata;
extern int _edata;

void Reset_Handler()
{
	// clear bien toan chua khoi tao gia tri ban dau - bss
	int* bss_start = &_sbss;
	int* bss_end = &_ebss;
	while(bss_start < bss_end)
	{
		*(bss_start) = 0;
		bss_start++;
	}

	// copy du lieu o sidata len sdata - de khoi tao gia tri ban dau cho data
	int* data_start = &_sdata;
	int* data_end   = &_edata;
	int* idata_start= &_sidata;
	while(data_start < data_end)
	{
		*data_start = *idata_start;
		data_start++;
		idata_start++;
	}

	main();
}

__attribute__((section(".vung_nay_de_luu_vttb"))) handler_t VTTB[] = {
		(handler_t)&_estack,
		Reset_Handler,
		0,
		0,
		0,
		0,

		0,
		0,
		0,
		0,

		0,
		0,
		0,
		0,

		0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		TIM1_UP_TIM10_IRQHandler,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
};
