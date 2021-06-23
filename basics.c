/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2018 Karl Palsson <karlp@tweak.net.au>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>


/* 
----------------------------
VCP PINS to ST-LINK

UART2   TX PA2  (AF7)
        RX PA15 (AF3)
----------------------------

UART1   RX PB7 (AF7) D4
		TX PB6 (AF7) D5

LED    PB3

I2C1   SCL PA9  (AF4) D1
       SDA PA10 (AF4) D0

SPI1   MOSI PA7 (AF5) A6
	   MISO PA6 (AF5) A5
	   SCLK PA5 (AF5) A4
	   SSEL PB0 (AF5) D3
*/

int _write(int file, char *ptr, int len);


/*
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
*/
static void clock_setup(void)
{
	rcc_osc_on(RCC_MSI);
	rcc_wait_for_osc_ready(RCC_MSI);
	rcc_set_msi_range(RCC_CR_MSIRANGE_4MHZ);

	flash_prefetch_enable();
	flash_set_ws(4);
	flash_dcache_enable();
	flash_icache_enable();

	// // Config the PLL for 80MHz off the MSI_OSC
	// rcc_set_main_pll(RCC_PLLCFGR_PLLSRC_MSI, 1, 40,
	// 				 7, 4, 2);
	// rcc_osc_on(RCC_PLL);
	// rcc_wait_for_osc_ready(RCC_PLL);

	rcc_set_sysclk_source(RCC_CFGR_SW_MSI); 
	rcc_wait_for_sysclk_status(RCC_MSI);

	/*
	// Set to use PLL with 80MHz
	rcc_set_sysclk_source(RCC_CFGR_SW_PLL); 
	rcc_wait_for_sysclk_status(RCC_PLL);
	*/

	/* FIXME - eventually handled internally */
	rcc_ahb_frequency =  4e6; //80e6;
	rcc_apb1_frequency = 4e6; //80e6;
	rcc_apb2_frequency = 4e6; //80e6;

	/* Enable clocks for the ports we need */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Enable clocks for peripherals we need */
	rcc_periph_clock_enable(RCC_USART2);
}

static void usart_setup(void)
{
	/* Setup GPIO pins for USART2 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2|GPIO15);

	/* Setup USART2 TX/RX pins as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
	gpio_set_af(GPIOA, GPIO_AF3, GPIO15);

	usart_set_baudrate(USART2, 9600);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

/**
 * Use USART2 as a console.
 * This is a syscall for newlib
 * @param file
 * @param ptr
 * @param len
 * @return
 */
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART2, '\r');
			}
			usart_send_blocking(USART2, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}


int main(void)
{
	int j = 0;
	clock_setup();
	usart_setup();
	printf("hi guys!\n");

	/* green led for ticking */
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);
	
	while (1) 
	{
		gpio_toggle(GPIOB, GPIO3);

		for (int i = 0; i < 4000000; i++)
		{ 
			__asm__("NOP");
		}
	}

	return 0;
}
