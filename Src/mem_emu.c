/*
 * mem_emu.c
 *
 *  Created on: Mar 27, 2023
 *      Author: luana
 */
void mem_emu_init(mem_emu_hw_t *mem, SPI_HandleTypeDef *hspi, GPIO_TypeDef  *gpio_port, uint16_t gpio_pin){
	mem->hspi = hspi;
	mem->gpio_port = gpio_port;
	mem->gpio_pin = gpio_pin;
}


