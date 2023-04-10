/*
 * mem_emu.c
 *
 *  Created on: Mar 27, 2023
 *      Author: luana
 */

#include "stm32f1xx_hal.h"
#include "mem_emu.h"

void mem_emu_init(mem_emu_hw_t *mem, SPI_HandleTypeDef *hspi, GPIO_TypeDef  *gpio_port, uint16_t gpio_pin){
	mem->hspi = hspi;
	mem->gpio_port = gpio_port;
	mem->gpio_pin = gpio_pin;
}


uint8_t mem_emu_read_byte(mem_emu_hw_t *mem, uint8_t addr){
	
	HAL_GPIO_WritePin(mem->gpio_port, mem->gpio_pin, GPIO_PIN_RESET);	
	uint8_t cmd_read[]= {0x03, addr};	
	HAL_SPI_Transmit(mem->hspi, cmd_read, 2, 1000);
	HAL_SPI_Receive(mem->hspi, mem, 1, HAL_MAX_DELAY);
}

void mem_emu_write_byte(mem_emu_hw_t *mem, uint8_t addr, uint8_t data){
	
	HAL_GPIO_WritePin(mem->gpio_port, mem->gpio_pin, GPIO_PIN_RESET);	
	uint8_t cmd_write[]= {0x02, addr, data};	
	HAL_SPI_Transmit(mem->hspi, cmd_write, 3, 1000);
}




