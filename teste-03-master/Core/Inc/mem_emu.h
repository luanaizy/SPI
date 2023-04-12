/*
 * mem_emu.h
 *
 *  Created on: 11 de abr de 2023
 *      Author: luana
 */

#ifndef INC_MEM_EMU_H_
#define INC_MEM_EMU_H_


#include "stm32f4xx_hal.h"

typedef struct{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef  *gpio_port;
	uint16_t gpio_pin;
}mem_emu_hw_t;

void mem_emu_init(mem_emu_hw_t *mem, SPI_HandleTypeDef *hspi, GPIO_TypeDef  *gpio_port, uint16_t gpio_pin);
uint8_t mem_emu_read_byte(mem_emu_hw_t *mem, uint8_t addr);
void mem_emu_write_byte(mem_emu_hw_t *mem, uint8_t addr, uint8_t data);

#endif /* INC_MEM_EMU_H_ */
