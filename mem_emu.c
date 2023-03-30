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


uint8_t mem_emu_read_byte(mem_emu_hw_t *mem, uint8_t addr){
	uint8_t rxdata[]= {0x03, addr};
	
	HAL_SPI_Transmit(&(mem->hspi), rxdata, 2, 1000);
	HAL_SPI_Receive(&(mem->hspi), mem, 1, HAL_MAX_DELAY);
}




