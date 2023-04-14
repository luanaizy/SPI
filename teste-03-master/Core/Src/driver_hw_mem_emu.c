/*
 * driver_hw_mem_emu.c
 *
 *  Created on: 13 de abr de 2023
 *      Author: luana
 */

#include "driver_mem_emu.h"

#include "stm32f4xx_hal.h"

/*
 * função de escrita em um pino gpio do hw
 * recebe: ponteiro para a referida memória, valor a ser escrito no pino (gpio_pin_high / gpio_pin_low)
 */

void hw_gpio_write_pin(mem_emu_hw_t *mem, uint8_t value){
	uint8_t pin_value;

	if(value == gpio_pin_high){
		pin_value = GPIO_PIN_SET;
	}else{
		pin_value = GPIO_PIN_RESET;
	}

	HAL_GPIO_WritePin((GPIO_TypeDef *)mem->gpio_port, mem->gpio_pin, pin_value);

}

/*
 * função bloqueante de transmissão spi
 * recebe: ponteiro para a conexão spi, ponteiro para dado a ser transmitido, tamanho do dado em bytes, timeout em ms
 * retorna: status
 */

status_hw hw_spi_transmit(void *hspi, uint8_t*pdata, uint16_t size, uint32_t timeout){
	int errorcode;
	errorcode = HAL_SPI_Transmit((SPI_HandleTypeDef *)hspi, pdata, size, timeout);
	if (errorcode == HAL_TIMEOUT){
			return error_timeout;
		}
	return error_ok;
}

/*
 * função bloqueante de recepção spi
 * recebe: ponteiro para a conexão spi, ponteiro para alocação do dado, tamanho do dado em bytes, timeout em ms
 * retorna: status
 */

status_hw hw_spi_receive(void*hspi, uint8_t *pdata, uint16_t size, uint32_t timeout){
	int errorcode;
	errorcode = HAL_SPI_Receive((SPI_HandleTypeDef *)hspi, pdata, size, timeout);
	if (errorcode == HAL_TIMEOUT){
				return error_timeout;
			}
		return error_ok;
}
