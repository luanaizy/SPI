/*
 * functions.c
 *
 *  Created on: 11 de abr de 2023
 *      Author: luana
 */
#include <driver_mem_emu.h>
/*
 * Comandos suportados pela memoria mem_emu
 * */
#define MEM_EMU_CMD_WRITE (0x02)
#define MEM_EMU_CMD_READ  (0x03)

/* Timeout das funções de leitura e escrita SPI em ms*/
#define MEM_EMU_TIMEOUT   (1000)

/*
 * função de inicialização da memória
 * recebe: ponteiro para a referida memória, conexão spi, porta do gpio e pino do gpio
 */
void mem_emu_init(mem_emu_hw_t *mem, void *hspi, void *gpio_port, uint16_t gpio_pin){
	mem->hspi = hspi;
	mem->gpio_port = gpio_port;
	mem->gpio_pin = gpio_pin;
}
/*
 * função de leitura de um byte na memória emulada
 * recebe: ponteiro para a referida memória, endereço de leitura, ponteiro para alocação do byte
 * */

status_hw mem_emu_read_byte(mem_emu_hw_t *mem, uint8_t addr, uint8_t *pdata){

	int errorcode;

	hw_gpio_write_pin(mem,  gpio_pin_low);
	uint8_t cmd_read[]= {MEM_EMU_CMD_READ, addr};
	errorcode = hw_spi_transmit(mem->hspi, cmd_read, 2, MEM_EMU_TIMEOUT);
	if(errorcode == error_timeout){
		return errorcode;
	}
	errorcode = hw_spi_receive(mem->hspi, pdata , 1, MEM_EMU_TIMEOUT);
	if(errorcode == error_timeout){
			return errorcode;
	}
	hw_gpio_write_pin(mem, gpio_pin_high);

	return error_ok ;
}

/*
 * função de escrita de um byte na memória emulada
 * recebe: ponteiro para a referida memória, endereço de escrita, byte a ser escrito
 * */

status_hw mem_emu_write_byte(mem_emu_hw_t *mem, uint8_t addr, uint8_t byte){
	int errorcode;

	hw_gpio_write_pin(mem,  gpio_pin_low);
	uint8_t cmd_write[]= {MEM_EMU_CMD_WRITE, addr, byte};
	errorcode = hw_spi_transmit(mem->hspi, cmd_write, 3, MEM_EMU_TIMEOUT);
	if (errorcode == error_timeout){
			return errorcode;
		}
	hw_gpio_write_pin(mem,  gpio_pin_high);

	return error_ok;
}


