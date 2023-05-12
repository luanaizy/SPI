/**
 * @file    : driver_mem_emu.c
 * @brief   : Driver de funcoes para a memoria emulada.
 *
 */
#include <driver_mem_emu.h>


/*defines-----------------------------------------------------------------------------------------*/
#define MEM_EMU_CMD_WRITE (0x02)/* @brief comando de escrita na memória emulada */
#define MEM_EMU_CMD_READ  (0x03)/* @brief comando de leitura na memória emulada*/

#define MEM_EMU_TIMEOUT   (1000)/* @brief Timeout das funções de leitura e escrita SPI em ms*/


/**
  * @brief função de inicialização da memória
  * @param mem ponteiro para estrutura de memória emulada
  * @param hspi ponteiro para a conexão spi da memória
  * @param gpio_port ponteiro para a porta do gpio
  * @param gpio_pin pino do gpio
  * @param gpio_write ponteiro para função de escrita no gpio referente a mem
  * @param spi_mem_write ponteiro para função de escrita spi referente a mem
  * @param spi_mem_read ponteiro para função de leitura spi referente a mem
  * @retval status_hw
  */
void mem_emu_init(mem_emu_hw_t *mem, void *hspi, void *gpio_port, uint16_t gpio_pin, gpio_write_cb gpio_write, spi_write_cb spi_mem_write, spi_read_cb spi_mem_read){
	mem->hspi = hspi;
	mem->gpio_port = gpio_port;
	mem->gpio_pin = gpio_pin;
	mem->gpio_write = gpio_write;
	mem->spi_mem_write = spi_mem_write;
	mem->spi_mem_read = spi_mem_read;
}


/**
  * @brief função de leitura de um byte na memória emulada
  * @param mem ponteiro para estrutura de memória emulada
  * @param addr endereço da memória a ser lido
  * @param pdata ponteiro para alocação do byte
  * @retval status_hw
  */

status_hw mem_emu_read_byte(mem_emu_hw_t *mem, uint8_t addr, uint8_t *pdata){

	int errorcode;

	hw_gpio_write_pin(mem->gpio_port, mem->gpio_pin,  gpio_pin_low);
	uint8_t cmd_read[]= {MEM_EMU_CMD_READ, addr};
	errorcode = hw_spi_transmit(mem->hspi, cmd_read, 2, MEM_EMU_TIMEOUT);
	if(errorcode == error_timeout){
		return errorcode;
	}
	errorcode = hw_spi_receive(mem->hspi, pdata , 1, MEM_EMU_TIMEOUT);
	if(errorcode == error_timeout){
			return errorcode;
	}
	hw_gpio_write_pin(mem->gpio_port, mem->gpio_pin, gpio_pin_high);

	return error_ok ;
}


/**
  * @brief função de escrita de um byte na memória emulada
  * @param mem ponteiro para estrutura de memória emulada
  * @param addr endereço da memória no qual será escrito o byte
  * @param byte byte a ser escrito
  * @retval status_hw
  */
status_hw mem_emu_write_byte(mem_emu_hw_t *mem, uint8_t addr, uint8_t byte){
	int errorcode;

	hw_gpio_write_pin(mem->gpio_port, mem->gpio_pin, gpio_pin_low);
	uint8_t cmd_write[]= {MEM_EMU_CMD_WRITE, addr, byte};
	errorcode = hw_spi_transmit(mem->hspi, cmd_write, 3, MEM_EMU_TIMEOUT);
	if (errorcode == error_timeout){
			return errorcode;
		}
	hw_gpio_write_pin(mem->gpio_port, mem->gpio_pin, gpio_pin_high);

	return error_ok;
}


