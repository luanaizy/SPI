/*
 * @file     :mem_emu.h
 * @brief    :biblioteca de funcoes para uso de memoria emulada
 */

#ifndef INC_DRIVER_MEM_EMU_H_
#define INC_DRIVER_MEM_EMU_H_


#include "stm32f4xx_hal.h"

/*
 *struct de memória emulada e definição das suas características
 * */

typedef void (*gpio_write_cb)(void*gpio_port, uint16_t gpio_pin, uint32_t value);
typedef void (*spi_write_cb)(void*hspi, uint8_t *pdata, uint16_t size, uint32_t timeout);
typedef void (*spi_read_cb)(void*hspi, uint8_t *pdata, uint16_t size, uint32_t timeout);

typedef struct{
	void *hspi;
	void *gpio_port;
	uint16_t gpio_pin;
	gpio_write_cb gpio_write;
	spi_write_cb spi_mem_write;
	spi_read_cb spi_mem_read;
}mem_emu_hw_t;




/*
 *possíveis status das funções de leitura/escrita e transmissão/recepção
 * */
typedef enum{
	error_ok,
	error_timeout
}status_hw;



/*
 *possíveis estados do pino gpio
 * */
typedef enum
{
  gpio_pin_low = 0,
  gpio_pin_high = 1
}gpio_pinstate;




/*
 * função de escrita em um pino gpio do hw
 * recebe: ponteiro para a referida memória, valor a ser escrito no pino (gpio_pin_high / gpio_pin_low)
 */
void hw_gpio_write_pin(void*gpio_port, uint16_t gpio_pin, uint32_t value);



/*
 * função bloqueante de transmissão spi
 * recebe: ponteiro para a conexão spi, ponteiro para dado a ser transmitido, tamanho do dado em bytes, timeout em ms
 * retorna: status
 */
status_hw hw_spi_transmit(void*hspi, uint8_t*pdata, uint16_t size, uint32_t timeout);




/*
 * função bloqueante de recepção spi
 * recebe: ponteiro para a conexão spi, ponteiro para alocação do dado, tamanho do dado em bytes, timeout em ms
 * retorna: status
 */
status_hw hw_spi_receive(void*hspi, uint8_t *pdata, uint16_t size, uint32_t timeout);




/*
 * função de inicialização da memória
 * recebe: ponteiro para a referida memória, conexão spi, porta do gpio e pino do gpio
 */
void mem_emu_init(mem_emu_hw_t *mem, void *hspi, void  *gpio_port, uint16_t gpio_pin, gpio_write_cb gpio_write, spi_write_cb spi_mem_write, spi_read_cb spi_mem_read);



/*
 * função de leitura de um byte na memória emulada
 * recebe: ponteiro para a referida memória, endereço de leitura, ponteiro para alocação do byte
 * retorna: status
 * */
status_hw mem_emu_read_byte(mem_emu_hw_t *mem, uint8_t addr, uint8_t *pdata);


/*
 * função de escrita de um byte na memória emulada
 * recebe: ponteiro para a referida memória, endereço de escrita, byte a ser escrito
 * retorna: status
 * */
status_hw mem_emu_write_byte(mem_emu_hw_t *mem, uint8_t addr, uint8_t data);




#endif /* INC_DRIVER_MEM_EMU_H_ */
