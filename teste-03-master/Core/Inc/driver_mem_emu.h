/*
 * @file     :mem_emu.h
 * @brief    :biblioteca de funcoes para uso de memoria emulada
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_DRIVER_MEM_EMU_H_
#define INC_DRIVER_MEM_EMU_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


/* typedef -----------------------------------------------------------*/
/**
 * @brief Ponteiros para funções de escrita e leitura que retornam void
 */
typedef void (*gpio_write_cb)(void*gpio_port, uint16_t gpio_pin, uint32_t value);
typedef void (*spi_write_cb)(void*hspi, uint8_t *pdata, uint16_t size, uint32_t timeout);
typedef void (*spi_read_cb)(void*hspi, uint8_t *pdata, uint16_t size, uint32_t timeout);

/**
 * @brief estrutura memoria emulada
 */
typedef struct{
	void *hspi;
	void *gpio_port;
	uint16_t gpio_pin;
	gpio_write_cb gpio_write;
	spi_write_cb spi_mem_write;
	spi_read_cb spi_mem_read;
}mem_emu_hw_t;




/**
 * @brief possíveis status das funções de leitura/escrita e transmissão/recepção
 */

typedef enum{
	error_ok,
	error_timeout
}status_hw;



/**
 * @brief possíveis estados do pino gpio
 */
typedef enum
{
  gpio_pin_low = 0,
  gpio_pin_high = 1
}gpio_pinstate;




/*Protótipos das funções-------------------------------------------------------------------*/
void hw_gpio_write_pin(void*gpio_port, uint16_t gpio_pin, uint32_t value);

status_hw hw_spi_transmit(void*hspi, uint8_t*pdata, uint16_t size, uint32_t timeout);

status_hw hw_spi_receive(void*hspi, uint8_t *pdata, uint16_t size, uint32_t timeout);

void mem_emu_init(mem_emu_hw_t *mem, void *hspi, void  *gpio_port, uint16_t gpio_pin, gpio_write_cb gpio_write, spi_write_cb spi_mem_write, spi_read_cb spi_mem_read);

status_hw mem_emu_read_byte(mem_emu_hw_t *mem, uint8_t addr, uint8_t *pdata);

status_hw mem_emu_write_byte(mem_emu_hw_t *mem, uint8_t addr, uint8_t data);




#endif /* INC_DRIVER_MEM_EMU_H_ */
