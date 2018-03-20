/*****************************************************************************
* Auteur : Maxime Turenne
* Copyright : Maxime Turenne
* Description: Demo d'utilisation de FreeRTOS dans le cadre d'un thermostate numérique.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Environment header files. */
#include "power_clocks_lib.h"

#include "print_funcs.h"
#include "board.h"
#include "compiler.h"
#include "dip204.h"
#include "intc.h"
#include "gpio.h"
#include "pm.h"
#include "delay.h"
#include "spi.h"
#include "usart.h"
#include "conf_clock.h"
#include "adc.h"

/* Scheduler header files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Config USART
#define USART_BAUDRATE              56700
#define ACQ_START_CHAR              0x73         // 0x73 = 's'
#define ACQ_STOP_CHAR               0x78         // 0x78 = 'x'
#define USART_TX_VAL_MASK           0xFE         // Les 7 premiers bits
#define USART_TX_VAL_LIGHT_ID       1
#define USART_TX_VAL_POT_ID         0
#define USART_WAIT_TXRDY \
while(!(AVR32_USART1.csr & (AVR32_USART_CSR_TXRDY_MASK)))
#define USART_TX_SET_VAL(val,id) \
AVR32_USART1.thr = ((val >> 2 & USART_TX_VAL_MASK) | id) & AVR32_USART_THR_TXCHR_MASK; AVR32_USART1.ier = AVR32_USART_IER_TXRDY_MASK

// message struct
typedef struct ACQData {
	U16 light_val;
	U16 pot_val;
} ACQData;// tasks

static void LED_Flash(void *pvParameters);
static void UART_Cmd_RX(void *pvParameters);
static void UART_SendSample(void *pvParameters);
static void ADC_Cmd(void *pvParameters);
static void AlarmMsgQ(void *pvParameters);

// initialization functions
void init_lcd(void);
void init_usart(void);

// global var
volatile bool is_in_acq = false;
volatile int idle_tick_count = 0;
volatile portTickType last_idle_tick = 0;
volatile short computed_sample_rate;

// semaphore
static xSemaphoreHandle sem_acq_status = NULL; //TODO

int main(void) {

	// Configure Osc0 in crystal mode (i.e. use of an external crystal source, with
	// frequency FOSC0) with an appropriate startup time then switch the main clock
	// source to Osc0.
	pcl_switch_to_osc(PCL_OSC0, FOSC0, OSC0_STARTUP);

	/* Setup the LED's all off */
	LED_Display(0);

	/* init */
	init_lcd();
	init_usart();

	sem_acq_status = xSemaphoreCreateCounting(1,1);

	/* tasks. */
	xTaskCreate(
	LED_Flash
	, (const signed portCHAR *)"LED and LCD"
	, configMINIMAL_STACK_SIZE*3
	, NULL
	, tskIDLE_PRIORITY + 1
	, NULL );

	xTaskCreate(
	UART_Cmd_RX
	, (const signed portCHAR *)"USART RX"
	, configMINIMAL_STACK_SIZE*3
	, NULL
	, tskIDLE_PRIORITY + 1
	, NULL );
/*
	xTaskCreate(
	UART_SendSample
	, (const signed portCHAR *)"USART TX"
	, configMINIMAL_STACK_SIZE*3
	, NULL
	, tskIDLE_PRIORITY + 1
	, NULL );
*/

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */

	return 0;
}

/**
 * Tasks
**/

static void LED_Flash(void *pvParameters)
{
	bool is_led_high = false;
	short led_cycle_count = 0;
	short lcd_cycle_count = 0;
	
	unsigned short cpu_percent = 0;
	portTickType total_tick_count = 0;
	portTickType last_total_tick_count = 0;
	char cpu_lcd_buffer[20];
	char sample_lcd_buffer[20];
	char dbg_lcd_buffer[20];

	while(1)
	{
		led_cycle_count++;
		lcd_cycle_count++;
		
		if(led_cycle_count == 2)
		{
			// update the led state
			if(is_led_high)
			{
				// Set the led to low
				gpio_set_gpio_pin(LED0_GPIO);
				gpio_set_gpio_pin(LED1_GPIO);
				is_led_high=false;
			}
			else
			{
				// Set the led to high
				gpio_clr_gpio_pin(LED0_GPIO);
				if(is_in_acq)
				{
					gpio_clr_gpio_pin(LED1_GPIO);
				}
				is_led_high=true;
			}
			led_cycle_count = 0;
		}
		
		if(lcd_cycle_count == 5)
		{
			// update the lcd
			total_tick_count = xTaskGetTickCount();
			cpu_percent = (idle_tick_count * 100) / ((total_tick_count - last_total_tick_count) * 100);
			sprintf(sample_lcd_buffer, "Sample: %dHz   ", computed_sample_rate);
			sprintf(cpu_lcd_buffer,    "CPU:    %d%%  ", cpu_percent);

			sprintf(dbg_lcd_buffer, "dbg:(a:%dc i:%dc)", (total_tick_count - last_total_tick_count), idle_tick_count);
			
			dip204_set_cursor_position(1, 1);
			dip204_write_string(sample_lcd_buffer);
			dip204_set_cursor_position(1, 2);
			dip204_write_string(cpu_lcd_buffer);
			dip204_set_cursor_position(1, 4);
			dip204_write_string(dbg_lcd_buffer);
			dip204_set_cursor_position(20, 4);
			
			last_total_tick_count = total_tick_count;
			idle_tick_count = 0;
			lcd_cycle_count = 0;
		}

		vTaskDelay(100);
	}
}

static void UART_Cmd_RX(void *pvParameters)
{
	U32 char_recu = 0;
	
	while(1)
	{
		if (AVR32_USART1.csr & (AVR32_USART_CSR_RXRDY_MASK))
		{
			//Lire le char recu dans registre RHR, et le stocker dans un 32bit
			char_recu = (AVR32_USART1.rhr & AVR32_USART_RHR_RXCHR_MASK);
			
			// On active ou désactive l'acquisition le charactere recus
			if(char_recu == ACQ_STOP_CHAR && is_in_acq)
			{
				is_in_acq=false;
			}
			if(char_recu == ACQ_START_CHAR && !is_in_acq)
			{
				is_in_acq=true;
			}	
		}
		
		vTaskDelay(50);
	}
	
}

static void UART_SendSample(void *pvParameters)
{
	struct ACQData acq_data;
	
	while(1)
	{
		//TODO semaphore
		while(xQueueReceive(&pvParameters, &acq_data, (portTickType)0) == pdTRUE)
		{
			USART_WAIT_TXRDY;
			USART_TX_SET_VAL(acq_data.light_val, USART_TX_VAL_LIGHT_ID);
			USART_WAIT_TXRDY;
			USART_TX_SET_VAL(acq_data.pot_val, USART_TX_VAL_POT_ID);
		}

		vTaskDelay(50);
	}
	
}

static void ADC_Cmd(void *pvParameters) {
	while (1) {
		vTaskDelay(250);
	}
}

static void Alarm_msgQ(void *pvParameters) {
	while (1) {
		vTaskDelay(1000);
	}
}


	
/**
 * Init functions
**/

void vApplicationIdleHook(void) {
	const portTickType current_tick = xTaskGetTickCount();
	if(current_tick != last_idle_tick)
	{
		// increment the tick count if it changed
		idle_tick_count++;
		last_idle_tick = current_tick;
	}
}

/**
 * Interrupts
**/
	
__attribute__((__interrupt__))
static void usart_tx_handler(void) {
	// Clear the register on TX
	AVR32_USART1.idr = AVR32_USART_IDR_TXRDY_MASK;
}
	
/**
 * Init functions
**/

void init_usart(void) {
	static const gpio_map_t USART_GPIO_MAP =
	{
		{AVR32_USART1_RXD_0_0_PIN, AVR32_USART1_RXD_0_0_FUNCTION},
		{AVR32_USART1_TXD_0_0_PIN, AVR32_USART1_TXD_0_0_FUNCTION}
	};
	static const usart_options_t usart_opt =
	{
		.baudrate = USART_BAUDRATE,
		.charlength = 8,
		.paritytype = USART_NO_PARITY,
		.stopbits = USART_1_STOPBIT,
		.channelmode = USART_NORMAL_CHMODE
	};
	
	// Assigner les pins du GPIO a etre utiliser par le USART1.
	gpio_enable_module(USART_GPIO_MAP,sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));
	// Initialise le USART1 en mode seriel RS232
	usart_init_rs232((&AVR32_USART1), &usart_opt, FOSC0);
	// On enregistre le handler
	INTC_register_interrupt(&usart_tx_handler, AVR32_USART1_IRQ, AVR32_INTC_INT0);
}

void init_lcd(void) {
	static const gpio_map_t DIP204_SPI_GPIO_MAP = {
		{ DIP204_SPI_SCK_PIN,	DIP204_SPI_SCK_FUNCTION }, // SPI Clock.
		{ DIP204_SPI_MISO_PIN, DIP204_SPI_MISO_FUNCTION }, // MISO.
		{ DIP204_SPI_MOSI_PIN, DIP204_SPI_MOSI_FUNCTION }, // MOSI.
		{ DIP204_SPI_NPCS_PIN, DIP204_SPI_NPCS_FUNCTION } // Chip Select NPCS.
	};

	// add the spi options driver structure for the LCD DIP204
	spi_options_t spiOptions = {
		.reg = DIP204_SPI_NPCS,
		.baudrate = 1000000,
		.bits = 8,
		.spck_delay = 0,
		.trans_delay = 8, // <---- Very importent with the new compilor in atmel 6.x
		.stay_act = 1,
		.spi_mode = 0,
		.modfdis = 1
	};

	// Assign I/Os to SPI
	gpio_enable_module(DIP204_SPI_GPIO_MAP, sizeof(DIP204_SPI_GPIO_MAP)
	/ sizeof(DIP204_SPI_GPIO_MAP[0]));

	// Initialize as master
	spi_initMaster(DIP204_SPI, &spiOptions);

	// Set selection mode: variable_ps, pcs_decode, delay
	spi_selectionMode(DIP204_SPI, 0, 0, 0);

	// Enable SPI
	spi_enable(DIP204_SPI);

	// setup chip registers
	spi_setupChipReg(DIP204_SPI, &spiOptions, FOSC0);

	// initialize LCD
	dip204_init(backlight_PWM, true);

}
