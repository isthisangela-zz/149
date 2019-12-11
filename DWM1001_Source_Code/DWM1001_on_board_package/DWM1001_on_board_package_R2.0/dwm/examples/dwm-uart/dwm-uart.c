/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * UART example.
 *
 * Copyright (c) 2016-2019, LEAPS. All rights reserved.
 *
 */

#include "dwm.h"
#include <stdio.h>
#include "nrf_drv_uart.h"
#include "sdk_common.h"

/* Thread priority */
#ifndef THREAD_APP_PRIO
#define THREAD_APP_PRIO	20
#endif /* THREAD_APP_PRIO */

/* Thread stack size */
#ifndef THREAD_APP_STACK_SIZE
#define THREAD_APP_STACK_SIZE	(3 * 1024)
#endif /* THREAD_APP_STACK_SIZE */

#define APP_ERR_CHECK(err_code)	\
do {							\
	if ((err_code) != DWM_OK)	\
		printf("err: line(%u) code(%u)", __LINE__, (err_code));\
} while (0)						\

#define MSG_INIT	\
	"\n\n"	\
	"App   :  dwm-simple\n"	\
	"Built :  " __DATE__ " " __TIME__ "\n"	\
	"\n"

#define TIMEOUT_BETWEEN_BYTES_US 1000000

typedef struct {
	uint8_t data[1000];
	uint16_t cursor;
	uint16_t len;
} rxbuf_t;

static dwm_mutex_t lock;
static dwm_cond_t signal;

static nrf_drv_uart_t uart_instance = NRF_DRV_UART_INSTANCE(0);
static nrf_drv_uart_config_t config = NRF_DRV_UART_DEFAULT_CONFIG;

static rxbuf_t rxbuf = {.len = sizeof(rxbuf.data), .cursor = 0};

/* UART event handler */
void nrf_uart_event_handler(nrf_drv_uart_event_t * p_event, void * p_context)
{
	switch (p_event->type) {
		case NRF_DRV_UART_EVT_RX_DONE:
			rxbuf.cursor += p_event->data.rxtx.bytes;

			if (*p_event->data.rxtx.p_data == '\r') {
				nrf_drv_uart_rx_abort(&uart_instance);
				dwm_cond_signal(&signal);
			} else {
				nrf_drv_uart_rx(&uart_instance, &rxbuf.data[rxbuf.cursor], sizeof(char));
			}
			break;

		case NRF_DRV_UART_EVT_ERROR:
			nrf_drv_uart_rx_abort(&uart_instance);
			nrf_drv_uart_tx_abort(&uart_instance);
			dwm_cond_signal(&signal);
			break;

		default:
			break;
	}
}

/* interrupt handled from nordic driver, see nrf_drv_uart.c */
extern void UART0_IRQHandler(void);
void dwm_irq_hndlr(void *p_context)
{
	UART0_IRQHandler();
}

/**
 * Application thread
 *
 * @param[in] data  Pointer to user data
 */
void app_thread_entry(uint32_t data)
{

    /* Initial message */
    printf(MSG_INIT);

    dwm_irq_t irq;
	int rv;
    uint32_t err_code = NRF_SUCCESS;
    const char *ok_msg = "\n\rUART ready, write something and press ENTER!\r\n";
    const char *err_msg = "ERROR!\r\n";
    const char *echo_prefix = "received: ";

    dwm_cond_init(&signal, &lock);

    irq.vector_nr = 2;
	irq.p_hndlr = dwm_irq_hndlr;

	rv = dwm_interrupt_register(&irq);
	if (rv == DWM_OK) {
		config.pselrxd = 11;
		config.pseltxd = 5;
		config.use_easy_dma = false;

		err_code = nrf_drv_uart_init(&uart_instance, &config, nrf_uart_event_handler);
		if (err_code == NRF_SUCCESS)
			nrf_drv_uart_tx(&uart_instance, (uint8_t const *)ok_msg, strlen(ok_msg));
		else
			nrf_drv_uart_tx(&uart_instance, (uint8_t const *)err_msg, strlen(err_msg));

		nrf_drv_uart_rx_enable(&uart_instance);
		nrf_drv_uart_rx(&uart_instance, rxbuf.data, sizeof(char));
	}

	while (true) {
		dwm_cond_wait(&signal);

		nrf_drv_uart_tx(&uart_instance, (uint8_t const *)echo_prefix, strlen(echo_prefix));
		while (nrf_drv_uart_tx_in_progress(&uart_instance))
			dwm_thread_delay(1);


		rxbuf.data[rxbuf.cursor++] = '\n';
		nrf_drv_uart_tx(&uart_instance, (uint8_t const *)rxbuf.data, rxbuf.cursor);

		while (nrf_drv_uart_tx_in_progress(&uart_instance))
			dwm_thread_delay(1);

		rxbuf.cursor = 0;
		nrf_drv_uart_rx(&uart_instance, rxbuf.data, sizeof(char));

	}
}

/**
 * Application entry point. Initialize application thread.
 *
 * @warning ONLY ENABLING OF LOCATION ENGINE OR BLE AND CREATION AND STARTING OF
 * USER THREADS CAN BE DONE IN THIS FUNCTION
 */
void dwm_user_start(void)
{
	uint8_t hndl;
	int rv;

	//Disabling ble by default as softdevice prevents debugging with breakpoints (due to priority)
	//dwm_ble_compile();
	dwm_le_compile();

        /* Initial message */
        printf(MSG_INIT);

	/* Create thread */
	rv = dwm_thread_create(THREAD_APP_PRIO, app_thread_entry, (void*)NULL,
			"app", THREAD_APP_STACK_SIZE, &hndl);
	APP_ERR_CHECK(rv);

	/* Start the thread */
	dwm_thread_resume(hndl);
}
