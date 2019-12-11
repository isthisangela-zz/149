/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Simple user application.
 *
 * Copyright (c) 2016-2019, LEAPS. All rights reserved.
 *
 */

#include "dwm.h"
#include <stdio.h>
#include "nrf_drv_timer.h"

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
	"App   :  dwm-timer\n"	\
	"Built :  " __DATE__ " " __TIME__ "\n"	\
	"\n"

void timer2_event_handler(nrf_timer_event_t event_type, void * p_context)
{
	static int cnt = 0;

    switch (event_type) {
        case NRF_TIMER_EVENT_COMPARE0:
			printf("%d. Timer 2 expired!\n", ++cnt);
            break;

        default:
            /*Do nothing.*/
            break;
    }
}

/* interrupt handled from nordic driver, see nrf_drv_timer.c */
extern void TIMER2_IRQHandler(void);

void dwm_irq_hndlr(void *p_context)
{
	TIMER2_IRQHandler();
}

/**
 * Application thread
 *
 * @param[in] data  Pointer to user data
 */
void app_thread_entry(uint32_t data)
{
    dwm_irq_t irq;
    int rv;
    uint32_t err_code = NRF_SUCCESS;
    uint32_t time_ticks;
    const nrf_drv_timer_t timer2 = NRF_DRV_TIMER_INSTANCE(2);
    nrf_drv_timer_config_t t_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;

    irq.vector_nr = 10;
    irq.p_hndlr = dwm_irq_hndlr;

    rv = dwm_interrupt_register(&irq);
    if (rv != DWM_OK)
    printf("interrupt can't be registered, error %d\n", rv);

    err_code = nrf_drv_timer_init(&timer2, &t_cfg, timer2_event_handler);
    if (err_code != NRF_SUCCESS)
    printf("timer can't be initialized, error %lu\n", err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&timer2, 5000);
    nrf_drv_timer_extended_compare(
         &timer2, NRF_TIMER_CC_CHANNEL0, time_ticks,
		 NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrf_drv_timer_enable(&timer2);

    /* Initial message */
    printf(MSG_INIT);

    while (true) {}
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

	dwm_shell_compile();
	//Disabling ble by default as softdevice prevents debugging with breakpoints (due to priority)
	//dwm_ble_compile();
	dwm_le_compile();

	/* Create thread */
	rv = dwm_thread_create(THREAD_APP_PRIO, app_thread_entry, (void*)NULL,
			"app", THREAD_APP_STACK_SIZE, &hndl);
	APP_ERR_CHECK(rv);

	/* Start the thread */
	dwm_thread_resume(hndl);
}
