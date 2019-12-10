dwm-timer.c

Purpose: This examples demonstrates usage of nrf52832 peripheral (timer) within PANS user application

Entry function: app_thread_entry

Descriptions:
A timer is instantiated using the NRF52 SDK drivers for timer (NRF_DRV_TIMER).
Every 5 seconds, the timer expires. An IRQ is raised and the program will enter the IRQ handler. 
In ths handler, "Timer 2 expired" is printed out (over UART).

For more information of the APIs, please refer to DWM1001_API_Guide