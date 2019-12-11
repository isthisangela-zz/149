dwm-uart.c

Purpose: This examples demonstrates usage of nrf52832 peripheral (uart) within PANS user application

Entry function: app_thread_entry

Descriptions:
In this example, the uart instance is reused in order to process any data sent over uart to the module, and send it back with the prefix : "received:".
For example if "test" is sent over uart, the device will return "received: test"

For more information of the APIs, please refer to DWM1001_API_Guide