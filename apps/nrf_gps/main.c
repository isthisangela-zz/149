#include <stdio.h>
#include <regex.h> 
#include <string.h>

#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_power.h"
#include "nrf_serial.h"
#include "app_timer.h"
#include "kobukiUtilities.h"


#include "app_error.h"
#include "app_util.h"
#include "boards.h"

#include "nrf_drv_uart.h"
extern const nrf_serial_t * serial_ref;


// #define PMTK_Q_RELEASE "$PMTK605*31"              ///< ask for the release and version
// #define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"  ///< turn on ALL THE DATA

// #define SERIAL_FIFO_TX_SIZE 32
// #define SERIAL_FIFO_RX_SIZE 32
#define UART_RX NRF_GPIO_PIN_MAP(0, 14)

#define UART_TX NRF_GPIO_PIN_MAP(0, 30)

// static void sleep_handler(void)
// {
//     __WFE();
//     __SEV();
//     __WFE();
// }


// #define SERIAL_BUFF_TX_SIZE 1
// #define SERIAL_BUFF_RX_SIZE 1
// extern const nrf_serial_t * serial_ref;
// extern const nrf_serial_queues_t * serial_queue;

// NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart0_drv_config,
//                       BUCKLER_UART_RX, BUCKLER_UART_TX,
//                       0, 0,
//                       NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
//                       NRF_UART_BAUDRATE_115200,
//                       UART_DEFAULT_CONFIG_IRQ_PRIORITY);


// NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);

// NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_DMA,
//                       &serial_queue, &serial_buffs, NULL, NULL);


bool is_GPGGA(char *str) {
	if (strlen(str) < 10)
		return false;
	char prefix[6] = "$GPGGA"; 
	for (int i = 0; i < 5; i++) {
		if (prefix[i] != str[i])
			return false;
	}
	return true;
}

char time[7];

void get_time(char *str) {
	for (int i = 0; i < 6; i++) {
		time[i] = str[i + 7];
	}
}

char north[10];
void get_north(char *str) {
	if (strlen(str) < 40) {
		return;
	}
	for (int i = 0; i < 9; i++) {
		north[i] = str[i + 18];
	}
}

char west[10];
void get_west(char *str) {
	if (strlen(str) < 10) {
		return;
	}
	for (int i = 0; i < 10; i++) {
		west[i] = str[i + 30];
	}
}


int main(int argc, char const *argv[])
{
    ret_code_t error_code = NRF_SUCCESS;
    error_code = nrf_drv_clock_init();
	error_code = NRF_LOG_INIT(NULL);

    APP_ERROR_CHECK(error_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    printf("Log initialized!\n");

    //nrf_drv_clock_lfclk_request(NULL);
    //error_code = app_timer_init();


	// config.baudrate = 2576384;
	// config.pselrxd = UART_RX;
	// config.pseltxd = UART_TX;
	// config.use_easy_dma = false;


	//error_code = nrf_serial_init(serial_ref, &m_uart0_drv_config, &serial_config);
    printf("fault\n");

	//nrf_drv_uart_init(&uart_instance, &config, nrf_uart_event_handler);	

//	char str[] = "$GPGGA,002609.000,3752.5071,N,12215.4352,W,2,06,1.38,112.9,M,-24.9,M,0000,0000*50";

    // nrf_gpio_pin_dir_set(UART_TX, NRF_GPIO_PIN_DIR_OUTPUT);
    // nrf_gpio_pin_dir_set(UART_RX, NRF_GPIO_PIN_DIR_INPUT);
	
//	uint32_t err_code = NRF_SUCCESS;
	//err_code = nrf_drv_uart_init(&uart_instance, &config, NULL);

	//char release[20];
	//nrf_drv_uart_tx(&uart_instance, "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29", strlen("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"));

	// nrf_drv_uart_rx_enable(&uart_instance);
	// nrf_drv_uart_rx(&uart_instance, release, 19);
	// printf("release: %s\n", release);

	// nrf_drv_uart_tx(&uart_instance, "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28", strlen("$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"));
	// nrf_drv_uart_tx(&uart_instance, "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28", strlen("$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"));
	// nrf_drv_uart_tx(&uart_instance, "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28", strlen("$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"));
	// nrf_drv_uart_tx(&uart_instance, "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28", strlen("$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"));
	// nrf_drv_uart_tx(&uart_instance, "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28", strlen("$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"));

	// nrf_drv_uart_tx(&uart_instance, PMTK_SET_NMEA_OUTPUT_ALLDATA, strlen(PMTK_SET_NMEA_OUTPUT_ALLDATA));
	// nrf_drv_uart_tx(&uart_instance, PMTK_SET_NMEA_OUTPUT_ALLDATA, strlen(PMTK_SET_NMEA_OUTPUT_ALLDATA));
	// nrf_drv_uart_tx(&uart_instance, PMTK_SET_NMEA_OUTPUT_ALLDATA, strlen(PMTK_SET_NMEA_OUTPUT_ALLDATA));
	// nrf_drv_uart_tx(&uart_instance, PMTK_SET_NMEA_OUTPUT_ALLDATA, strlen(PMTK_SET_NMEA_OUTPUT_ALLDATA));
	// nrf_drv_uart_tx(&uart_instance, PMTK_SET_NMEA_OUTPUT_ALLDATA, strlen(PMTK_SET_NMEA_OUTPUT_ALLDATA));
	//nrf_drv_uart_rx_enable(&uart_instance);
	//uint8_t data[1000];

	while (1) {

		char header[10];
        error_code = nrf_serial_read(serial_ref, header, sizeof(header), NULL, 1000);
   		printf("fault 1\n");

        printf("data %s\n", header);
	// 	memset(data, 0, 1000);
	// 	err_code = nrf_drv_uart_init(&uart_instance, &config, NULL);
	// 	nrf_drv_uart_rx_enable(&uart_instance);
	// 	printf("waiting\n");
	// 	nrf_drv_uart_rx(&uart_instance, data, sizeof(char));

	// 	data[160] = 0;
	// 	printf("data %s\n", data);

	// 	char * token = strtok(data, "\n");
	// 	//printf("token %s\n", token);
	// 	// loop through the string to extract all other tokens
	// 	while( token != NULL ) {
	// 	  if (is_GPGGA(token)) {

	// 	  	get_time(token);

	// 	  	printf("Time: %s\n", time);
		  	
	// 	  	if (strlen(token) < strlen(str) - 5) {

	// 	  		printf("No fix.\n");
	// 	  		token = strtok(NULL, "\n");
	// 	  		continue;
	// 	  	}
	// 	  	printf( "%s\n", token); //printing each token
	// 	  	get_north(token);
	// 	  	get_west(token);
	// 	  	printf("Time: %s\n", time);
	// 	  	printf("North: %s\n", north);
	// 	  	printf("West: %s\n", west);
	// 	  }
	// 	  token = strtok(NULL, "\n");
	// 	}
	// 	nrf_delay_ms(6);
	// }

	// return 0;
	}
}
