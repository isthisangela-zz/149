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


#include "nrf_drv_uart.h"

#define PMTK_Q_RELEASE "$PMTK605*31"              ///< ask for the release and version
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"  ///< turn on ALL THE DATA

static nrf_drv_uart_t uart_instance = NRF_DRV_UART_INSTANCE(0);
static nrf_drv_uart_config_t config = NRF_DRV_UART_DEFAULT_CONFIG;

typedef struct {
	uint8_t data[1000];
	uint16_t cursor;
	uint16_t len;
} rxbuf_t;

uint16_t prev_cursor; 

static rxbuf_t rxbuf = {.len = sizeof(rxbuf.data), .cursor = 0};


#define UART_RX NRF_GPIO_PIN_MAP(0, 29)
#define UART_TX NRF_GPIO_PIN_MAP(0, 30)

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
	error_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(error_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    printf("Log initialized!\n");

	config.baudrate = 2576384;
	config.pselrxd = UART_RX;
	config.pseltxd = UART_TX;
	config.use_easy_dma = false;

	//nrf_drv_uart_init(&uart_instance, &config, nrf_uart_event_handler);	

	char str[] = "$GPGGA,002609.000,3752.5071,N,12215.4352,W,2,06,1.38,112.9,M,-24.9,M,0000,0000*50";

    nrf_gpio_pin_dir_set(UART_TX, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_dir_set(UART_RX, NRF_GPIO_PIN_DIR_INPUT);
	
	uint32_t err_code = NRF_SUCCESS;
	err_code = nrf_drv_uart_init(&uart_instance, &config, NULL);

	char release[20];
	//nrf_drv_uart_tx(&uart_instance, "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29", strlen("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"));

	nrf_drv_uart_rx_enable(&uart_instance);
	nrf_drv_uart_rx(&uart_instance, release, 19);
	printf("release: %s\n", release);

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
	nrf_drv_uart_rx_enable(&uart_instance);
	uint8_t data[1000];

	while (1) {
		nrf_drv_uart_rx_enable(&uart_instance);
		nrf_drv_uart_rx(&uart_instance, data, 156);

		data[160] = 0;
		printf("data %s\n", data);

		char * token = strtok(data, "\n");
		//printf("token %s\n", token);
		// loop through the string to extract all other tokens
		while( token != NULL ) {
		  if (is_GPGGA(token)) {

		  	get_time(token);

		  	printf("Time: %s\n", time);
		  	
		  	if (strlen(token) < strlen(str) - 5) {

		  		printf("No fix.\n");
		  		token = strtok(NULL, "\n");
		  		continue;
		  	}
		  	printf( "%s\n", token); //printing each token
		  	get_north(token);
		  	get_west(token);
		  	printf("Time: %s\n", time);
		  	printf("North: %s\n", north);
		  	printf("West: %s\n", west);
		  }
		  token = strtok(NULL, "\n");
		}
	}

	return 0;
}
