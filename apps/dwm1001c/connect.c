// Buckler Driver to connect to DWM1001C 

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_util.h"
#include "nrf_twi_mngr.h"
#include "nrf_gpio.h"
#include "dwm_driver.h"
#include "nrf_delay.h"

#include "buckler.h"

#include "max44009.h"

int main(void) {

  // Initialize
  printf("Hello!\n");
  // initialize display
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = NRF_GPIO_PIN_MAP(0, 2),
    .mosi_pin = NRF_GPIO_PIN_MAP(0, 25),
    .miso_pin = NRF_GPIO_PIN_MAP(0, 24),
    .ss_pin = NRF_GPIO_PIN_MAP(0, 26),
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  printf("2nd part\n");
  ret_code_t error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  printf("Error Check done.\n");
  uint8_t* readData = dwm_tag_init(&spi_instance);
  printf("initialize\n");
  while (readData[0] != 0x40 || readData[2] != 0x00) {
    printf("Config errored!");
    free(readData);
    readData = dwm_tag_init(&spi_instance);
  }
  while (!dwm_reset(&spi_instance)) {
    printf("Resetting");
  }
  while(1) {
    printf("looping\n");
    readData = dwm_read_pos(&spi_instance);
    // Extracting x, y, z coords
    if (readData[3] == 0x41) {
      uint8_t x[] = {readData[5], readData[6], readData[7], readData[8]};
      uint8_t y[] = {readData[9], readData[10], readData[11], readData[12]};
      uint8_t z[] = {readData[13], readData[14], readData[15], readData[16]};
      
      int x_m, y_m, z_m;
      memcpy(&x_m, &x, sizeof(x_m));
      memcpy(&y_m, &y, sizeof(y_m));
      memcpy(&z_m, &z, sizeof(z_m));

      printf("Coordinates: (%f, %f, %f)\n", x_m / 1000.0, y_m / 1000.0, z_m / 1000.0);
    }
    nrf_delay_ms(1000);
  }
}
