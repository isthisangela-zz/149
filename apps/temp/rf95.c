#include "rf95.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"


static const nrf_drv_spi_t* spi_instance;
static nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

init(RH_RF95* rf95) {
	spi_config.sck_pin    = SPI_SCLK;
  spi_config.miso_pin   = SPI_MISO;
  spi_config.mosi_pin   = SPI_MOSI;
  spi_config.ss_pin     = RTC_CS;
  spi_config.frequency  = NRF_DRV_SPI_FREQ_2M;
  spi_config.mode       = NRF_DRV_SPI_MODE_0;
  spi_config.bit_order  = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

  nrf_gpio_cfg_output(RTC_WDI);
  nrf_gpio_pin_set(RTC_WDI);
  nrf_gpio_pin_output(rf95->_slaveSelectPin);
  nrf_gpio_pin_set(rf95->_slaveSelectPin);
  nrf_delay_ms(100);

  int interrruptNumber = 
}

static void interrupt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  // read and clear interrupts
  uint8_t status = 0;
  ab1815_read_reg(AB1815_STATUS, &status, 1);

  if (status & 0x4 && interrupt_callback) {
    // call user callback
    interrupt_callback();
  }
}