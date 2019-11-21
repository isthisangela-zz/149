// Blink app
//
// Blinks the LED on SPARKFUN

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <RH_RF95.h>

#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrfx_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"

#include "sparkfun.h"

#include "nrf_drv_gpiote.h"
#include "nrf_drv_spi.h"


static const nrf_drv_spi_t* spi_instance;
static nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

spi_config.sck_pin    = SPI_SCLK;
spi_config.miso_pin   = SPI_MISO;
spi_config.mosi_pin   = SPI_MOSI;
spi_config.ss_pin     = RTC_CS;
spi_config.frequency  = NRF_DRV_SPI_FREQ_2M;
spi_config.mode       = NRF_DRV_SPI_MODE_0;
spi_config.bit_order  = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

nrf_gpio_cfg_output(RTC_WDI);
nrf_gpio_pin_set(RTC_WDI);


// Adapted from Radiohead:
// C++ level interrupt handler for this instance
// LORA is unusual in that it has several interrupt lines, and not a single, combined one.
// On MiniWirelessLoRa, only one of the several interrupt lines (DI0) from the RFM95 is usefuly 
// connnected to the processor.
// We use this to get RxDone and TxDone interrupts
void handleInterrupt() {
    // Read the interrupt register
    uint8_t irq_flags = spiRead(RH_RF95_REG_12_IRQ_FLAGS);
    if (_mode == RHModeRx && irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR)) {
	    _rxBad++;
    }
    else if (_mode == RHModeRx && irq_flags & RH_RF95_RX_DONE) {
	    // Have received a packet
	    uint8_t len = spiRead(RH_RF95_REG_13_RX_NB_BYTES);

	    // Reset the fifo read ptr to the beginning of the packet
	    spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, spiRead(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
	    spiBurstRead(RH_RF95_REG_00_FIFO, _buf, len);
    	_bufLen = len;
	    spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

	    // Remember the RSSI of this packet
      // this is according to the doc, but is it really correct?
      // weakest receiveable signals are reported RSSI at about -66
      _lastRssi = spiRead(RH_RF95_REG_1A_PKT_RSSI_VALUE) - 137;
      // We have received a message.
      validateRxBuf(); 
      if (_rxBufValid)
	      setModeIdle(); // Got one 
    }
    else if (_mode == RHModeTx && irq_flags & RH_RF95_TX_DONE) {
    _txGood++;
    setModeIdle();
    }
    spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
}

void spiWrite(uint8_t buf, int len) {

  nrf_drv_spi_init(spi_instance, &spi_config, NULL, NULL);
  nrf_drv_spi_transfer(spi_instance, &readreg, 1, buf, len+1);
  nrf_drv_spi_uninit(spi_instance);
}

// LED array
static uint8_t LED = SPARKFUN_LED;

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize GPIO driver
  if (!nrfx_gpiote_is_init()) {
    error_code = nrfx_gpiote_init();
  }
  APP_ERROR_CHECK(error_code);

  // configure leds
  // manually-controlled (simple) output, initially set
  nrfx_gpiote_out_config_t out_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
  error_code = nrfx_gpiote_out_init(LED, &out_config);
  APP_ERROR_CHECK(error_code);

  // loop forever
  while (1) {
    nrf_delay_ms(250); 
    nrf_gpio_pin_toggle(LED);
  }
}

