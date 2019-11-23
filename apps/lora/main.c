// Blink app
//
// Blinks the LED on SPARKFUN

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <RH_NRF51.h>
//#include <RH_RF95.h>

#include "pthread.h"
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
uint8_t _buf[RH_NRF51_MAX_PAYLOAD_LEN+1];


spi_config.sck_pin    = SPI_SCLK;
spi_config.miso_pin   = SPI_MISO;
spi_config.mosi_pin   = SPI_MOSI;
spi_config.ss_pin     = RTC_CS;
spi_config.frequency  = NRF_DRV_SPI_FREQ_2M;
spi_config.mode       = NRF_DRV_SPI_MODE_0;
spi_config.bit_order  = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

nrf_gpio_cfg_output(RTC_WDI);
nrf_gpio_pin_set(RTC_WDI);


bool init() {
    // Enable the High Frequency clock to the system as a whole
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) { }
    
    // Enables the DC/DC converter when the radio is enabled. Need this!
    NRF_POWER->DCDCEN = 0x00000001; 

    // Disable and reset the radio
    NRF_RADIO->POWER = RADIO_POWER_POWER_Disabled;
    NRF_RADIO->POWER = RADIO_POWER_POWER_Enabled;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE   = 1;
    // Wait until we are in DISABLE state
    while (NRF_RADIO->EVENTS_DISABLED == 0) {}

    // Physical on-air address is set in PREFIX0 + BASE0 by setNetworkAddress
    NRF_RADIO->TXADDRESS    = 0x00;	// Use logical address 0 (PREFIX0 + BASE0)
    NRF_RADIO->RXADDRESSES  = 0x01;	// Enable reception on logical address 0 (PREFIX0 + BASE0)

    // Configure the CRC
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Number of checksum bits
    NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value      
    NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1

    // These shorts will make the radio transition from Ready to Start to Disable automatically
    // for both TX and RX, which makes for much shorter on-air times
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos)
	              | (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);

    NRF_RADIO->PCNF0 = ((8 << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk); // Payload length in bits

    // Make sure we are powered down
    setModeIdle();

    // Set a default network address
    uint8_t default_network_address[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    setNetworkAddress(default_network_address, sizeof(default_network_address));

    setChannel(2); // The default, in case it was set by another app without powering down
    setRF(RH_NRF51::DataRate2Mbps, RH_NRF51::TransmitPower0dBm);

    return true;
}



// void setModeTx() {
//     if (spi_config.mode != RHModeTx)
//     {
// 	spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX);
// 	spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
// 	spi_config.mode = RHModeTx;
//     }
// }

void setModeTx()
{
    if (spi_config.mode != RHModeTx)
    {
	setModeIdle(); // Can only start RX from DISABLE state
	// Radio will transition automatically to Disable state at the end of transmission
	NRF_RADIO->PACKETPTR = (uint32_t)_buf;
	NRF_RADIO->EVENTS_DISABLED = 0U; // So we can detect end of transmission
	NRF_RADIO->TASKS_TXEN = 1;
	spi_config.mode = RHModeTx;
    }
}


void setModeIdle() {
    if (spi_config.mode != RHModeIdle)
    {
	spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
	spi_config.mode = RHModeIdle;
    }
}

bool waitPacketSent() {
	while (spi_config.mode == RHModeTx)
		pthread_yield();
	return true;
}


bool send(const uint8_t* data, uint8_t len)
{
    if (len > RH_NRF51_MAX_MESSAGE_LEN)
	return false;
    // Set up the headers
    _buf[0] = len + RH_NRF51_HEADER_LEN;
    _buf[1] = 0xff;
    _buf[2] = 0xff;
    _buf[3] = 0;
    _buf[4] = 0;
    memcpy(_buf+RH_NRF51_HEADER_LEN+1, data, len);

    _rxBufValid = false;
    setModeTx();
    // Radio will return to Disabled state after transmission is complete
    _txGood++;
    return true;
}


// bool send(const uint8_t* data, uint8_t len) {
//     if (len > RH_RF95_MAX_MESSAGE_LEN)
// 	return false;

//     waitPacketSent(); // Make sure we dont interrupt an outgoing message
//     setModeIdle();

//     // Position at the beginning of the FIFO
//     spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);
//     // The headers
//     spiWrite(RH_RF95_REG_00_FIFO, _txHeaderTo);
//     spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFrom);
//     spiWrite(RH_RF95_REG_00_FIFO, _txHeaderId);
//     spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFlags);
//     // The message data
//     spiBurstWrite(RH_RF95_REG_00_FIFO, data, len);
//     spiWrite(RH_RF95_REG_22_PAYLOAD_LENGTH, len + RH_RF95_HEADER_LEN);

//     setModeTx(); // Start the transmitter
//     // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
//     return true;
// }


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


void spiBurstWrite(uint8_t reg, uint8_t* write_buf, size_t len){
  if (len > 256) return;
  uint8_t buf[257];
  buf[0] = 0x80 | reg;
  memcpy(buf+1, write_buf, len);

  nrf_drv_spi_init(spi_instance, &spi_config, NULL, NULL);
  nrf_drv_spi_transfer(spi_instance, buf, len+1, NULL, 0);
  nrf_drv_spi_uninit(spi_instance);
}


// reads a buffer of values
void spiBurstRead(uint8_t reg, uint8_t* read_buf, size_t len){
  if (len > 256) return;
  uint8_t readreg = reg;
  uint8_t buf[257];

  nrf_drv_spi_init(spi_instance, &spi_config, NULL, NULL);
  nrf_drv_spi_transfer(spi_instance, &readreg, 1, buf, len+1);
  nrf_drv_spi_uninit(spi_instance);

  memcpy(read_buf, buf+1, len);
}

// writes one uint8_t value
void spiWrite(uint8_t reg, uint8_t val) {
  uint8_t buf[2];
  buf[0] = 0x80 | reg;
  memcpy(buf+1, &val, 1);
  nrf_drv_spi_init(spi_instance, &spi_config, NULL, NULL);
  nrf_drv_spi_transfer(spi_instance, &buf, 1, NULL, 0);
  nrf_drv_spi_uninit(spi_instance);
}


// reads one uint8_t value
uint8_t spiRead(uint8_t reg) {
  uint8_t buf;
  nrf_drv_spi_init(spi_instance, &spi_config, NULL, NULL);
  nrf_drv_spi_transfer(spi_instance, &reg, 1, &buf, 1);
  nrf_drv_spi_uninit(spi_instance);

  return buf;
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

