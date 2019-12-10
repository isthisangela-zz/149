// LoRa 9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example LoRa9x_RX
 
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"

 
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2


// SPI
#define SPI_SCLK    NRF_GPIO_PIN_MAP(0,12)
#define SPI_MISO    NRF_GPIO_PIN_MAP(1,9)
#define SPI_MOSI    NRF_GPIO_PIN_MAP(0,8)

#define RTC_WDI    NRF_GPIO_PIN_MAP(0, 11)


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

volatile RHMode     _mode;


int main(void) {
  ret_code_t error_code = NRF_SUCCESS;
  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

    // initialize LED
  nrf_gpio_pin_dir_set(7, NRF_GPIO_PIN_DIR_OUTPUT);

  nrf_drv_spi_t *spi_instance = &NRF_DRV_SPI_INSTANCE(1);

  nrf_drv_spi_config_t spi_config = {
    .sck_pin = SPI_SCLK,
    .mosi_pin = SPI_MOSI,
    .miso_pin = SPI_MISO,
    .ss_pin = RFM95_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };

  nrf_gpio_cfg_output(RTC_WDI);
  nrf_gpio_pin_set(RTC_WDI);

  setup();

  while (1) {
    loop();
  }
}

void setup() {

  nrf_gpio_pin_dir_set(RFM95_RST, NRF_GPIO_PIN_DIR_OUTPUT);
  //digitalWrite(RFM95_RST, HIGH);
  nrf_gpio_pin_write(RFM95_RST, 1);
 
  printf("Arduino LoRa TX Test!");
 
  // manual reset
  nrf_gpio_pin_write(RFM95_RST, 0);
  delay(10);
  nrf_gpio_pin_write(RFM95_RST, 1);
  delay(10);
 
  while (!init()) {
    printf("LoRa radio init failed");
    while (1);
  }
  printf("LoRa radio init OK!");
 
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!setFrequency(RF95_FREQ)) {
    printf("setFrequency failed");
    while (1);
  }
  printf("Set Freq to: ");
  printf(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  setTxPower(23, false);
}
 
int16_t packetnum = 0;  // packet counter, we increment per xmission
 
void loop() {
  printf("Sending to rf95_server");
  // Send a message to rf95_server
  
  char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket+13, 10);
  printf("Sending ");
  printf(radiopacket);
  radiopacket[19] = 0;
  
  printf("Sending...");
  nrf_delay_ms(10);
  send((uint8_t *)radiopacket, 20);
 
  printf("Waiting for packet to complete...");
  delay(10);
  waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
 
  printf("Waiting for reply...");
  delay(10);
  if (waitAvailableTimeout(1000)) { 
    // Should be a reply message for us now   
    if (recv(buf, &len)) {
      printf("Got reply: ");
      printf((char*)buf);
      printf("\n RSSI: ");
      printf(lastRssi(), DEC);    
    }
    else {
      printf("Receive failed");
    }
  }
  else {
    printf("No reply, is there a listener around?");
  }
  delay(1000);
}


bool send(const uint8_t* data, uint8_t len) {
  if (len > RH_RF95_MAX_MESSAGE_LEN)
    return false;

  waitPacketSent(); // Make sure we dont interrupt an outgoing message
  setModeIdle();

  if (!waitCAD()) 
    return false;  // Check channel activity

  // Position at the beginning of the FIFO
  spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);
  // The headers
  spiWrite(RH_RF95_REG_00_FIFO, _txHeaderTo);
  spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFrom);
  spiWrite(RH_RF95_REG_00_FIFO, _txHeaderId);
  spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFlags);
  // The message data
  spiBurstWrite(RH_RF95_REG_00_FIFO, data, len);
  spiWrite(RH_RF95_REG_22_PAYLOAD_LENGTH, len + RH_RF95_HEADER_LEN);

  setModeTx(); // Start the transmitter
  // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
  return true;
}

bool waitPacketSent() {
  while (_mode == RHModeTx)
    YIELD; // Wait for any previous transmit to finish
  return true;
}


void setModeIdle() {
  if (_mode != RHModeIdle) {
    spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
    _mode = RHModeIdle;
  }
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


void setModeTx() {
  if (_mode != RHModeTx) {
    spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX);
    spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
    _mode = RHModeTx;
  }
}

