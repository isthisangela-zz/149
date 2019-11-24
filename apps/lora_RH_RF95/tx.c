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

#include <RH_RF95.h>
 
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
 
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
 

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;
  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

    // initialize LED
  nrf_gpio_pin_dir_set(7, NRF_GPIO_PIN_DIR_OUTPUT);


  setup();
  while (1) {
    loop();
  }
}


void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
 
  printf("Arduino LoRa TX Test!");
 
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
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
  nrf_delay(10);
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
