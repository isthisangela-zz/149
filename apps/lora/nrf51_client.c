// nrf51_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_NRF51 class. RH_NRF51 class does not provide for addressing or
// reliability, so you should only use RH_NRF51 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example nrf51_server.
// Tested on RedBearLabs nRF51822 and BLE Nano kit, built with Arduino 1.6.4.
// See http://redbearlab.com/getting-started-nrf51822/
// for how to set up your Arduino build environment
// Also tested with Sparkfun nRF52832 breakout board, witth Arduino 1.6.13 and
// Sparkfun nRF52 boards manager 0.2.3
#include <NRFDriver.h>

int main(void) {
    nrf_power_dcdcen_set(1);

  // Turn on power gate
  nrf_gpio_cfg_output(MAX44009_EN);
  nrf_gpio_cfg_output(ISL29125_EN);
  nrf_gpio_cfg_output(MS5637_EN);
  nrf_gpio_cfg_output(SI7021_EN);
  nrf_gpio_pin_set(MAX44009_EN);
  nrf_gpio_pin_set(ISL29125_EN);
  nrf_gpio_pin_set(MS5637_EN);
  nrf_gpio_pin_set(SI7021_EN);

  ab1815_init(&spi_instance);
  ab1815_get_config(&config);
  config.auto_rst = 1;
  config.write_rtc = 1;
  ab1815_set_config(config);

  setup();

  while (1) {
    loop();
  }
}

void setup() {
  if (!init())
  printf("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!setChannel(1))
    printf("setChannel failed");
  if (!setRF(RH_NRF51::DataRate2Mbps, RH_NRF51::TransmitPower0dBm))
    printf("setRF failed"); 
}


void loop() {
  printf("Sending to nrf51_server");
  // Send a message to nrf51_server
  uint8_t data[] = "Hello World!";
  send(data, sizeof(data));
  waitPacketSent();

  // Now wait for a reply
  uint8_t buf[RH_NRF51_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (waitAvailableTimeout(500)) { 
    // Should be a reply message for us now   
    if (recv(buf, &len)) {
      printf("got reply: ");
      Serial.println((char*)buf);
    }
    else
    {
      Serial.println("recv failed");
    }
  }
  else
  {
    Serial.println("No reply, is nrf51_server running?");
  }

  delay(400);
}

