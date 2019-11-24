// nrf51_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing server
// with the RH_NRF51 class. RH_NRF51 class does not provide for addressing or
// reliability, so you should only use RH_NRF51 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example nrf51_client
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
  if (!nrf51.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf51.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf51.setRF(RH_NRF51::DataRate2Mbps, RH_NRF51::TransmitPower0dBm))
    Serial.println("setRF failed");    
}

void loop()
{
  if (available()) {
    // Should be a message for us now   
    uint8_t buf[RH_NRF51_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (nrf51.recv(buf, &len)) {
      Serial.print("got request: ");
      Serial.println((char*)buf);
      // Send a reply
      uint8_t data[] = "And hello back to you";
      nrf51.send(data, sizeof(data));
      nrf51.waitPacketSent();
      Serial.println("Sent a reply");
    }
    else
    {
      Serial.println("recv failed");
    }
  }
}

