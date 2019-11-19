// Pin definitions for the Berkeley Buckler revision B

#pragma once

#include "nrf_gpio.h"
#include "nrf_saadc.h"


// TODO: Define SPI pins 

// LED
#define SPARKFUN_LED NRF_GPIO_PIN_MAP(0,7)

// Buttons
#define SPARKFUN_RESET NRF_GPIO_PIN_MAP(0,21)
#define SPARKFUN_BUTTON NRF_GPIO_PIN_MAP(0,6)






// Might use later:

// LCD screen
#define SPARKFUN_LCD_SCLK NRF_GPIO_PIN_MAP(0,17)
#define SPARKFUN_LCD_MISO NRF_GPIO_PIN_MAP(0,16)
#define SPARKFUN_LCD_MOSI NRF_GPIO_PIN_MAP(0,15)
#define SPARKFUN_LCD_CS   NRF_GPIO_PIN_MAP(0,18)

// Analog accelerometer
#define BUCKLER_ANALOG_ACCEL_X NRF_SAADC_INPUT_AIN5 // P0.29
#define BUCKLER_ANALOG_ACCEL_Y NRF_SAADC_INPUT_AIN6 // P0.30
#define BUCKLER_ANALOG_ACCEL_Z NRF_SAADC_INPUT_AIN7 // P0.31

// Re-define GPIOs for compatibility with simple_logger
#define SD_CARD_ENABLE      BUCKLER_SD_ENABLE
#define SD_CARD_SPI_CS      BUCKLER_SD_CS
#define SD_CARD_SPI_MISO    BUCKLER_SD_MISO
#define SD_CARD_SPI_MOSI    BUCKLER_SD_MOSI
#define SD_CARD_SPI_SCLK    BUCKLER_SD_SCLK

// Define which SPI to use
#define SD_CARD_SPI_INSTANCE    NRF_SPI1

// UART serial connection (to Kobuki)
#define BUCKLER_UART_RX NRF_GPIO_PIN_MAP(0,6)
#define BUCKLER_UART_TX NRF_GPIO_PIN_MAP(0,8)

