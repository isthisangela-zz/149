// DWM1001C Driver

#pragma once
#include "app_error.h"
#include "nrf_drv_spi.h"

// Function to initialize the connected tag as a tag.
uint8_t* dwm_tag_init(nrf_drv_spi_t* s);

// Function to read the update rate from the connected tag.
uint8_t *dwm_read_rate(nrf_drv_spi_t *s);

// Function to read the position from the connected tag.
uint8_t *dwm_read_pos(nrf_drv_spi_t *s);

// Function to reset the connected tag.
bool dwm_reset(nrf_drv_spi_t *s);
