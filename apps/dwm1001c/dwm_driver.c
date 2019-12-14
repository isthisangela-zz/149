// DWM1001C Driver

#include <stdlib.h>
#include <stdbool.h>
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "app_error.h"
#include "dwm_driver.h"

static nrf_drv_spi_t* spi;
static void update_message(uint8_t *msg, size_t msg_len);

static void update_message(uint8_t *msg, size_t msg_len) {
  for (size_t i = 0; i < msg_len; i++) {
    msg[i] = msg[i] << 1;
  }
}


uint8_t* dwm_tag_init(nrf_drv_spi_t* s) {
  spi = s;
  // we want 11011110 = de
  // uwb_mode active, fw_update_en, ble_en, led_en, reserved, loc-engine_en, low_power_en
  uint8_t data[4];
  data[0] = 0x05;
  data[1] = 0x02;
  data[2] = 0xDE;
  data[3] = 0x00;
  update_message(data, 4);
  printf("1\n");
  ret_code_t err_code = nrf_drv_spi_transfer(spi, data, 4, NULL, 0);
  printf("2\n");
  APP_ERROR_CHECK(err_code);
  if (err_code != NRF_SUCCESS) {
    return NULL;  
  }
  uint8_t size_num[2];
    printf("3\n");

  err_code = nrf_drv_spi_transfer(spi, NULL, 0, size_num, 2);
    printf("4\n");

  while (size_num[0] == 0x00) {
    printf("loop\n");
    APP_ERROR_CHECK(err_code);
    if (err_code != NRF_SUCCESS) {
      return NULL;
    }
    nrf_delay_ms(10);
    err_code = nrf_drv_spi_transfer(spi, NULL, 0, size_num, 2);
  } 
  printf("5\n");


  printf("%x %x\n", size_num[0], size_num[1]);
  uint8_t* readData = (uint8_t *)malloc(sizeof(uint8_t)*size_num[0]);
  err_code = nrf_drv_spi_transfer(spi, NULL, 0, readData, size_num[0]);
  APP_ERROR_CHECK(err_code);
  if (err_code != NRF_SUCCESS) {
    return NULL;
  }
  printf("%x %x %x\n", readData[0], readData[1], readData[2]);
  return readData;
}

bool dwm_reset(nrf_drv_spi_t *s) {
  spi = s;
  uint8_t data[2];
  data[0] = 0x14;
  data[1] = 0x00;
  update_message(data, 2);
  ret_code_t err_code = nrf_drv_spi_transfer(spi, data, 2, NULL, 0);
  APP_ERROR_CHECK(err_code);
  if (err_code != NRF_SUCCESS) {
    return false;  
  }
  uint8_t size_num[2];
  err_code = nrf_drv_spi_transfer(spi, NULL, 0, size_num, 2);
  while (size_num[0] == 0x00) {
    APP_ERROR_CHECK(err_code);
    if (err_code != NRF_SUCCESS) {
      return false;
    }
    nrf_delay_ms(10);
    err_code = nrf_drv_spi_transfer(spi, NULL, 0, size_num, 2);
  }
  printf("%x %x\n", size_num[0], size_num[1]);
  uint8_t* readData = (uint8_t *)malloc(sizeof(uint8_t)*size_num[0]);
  err_code = nrf_drv_spi_transfer(spi, NULL, 0, readData, size_num[0]);
  APP_ERROR_CHECK(err_code);
  if (err_code != NRF_SUCCESS) {
    return false;
  }
  return (readData[0] == 0x40 && readData[2] == 0x00);
}
uint8_t *dwm_read_rate(nrf_drv_spi_t *s) {
  spi = s;
  uint8_t data[2];
  data[0] = 0x04;
  data[1] = 0x00;
  printf("input commands: %x %x\n", data[0], data[1]);
  update_message(data, 2);
  ret_code_t err_code = nrf_drv_spi_transfer(spi, data, 2, NULL, 0);
  APP_ERROR_CHECK(err_code);
  if (err_code != NRF_SUCCESS) {
    return NULL;  
  }
  uint8_t size_num[2];
  err_code = nrf_drv_spi_transfer(spi, NULL, 0, size_num, 2);
  while (size_num[0] == 0x00) {
    APP_ERROR_CHECK(err_code);
    if (err_code != NRF_SUCCESS) {
      return NULL;
    }
    nrf_delay_ms(10);
    err_code = nrf_drv_spi_transfer(spi, NULL, 0, size_num, 2);
  }
  printf("%x %x\n", size_num[0], size_num[1]);
  uint8_t* readData = (uint8_t *)malloc(sizeof(uint8_t)*size_num[0]);
  err_code = nrf_drv_spi_transfer(spi, NULL, 0, readData, size_num[0]);
  APP_ERROR_CHECK(err_code);
  if (err_code != NRF_SUCCESS) {
    return NULL;
  }
  int i = 0;
  while (i < size_num[0]) {
    printf("%x ", readData[i++]);
  }
  printf("\n");
  return readData;

}

uint8_t *dwm_write_rate(nrf_drv_spi_t *s) {
  spi = s;
  uint8_t data[6];
  data[0] = 0x03;
  data[1] = 0x04;
  data[2] = 0x0B;
  data[3] = 0x00;
  data[4] = 0x32;
  data[5] = 0x00;
  printf("input commands: %x %x\n", data[0], data[1]);
  update_message(data, 6);
  ret_code_t err_code = nrf_drv_spi_transfer(spi, data, 6, NULL, 0);
  APP_ERROR_CHECK(err_code);
  if (err_code != NRF_SUCCESS) {
    return NULL;  
  }
  uint8_t size_num[2];
  err_code = nrf_drv_spi_transfer(spi, NULL, 0, size_num, 2);
  while (size_num[0] == 0x00) {
    APP_ERROR_CHECK(err_code);
    if (err_code != NRF_SUCCESS) {
      return NULL;
    }
    nrf_delay_ms(10);
    err_code = nrf_drv_spi_transfer(spi, NULL, 0, size_num, 2);
  }
  printf("%x %x\n", size_num[0], size_num[1]);
  uint8_t* readData = (uint8_t *)malloc(sizeof(uint8_t)*size_num[0]);
  err_code = nrf_drv_spi_transfer(spi, NULL, 0, readData, size_num[0]);
  APP_ERROR_CHECK(err_code);
  if (err_code != NRF_SUCCESS) {
    return NULL;
  }
  int i = 0;
  while (i < size_num[0]) {
    printf("%x ", readData[i++]);
  }
  printf("\n");
  return readData;

}

uint8_t *dwm_read_pos(nrf_drv_spi_t *s) {
  // This TLV request corresponds to dwm_pos_get
  spi = s;
  uint8_t data[2];
  data[0] = 0x02;
  data[1] = 0x00;
  printf("Input commands: %x %x\n", data[0], data[1]);
  update_message(data, 2);
  // Sending TLV request to tag
  ret_code_t err_code = nrf_drv_spi_transfer(spi, data, 2, NULL, 0);
  APP_ERROR_CHECK(err_code);
  if (err_code != NRF_SUCCESS) {
    return NULL;
  }
  // Tag responds with (size per transmission, num_transitions)
  uint8_t size_num[2];
  err_code = nrf_drv_spi_transfer(spi, NULL, 0, size_num, 2);
  while(size_num[0] == 0x00) {
    APP_ERROR_CHECK(err_code);
    if (err_code != NRF_SUCCESS) {
      return NULL;
    }
    nrf_delay_ms(10);
    err_code = nrf_drv_spi_transfer(spi, NULL, 0, size_num, 2);
  }
  printf("%x %x\n", size_num[0], size_num[1]);
  // Reading data from tag
  uint8_t* readData = (uint8_t *)malloc(sizeof(uint8_t)*size_num[0]);
  err_code = nrf_drv_spi_transfer(spi, NULL, 0, readData, size_num[0]);
  APP_ERROR_CHECK(err_code);
  if (err_code != NRF_SUCCESS) {
    return NULL;
  }
  int i = 0;
  while (i < size_num[0]) {
    printf("%x ", readData[i++]);
  }
  printf("\n");
  return readData;
}
