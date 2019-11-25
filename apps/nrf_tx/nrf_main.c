// LoRa 9x_TX
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example LoRa9x_RX
 
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <pthread.h>
#include <time.h>

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

#define RH_RF95_FIFO_SIZE 255
#define RH_RF95_MAX_PAYLOAD_LEN RH_RF95_FIFO_SIZE

#define RH_RF95_HEADER_LEN 4
#define RH_RF95_REG_00_FIFO                                0x00
#define RH_RF95_REG_01_OP_MODE                             0x01
#define RH_RF95_REG_40_DIO_MAPPING1                        0x40
#define RH_RF95_MODE_TX                               0x03
#define RH_RF95_MODE_STDBY                            0x01
#define RH_RF95_REG_22_PAYLOAD_LENGTH                      0x22
#define RH_RF95_REG_0D_FIFO_ADDR_PTR                       0x0d
#define RH_RF95_MAX_MESSAGE_LEN (RH_RF95_MAX_PAYLOAD_LEN - RH_RF95_HEADER_LEN)
#define RH_BROADCAST_ADDRESS 0xff
#define RH_RF95_MAX_POWER                             0x70
#define RH_RF95_REG_09_PA_CONFIG                           0x09
#define RH_RF95_PA_DAC_ENABLE                         0x07
#define RH_RF95_REG_4D_PA_DAC                              0x4d
#define RH_RF95_PA_DAC_DISABLE                        0x04
#define RH_RF95_PA_SELECT                             0x80
#define RH_RF95_REG_08_FRF_LSB                             0x08
#define RH_RF95_REG_07_FRF_MID                             0x07
#define RH_RF95_REG_06_FRF_MSB                             0x06

#define RH_RF95_FXOSC 32000000.0
#define RH_RF95_FSTEP  (RH_RF95_FXOSC / 524288)
#define RH_NRF51_HEADER_LEN 7

// This is the maximum number of bytes that can be carried by the nRF51.
// We use some for headers, keeping fewer for RadioHead messages
#define RH_NRF51_MAX_PAYLOAD_LEN 254


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0


typedef enum {
  RHModeInitialising = 0, ///< Transport is initialising. Initial default value until init() is called..
  RHModeSleep,            ///< Transport hardware is in low power sleep mode (if supported)
  RHModeIdle,             ///< Transport is idle.
  RHModeTx,               ///< Transport is in the process of transmitting a message.
  RHModeRx,               ///< Transport is in the process of receiving a message.
  RHModeCad               ///< Transport is in the process of detecting channel activity (if supported)
} RHMode;

typedef enum {
  DataRate1Mbps = 0,   ///< 1 Mbps
  DataRate2Mbps,       ///< 2 Mbps
  DataRate250kbps      ///< 250 kbps
} DataRate;

/// \brief Convenient values for setting transmitter power in setRF()
typedef enum {
  // Add 20dBm for nRF24L01p with PA and LNA modules
  TransmitPower4dBm = 0,        ///<  4 dBm
  TransmitPower0dBm,            ///<  0 dBm
  TransmitPowerm4dBm,           ///< -4 dBm
  TransmitPowerm8dBm,           ///< -8 dBm
  TransmitPowerm12dBm,          ///< -12 dBm
  TransmitPowerm16dBm,          ///< -16 dBm
  TransmitPowerm20dBm,          ///< -20 dBm
  TransmitPowerm30dBm,          ///< -30 dBm
} TransmitPower;

volatile RHMode     _mode;
nrf_drv_spi_config_t spi_config;
uint8_t _txHeaderFlags = 0;
uint8_t _txHeaderId = 0;
uint8_t _txHeaderTo = RH_BROADCAST_ADDRESS;
uint8_t _txHeaderFrom = RH_BROADCAST_ADDRESS;
bool _usingHFport = true;
/// TO header in the last received mesasge
volatile uint8_t    _rxHeaderTo;
/// FROM header in the last received mesasge
volatile uint8_t    _rxHeaderFrom;
/// ID header in the last received mesasge
volatile uint8_t    _rxHeaderId;
/// FLAGS header in the last received mesasge
volatile uint8_t    _rxHeaderFlags;
/// Whether the transport is in promiscuous mode
bool                _promiscuous;
/// Count of the number of successfully transmitted messaged
volatile uint16_t   _rxGood;
/// This node id
uint8_t             _thisAddress;
/// Count of the number of bad messages (eg bad checksum etc) received
volatile uint16_t   _rxBad;
uint8_t             _buf[RH_NRF51_MAX_PAYLOAD_LEN+1];
/// True when there is a valid message in the buffer
volatile bool       _rxBufValid;


static nrf_drv_spi_t instance = NRF_DRV_SPI_INSTANCE(1);
static const nrf_drv_spi_t* spi_instance;



void clearRxBuf() {
    _rxBufValid = false;
    _buf[1] = 0;
}

// writes one uint8_t value
void spiWrite(uint8_t reg, uint8_t val) {
  uint8_t buf[2];
  buf[0] = 0x80 | reg;
  memcpy(buf+1, &val, 1);
  nrf_drv_spi_init(spi_instance, &spi_config, NULL, NULL);
  nrf_drv_spi_transfer(spi_instance, buf, 1, NULL, 0);
  nrf_drv_spi_uninit(spi_instance);
}

void setModeIdle() {
  if (_mode != RHModeIdle) {
    spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
    _mode = RHModeIdle;
  }
}

void setModeRx() {
  if (_mode != RHModeRx) {
    setModeIdle(); // Can only start RX from DISABLE state

    // Radio will transition automatically to Disable state when a message is received
    NRF_RADIO->PACKETPTR = (uint32_t)_buf;
    NRF_RADIO->EVENTS_READY = 0U;
    NRF_RADIO->TASKS_RXEN = 1;
    NRF_RADIO->EVENTS_END = 0U; // So we can detect end of reception
    _mode = RHModeRx;
  }
}

// Check whether the latest received message is complete and uncorrupted
void validateRxBuf() {
  if (_buf[1] < RH_NRF51_HEADER_LEN)
    return; // Too short to be a real message
  // Extract the 4 headers following S0, LEN and S1
  _rxHeaderTo    = _buf[3];
  _rxHeaderFrom  = _buf[4];
  _rxHeaderId    = _buf[5];
  _rxHeaderFlags = _buf[6];

  if (_promiscuous || _rxHeaderTo == _thisAddress || _rxHeaderTo == RH_BROADCAST_ADDRESS) {
    _rxGood++;
    _rxBufValid = true;
  }
}


bool setChannel(uint8_t channel) {
  NRF_RADIO->FREQUENCY = ((channel << RADIO_FREQUENCY_FREQUENCY_Pos) & RADIO_FREQUENCY_FREQUENCY_Msk);
  return true;
}

bool setNetworkAddress(uint8_t* address, uint8_t len) {
  if (len < 3 || len > 5)
    return false;

  // First byte is the prefix, remainder are base
  NRF_RADIO->PREFIX0    = ((address[0] << RADIO_PREFIX0_AP0_Pos) & RADIO_PREFIX0_AP0_Msk);
  uint32_t base;
  memcpy(&base, address+1, len-1);
  NRF_RADIO->BASE0 = base;

  NRF_RADIO->PCNF1 =  (
    (((sizeof(_buf)) << RADIO_PCNF1_MAXLEN_Pos)  & RADIO_PCNF1_MAXLEN_Msk)  // maximum length of payload
    | (((0UL)        << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk) // expand the payload with 0 bytes
    | (((len-1)      << RADIO_PCNF1_BALEN_Pos)   & RADIO_PCNF1_BALEN_Msk)); // base address length in number of bytes.

  return true;
}

bool available() {
  if (!_rxBufValid) {
    if (_mode == RHModeTx)
      return false;
    setModeRx();
    if (!NRF_RADIO->EVENTS_END)
      return false; // No message yet
    setModeIdle();

    if (!NRF_RADIO->CRCSTATUS) {
      // Bad CRC, restart the radio     
      _rxBad++;
      setModeRx();
      return false;
    }
    validateRxBuf(); 
    if (!_rxBufValid)
      setModeRx(); // Try for another
    }
  return _rxBufValid;
}

bool recv(uint8_t* buf, uint8_t* len) {
  if (!available())
    return false;
  if (buf && len) {
    // Skip the 4 headers that are at the beginning of the rxBuf
    // the payload length is the first octet in _buf
    if (*len > _buf[1]-RH_NRF51_HEADER_LEN)
      *len = _buf[1]-RH_NRF51_HEADER_LEN;
    memcpy(buf, _buf+RH_NRF51_HEADER_LEN, *len);
  }
  clearRxBuf(); // This message accepted and cleared
  return true;
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

void setTxPower(int8_t power, bool useRFO) {
  // Sigh, different behaviours depending on whther the module use PA_BOOST or the RFO pin
  // for the transmitter output
  if (useRFO) {
    if (power > 14)
      power = 14;
    if (power < -1)
      power = -1;
    spiWrite(RH_RF95_REG_09_PA_CONFIG, RH_RF95_MAX_POWER | (power + 1));
  } else {
    if (power > 23)
      power = 23;
    if (power < 5)
      power = 5;
    // For RH_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
    // RH_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will us it
    // for 21, 22 and 23dBm
    if (power > 20) {
      spiWrite(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_ENABLE);
      power -= 3;
    } else {
      spiWrite(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE);
    }
    // RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
    // pin is connected, so must use PA_BOOST
    // Pout = 2 + OutputPower.
    // The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
    // but OutputPower claims it would be 17dBm.
    // My measurements show 20dBm is correct
    spiWrite(RH_RF95_REG_09_PA_CONFIG, RH_RF95_PA_SELECT | (power-5));
  }
}

bool setFrequency(float centre) {
  // Frf = FRF / FSTEP
  uint32_t frf = (centre * 1000000.0) / RH_RF95_FSTEP;
  spiWrite(RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
  spiWrite(RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
  spiWrite(RH_RF95_REG_08_FRF_LSB, frf & 0xff);
  _usingHFport = (centre >= 779.0);

  return true;
}

// bool waitPacketSent() {
//   while (_mode == RHModeTx)
//     pthread_yield(); // Wait for any previous transmit to finish
//   return true;
// }
  
bool setRF(DataRate data_rate, TransmitPower power) {
  uint8_t mode;
  uint8_t p;

  if (data_rate == DataRate2Mbps)
    mode = RADIO_MODE_MODE_Nrf_2Mbit;
  else if (data_rate == DataRate1Mbps)
    mode = RADIO_MODE_MODE_Nrf_1Mbit;
  else if (data_rate == DataRate250kbps)
    mode = RADIO_MODE_MODE_Nrf_250Kbit;
  else
    return false;// Invalid

  if      (power == TransmitPower4dBm)
    p = RADIO_TXPOWER_TXPOWER_Pos4dBm;
  else if (power == TransmitPower0dBm)
    p = RADIO_TXPOWER_TXPOWER_0dBm;
  else if (power == TransmitPowerm4dBm)
    p = RADIO_TXPOWER_TXPOWER_Neg4dBm;
  else if (power == TransmitPowerm8dBm)
    p = RADIO_TXPOWER_TXPOWER_Neg8dBm;
  else if (power == TransmitPowerm12dBm)
    p = RADIO_TXPOWER_TXPOWER_Neg12dBm;
  else if (power == TransmitPowerm16dBm)
    p = RADIO_TXPOWER_TXPOWER_Neg16dBm;
  else if (power == TransmitPowerm20dBm)
    p = RADIO_TXPOWER_TXPOWER_Neg20dBm;
  else if (power == TransmitPowerm30dBm)
    p = RADIO_TXPOWER_TXPOWER_Neg30dBm;
  else
    return false; // Invalid


  NRF_RADIO->TXPOWER = ((p << RADIO_TXPOWER_TXPOWER_Pos) & RADIO_TXPOWER_TXPOWER_Msk);
  NRF_RADIO->MODE    = ((mode << RADIO_MODE_MODE_Pos) & RADIO_MODE_MODE_Msk);

  return true;
}

bool init() {
    // Enable the High Frequency clock to the system as a whole
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
    
    // Disable and reset the radio
    NRF_RADIO->POWER = RADIO_POWER_POWER_Disabled;
    NRF_RADIO->POWER = RADIO_POWER_POWER_Enabled;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE   = 1;
    // Wait until we are in DISABLE state
    while (NRF_RADIO->EVENTS_DISABLED == 0) {}

    // Physical on-air address is set in PREFIX0 + BASE0 by setNetworkAddress
    NRF_RADIO->TXADDRESS    = 0x00; // Use logical address 0 (PREFIX0 + BASE0)
    NRF_RADIO->RXADDRESSES  = 0x01; // Enable reception on logical address 0 (PREFIX0 + BASE0)

    // Configure the CRC
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Number of checksum bits
    NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value      
    NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1

    // These shorts will make the radio transition from Ready to Start to Disable automatically
    // for both TX and RX, which makes for much shorter on-air times
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos)
                | (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);

    NRF_RADIO->PCNF0 =   (8 << RADIO_PCNF0_LFLEN_Pos) // Payload size length in bits
                 | (1 << RADIO_PCNF0_S0LEN_Pos) // S0 is 1 octet
                 | (8 << RADIO_PCNF0_S1LEN_Pos); // S1 is 1 octet

    // Make sure we are powered down
    setModeIdle();

    // Set a default network address
    uint8_t default_network_address[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    setNetworkAddress(default_network_address, sizeof(default_network_address));

    setChannel(2); // The default, in case it was set by another app without powering down
    setRF(DataRate2Mbps, TransmitPower0dBm);
    return true;
}

void setup() {
  nrf_gpio_pin_dir_set(RFM95_RST, NRF_GPIO_PIN_DIR_OUTPUT);
  //digitalWrite(RFM95_RST, HIGH);
  nrf_gpio_pin_write(RFM95_RST, 1);
  printf("Arduino LoRa TX Test!");
  // manual reset
  nrf_gpio_pin_write(RFM95_RST, 0);
  nrf_delay_ms(10);
  nrf_gpio_pin_write(RFM95_RST, 1);
  nrf_delay_ms(10);
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
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  setTxPower(23, false);
}
 
int16_t packetnum = 0;  // packet counter, we increment per xmission

bool send(const uint8_t* data, uint8_t len) {
  if (len > RH_RF95_MAX_MESSAGE_LEN)
    return false;

  //waitPacketSent(); // Make sure we dont interrupt an outgoing message
  setModeIdle();

  // Position at the beginning of the FIFO
  spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);
  // The headers
  spiWrite(RH_RF95_REG_00_FIFO, _txHeaderTo);
  spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFrom);
  spiWrite(RH_RF95_REG_00_FIFO, _txHeaderId);
  spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFlags);
  // The message data
  uint8_t tdata = *data;
  spiBurstWrite(RH_RF95_REG_00_FIFO, &tdata, len);
  spiWrite(RH_RF95_REG_22_PAYLOAD_LENGTH, len + RH_RF95_HEADER_LEN);

  setModeTx(); // Start the transmitter
  // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
  return true;
}

bool waitAvailableTimeout(uint16_t timeout) {
  unsigned long starttime = clock();
  while ((clock() - starttime) < timeout) {
    if (available()) {
      return true;
    }
    //pthread_yield();  
  }
  return false;
}

void loop() {
  printf("Sending to rf95_server");
  // Send a message to rf95_server
  
  char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket+13, 10);
  radiopacket[19] = 0;
  
  printf("Sending...");
  nrf_delay_ms(10);
  send((uint8_t *)radiopacket, 20);
 
  printf("Waiting for packet to complete...");
  nrf_delay_ms(500);
  //nrf_delay_ms(10);
  // TODO: Do we need this?
  // waitPacketSent();

  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
 
  printf("Waiting for reply...");
  nrf_delay_ms(10);
  if (waitAvailableTimeout(1000)) { 
    // Should be a reply message for us now   
    if (recv(buf, &len)) {
      printf("Got reply");
    }
    else {
      printf("Receive failed");
    }
  }
  else {
    printf("No reply, is there a listener around?");
  }
  nrf_delay_ms(1000);
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;
  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

    // initialize LED
  nrf_gpio_pin_dir_set(7, NRF_GPIO_PIN_DIR_OUTPUT);

  spi_instance = &instance;

  nrf_drv_spi_config_t config = {
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

  spi_config = config;

  nrf_gpio_cfg_output(RTC_WDI);
  nrf_gpio_pin_set(RTC_WDI);

  setup();

  while (1) {
    loop();
  }
}

