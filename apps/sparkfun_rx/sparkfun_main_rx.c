// LoRa 9x_RX
// Example sketch showing how to create a simple messaging client (receiver)
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
#include "nrf_drv_gpiote.h"



// Register names (LoRa Mode, from table 85)
#define RH_RF95_REG_00_FIFO                                0x00
#define RH_RF95_REG_01_OP_MODE                             0x01
#define RH_RF95_REG_02_RESERVED                            0x02
#define RH_RF95_REG_03_RESERVED                            0x03
#define RH_RF95_REG_04_RESERVED                            0x04
#define RH_RF95_REG_05_RESERVED                            0x05
#define RH_RF95_REG_06_FRF_MSB                             0x06
#define RH_RF95_REG_07_FRF_MID                             0x07
#define RH_RF95_REG_08_FRF_LSB                             0x08
#define RH_RF95_REG_09_PA_CONFIG                           0x09
#define RH_RF95_REG_0A_PA_RAMP                             0x0a
#define RH_RF95_REG_0B_OCP                                 0x0b
#define RH_RF95_REG_0C_LNA                                 0x0c
#define RH_RF95_REG_0D_FIFO_ADDR_PTR                       0x0d
#define RH_RF95_REG_0E_FIFO_TX_BASE_ADDR                   0x0e
#define RH_RF95_REG_0F_FIFO_RX_BASE_ADDR                   0x0f
#define RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR                0x10
#define RH_RF95_REG_11_IRQ_FLAGS_MASK                      0x11
#define RH_RF95_REG_12_IRQ_FLAGS                           0x12
#define RH_RF95_REG_13_RX_NB_BYTES                         0x13
#define RH_RF95_REG_14_RX_HEADER_CNT_VALUE_MSB             0x14
#define RH_RF95_REG_15_RX_HEADER_CNT_VALUE_LSB             0x15
#define RH_RF95_REG_16_RX_PACKET_CNT_VALUE_MSB             0x16
#define RH_RF95_REG_17_RX_PACKET_CNT_VALUE_LSB             0x17
#define RH_RF95_REG_18_MODEM_STAT                          0x18
#define RH_RF95_REG_19_PKT_SNR_VALUE                       0x19
#define RH_RF95_REG_1A_PKT_RSSI_VALUE                      0x1a
#define RH_RF95_REG_1B_RSSI_VALUE                          0x1b
#define RH_RF95_REG_1C_HOP_CHANNEL                         0x1c
#define RH_RF95_REG_1D_MODEM_CONFIG1                       0x1d
#define RH_RF95_REG_1E_MODEM_CONFIG2                       0x1e
#define RH_RF95_REG_1F_SYMB_TIMEOUT_LSB                    0x1f
#define RH_RF95_REG_20_PREAMBLE_MSB                        0x20
#define RH_RF95_REG_21_PREAMBLE_LSB                        0x21
#define RH_RF95_REG_22_PAYLOAD_LENGTH                      0x22
#define RH_RF95_REG_23_MAX_PAYLOAD_LENGTH                  0x23
#define RH_RF95_REG_24_HOP_PERIOD                          0x24
#define RH_RF95_REG_25_FIFO_RX_BYTE_ADDR                   0x25
#define RH_RF95_REG_26_MODEM_CONFIG3                       0x26

#define RH_RF95_REG_27_PPM_CORRECTION                      0x27
#define RH_RF95_REG_28_FEI_MSB                             0x28
#define RH_RF95_REG_29_FEI_MID                             0x29
#define RH_RF95_REG_2A_FEI_LSB                             0x2a
#define RH_RF95_REG_2C_RSSI_WIDEBAND                       0x2c
#define RH_RF95_REG_31_DETECT_OPTIMIZE                     0x31
#define RH_RF95_REG_33_INVERT_IQ                           0x33
#define RH_RF95_REG_37_DETECTION_THRESHOLD                 0x37
#define RH_RF95_REG_39_SYNC_WORD                           0x39

#define RH_RF95_REG_40_DIO_MAPPING1                        0x40
#define RH_RF95_REG_41_DIO_MAPPING2                        0x41
#define RH_RF95_REG_42_VERSION                             0x42

#define RH_RF95_REG_4B_TCXO                                0x4b
#define RH_RF95_REG_4D_PA_DAC                              0x4d
#define RH_RF95_REG_5B_FORMER_TEMP                         0x5b
#define RH_RF95_REG_61_AGC_REF                             0x61
#define RH_RF95_REG_62_AGC_THRESH1                         0x62
#define RH_RF95_REG_63_AGC_THRESH2                         0x63
#define RH_RF95_REG_64_AGC_THRESH3                         0x64

// RH_RF95_REG_01_OP_MODE                             0x01
#define RH_RF95_LONG_RANGE_MODE                       0x80
#define RH_RF95_ACCESS_SHARED_REG                     0x40
#define RH_RF95_LOW_FREQUENCY_MODE                    0x08
#define RH_RF95_MODE                                  0x07
#define RH_RF95_MODE_SLEEP                            0x00
#define RH_RF95_MODE_STDBY                            0x01
#define RH_RF95_MODE_FSTX                             0x02
#define RH_RF95_MODE_TX                               0x03
#define RH_RF95_MODE_FSRX                             0x04
#define RH_RF95_MODE_RXCONTINUOUS                     0x05
#define RH_RF95_MODE_RXSINGLE                         0x06
#define RH_RF95_MODE_CAD                              0x07

// RH_RF95_REG_09_PA_CONFIG                           0x09
#define RH_RF95_PA_SELECT                             0x80
#define RH_RF95_MAX_POWER                             0x70
#define RH_RF95_OUTPUT_POWER                          0x0f

// RH_RF95_REG_0A_PA_RAMP                             0x0a
#define RH_RF95_LOW_PN_TX_PLL_OFF                     0x10
#define RH_RF95_PA_RAMP                               0x0f
#define RH_RF95_PA_RAMP_3_4MS                         0x00
#define RH_RF95_PA_RAMP_2MS                           0x01
#define RH_RF95_PA_RAMP_1MS                           0x02
#define RH_RF95_PA_RAMP_500US                         0x03
#define RH_RF95_PA_RAMP_250US                         0x04
#define RH_RF95_PA_RAMP_125US                         0x05
#define RH_RF95_PA_RAMP_100US                         0x06
#define RH_RF95_PA_RAMP_62US                          0x07
#define RH_RF95_PA_RAMP_50US                          0x08
#define RH_RF95_PA_RAMP_40US                          0x09
#define RH_RF95_PA_RAMP_31US                          0x0a
#define RH_RF95_PA_RAMP_25US                          0x0b
#define RH_RF95_PA_RAMP_20US                          0x0c
#define RH_RF95_PA_RAMP_15US                          0x0d
#define RH_RF95_PA_RAMP_12US                          0x0e
#define RH_RF95_PA_RAMP_10US                          0x0f

// RH_RF95_REG_0B_OCP                                 0x0b
#define RH_RF95_OCP_ON                                0x20
#define RH_RF95_OCP_TRIM                              0x1f

// RH_RF95_REG_0C_LNA                                 0x0c
#define RH_RF95_LNA_GAIN                              0xe0
#define RH_RF95_LNA_GAIN_G1                           0x20
#define RH_RF95_LNA_GAIN_G2                           0x40
#define RH_RF95_LNA_GAIN_G3                           0x60                
#define RH_RF95_LNA_GAIN_G4                           0x80
#define RH_RF95_LNA_GAIN_G5                           0xa0
#define RH_RF95_LNA_GAIN_G6                           0xc0
#define RH_RF95_LNA_BOOST_LF                          0x18
#define RH_RF95_LNA_BOOST_LF_DEFAULT                  0x00
#define RH_RF95_LNA_BOOST_HF                          0x03
#define RH_RF95_LNA_BOOST_HF_DEFAULT                  0x00
#define RH_RF95_LNA_BOOST_HF_150PC                    0x03

// RH_RF95_REG_11_IRQ_FLAGS_MASK                      0x11
#define RH_RF95_RX_TIMEOUT_MASK                       0x80
#define RH_RF95_RX_DONE_MASK                          0x40
#define RH_RF95_PAYLOAD_CRC_ERROR_MASK                0x20
#define RH_RF95_VALID_HEADER_MASK                     0x10
#define RH_RF95_TX_DONE_MASK                          0x08
#define RH_RF95_CAD_DONE_MASK                         0x04
#define RH_RF95_FHSS_CHANGE_CHANNEL_MASK              0x02
#define RH_RF95_CAD_DETECTED_MASK                     0x01

// RH_RF95_REG_12_IRQ_FLAGS                           0x12
#define RH_RF95_RX_TIMEOUT                            0x80
#define RH_RF95_RX_DONE                               0x40
#define RH_RF95_PAYLOAD_CRC_ERROR                     0x20
#define RH_RF95_VALID_HEADER                          0x10
#define RH_RF95_TX_DONE                               0x08
#define RH_RF95_CAD_DONE                              0x04
#define RH_RF95_FHSS_CHANGE_CHANNEL                   0x02
#define RH_RF95_CAD_DETECTED                          0x01

// RH_RF95_REG_18_MODEM_STAT                          0x18
#define RH_RF95_RX_CODING_RATE                        0xe0
#define RH_RF95_MODEM_STATUS_CLEAR                    0x10
#define RH_RF95_MODEM_STATUS_HEADER_INFO_VALID        0x08
#define RH_RF95_MODEM_STATUS_RX_ONGOING               0x04
#define RH_RF95_MODEM_STATUS_SIGNAL_SYNCHRONIZED      0x02
#define RH_RF95_MODEM_STATUS_SIGNAL_DETECTED          0x01

// RH_RF95_REG_1C_HOP_CHANNEL                         0x1c
#define RH_RF95_PLL_TIMEOUT                           0x80
#define RH_RF95_RX_PAYLOAD_CRC_IS_ON                  0x40
#define RH_RF95_FHSS_PRESENT_CHANNEL                  0x3f

// RH_RF95_REG_1D_MODEM_CONFIG1                       0x1d
#define RH_RF95_BW                                    0xf0

#define RH_RF95_BW_7_8KHZ                             0x00
#define RH_RF95_BW_10_4KHZ                            0x10
#define RH_RF95_BW_15_6KHZ                            0x20
#define RH_RF95_BW_20_8KHZ                            0x30
#define RH_RF95_BW_31_25KHZ                           0x40
#define RH_RF95_BW_41_7KHZ                            0x50
#define RH_RF95_BW_62_5KHZ                            0x60
#define RH_RF95_BW_125KHZ                             0x70
#define RH_RF95_BW_250KHZ                             0x80
#define RH_RF95_BW_500KHZ                             0x90
#define RH_RF95_CODING_RATE                           0x0e
#define RH_RF95_CODING_RATE_4_5                       0x02
#define RH_RF95_CODING_RATE_4_6                       0x04
#define RH_RF95_CODING_RATE_4_7                       0x06
#define RH_RF95_CODING_RATE_4_8                       0x08
#define RH_RF95_IMPLICIT_HEADER_MODE_ON               0x01

// RH_RF95_REG_1E_MODEM_CONFIG2                       0x1e
#define RH_RF95_SPREADING_FACTOR                      0xf0
#define RH_RF95_SPREADING_FACTOR_64CPS                0x60
#define RH_RF95_SPREADING_FACTOR_128CPS               0x70
#define RH_RF95_SPREADING_FACTOR_256CPS               0x80
#define RH_RF95_SPREADING_FACTOR_512CPS               0x90
#define RH_RF95_SPREADING_FACTOR_1024CPS              0xa0
#define RH_RF95_SPREADING_FACTOR_2048CPS              0xb0
#define RH_RF95_SPREADING_FACTOR_4096CPS              0xc0
#define RH_RF95_TX_CONTINUOUS_MODE                    0x08

#define RH_RF95_PAYLOAD_CRC_ON                        0x04
#define RH_RF95_SYM_TIMEOUT_MSB                       0x03

// RH_RF95_REG_26_MODEM_CONFIG3
#define RH_RF95_MOBILE_NODE                           0x08 // HopeRF term
#define RH_RF95_LOW_DATA_RATE_OPTIMIZE                0x08 // Semtechs term
#define RH_RF95_AGC_AUTO_ON                           0x04

// RH_RF95_REG_4B_TCXO                                0x4b
#define RH_RF95_TCXO_TCXO_INPUT_ON                    0x10

// RH_RF95_REG_4D_PA_DAC                              0x4d
#define RH_RF95_PA_DAC_DISABLE                        0x04
#define RH_RF95_PA_DAC_ENABLE                         0x07


// SPI
// LoRa Module pin connections
// Reset
#define RFM95_RST   NRF_GPIO_PIN_MAP(0,15)
// Slave/chip select
#define RFM95_CS    NRF_GPIO_PIN_MAP(0,13)
// Master out, slave in
#define SPI_MOSI    NRF_GPIO_PIN_MAP(0,12)
// Master in, slave out
#define SPI_MISO    NRF_GPIO_PIN_MAP(0,11)
// Clock
#define SPI_SCLK    NRF_GPIO_PIN_MAP(0,10)
// Interrupt
#define RFM95_INT   NRF_GPIO_PIN_MAP(0, 9)


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
#define RH_RF95_REG_0E_FIFO_TX_BASE_ADDR                   0x0e
#define RH_RF95_LONG_RANGE_MODE                       0x80
static uint8_t LED = NRF_GPIO_PIN_MAP(0,7);



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

volatile RHMode     _mode;
nrf_drv_spi_config_t spi_config;
uint8_t _txHeaderFlags = 0;
volatile uint16_t   _txGood;
volatile uint8_t    _bufLen;
// Last measured SNR, dB
int8_t              _lastSNR; 
volatile int16_t     _lastRssi;
uint8_t _txHeaderId = 0;
uint8_t _txHeaderTo = RH_BROADCAST_ADDRESS;
uint8_t _txHeaderFrom = RH_BROADCAST_ADDRESS;
bool _usingHFport = true;
  
/// Channel activity detected
volatile bool       _cad;
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
  printf("\n%u\n", reg);
  printf("\n%u\n", val);
  uint8_t buf[257];
  buf[0] = 0x80 | reg;
  memcpy(buf+1, &val, 1);
  nrf_drv_spi_init(spi_instance, &spi_config, NULL, NULL);
  printf("\n%x\n", buf);
  nrf_drv_spi_transfer(spi_instance, buf, 2, NULL, 0);
  printf("init\n");
  nrf_drv_spi_uninit(spi_instance);
  printf("init\n");
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

//bool RH_RF95::available()
bool available()
{
    if (_mode == RHModeTx)
  return false;
    setModeRx();
    return _rxBufValid; // Will be set by the interrupt handler when a good message is received
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

//void RH_RF95::setTxPower(int8_t power, bool useRFO)
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

bool waitPacketSent() {
  while (_mode == RHModeTx)
    ;
    //pthread_yield(); // Wait for any previous transmit to finish
  return true;
}

void setPreambleLength(uint16_t bytes) {
    spiWrite(RH_RF95_REG_20_PREAMBLE_MSB, bytes >> 8);
    spiWrite(RH_RF95_REG_21_PREAMBLE_LSB, bytes & 0xff);
}

//bool RH_RF95::init()
bool init() {

	nrf_gpio_pin_dir_set(RFM95_CS, NRF_GPIO_PIN_DIR_OUTPUT);
  	nrf_gpio_pin_write(RFM95_CS, 1);
     // Set sleep mode, so we can also set LORA mode:
    printf("spiWrite");
    spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE);
    nrf_delay_ms(10); // Wait for sleep mode to take over from say, CAD
    // Check we are in sleep mode, with LORA set
    if (spiRead(RH_RF95_REG_01_OP_MODE) != (RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE)) {
      return false; // No device present?
    }
    // Set up FIFO
    // We configure so that we can use the entire 256 byte FIFO for either receive
    // or transmit, but not both at the same time
    spiWrite(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
    spiWrite(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);

    // Packet format is preamble + explicit-header + payload + crc
    // Explicit Header Mode
    // payload is TO + FROM + ID + FLAGS + message data
    // RX mode is implmented with RXCONTINUOUS
    // max message data length is 255 - 4 = 251 octets
    printf("setModeIdle");
    setModeIdle();

    // Set up default configuration
    // No Sync Words in LORA mode.

    //setModemConfig(0); // Radio default
    //setModemConfig(Bw125Cr48Sf4096); // slow and reliable?
    setPreambleLength(8); // Default is 8
    // An innocuous ISM frequency, same as RF22's
    setFrequency(915.0);
    // Lowish power
    setTxPower(13, false);

    return true;
}

void handleInterrupts(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  // Read the interrupt register
  uint8_t irq_flags = spiRead(RH_RF95_REG_12_IRQ_FLAGS);
  // Read the RegHopChannel register to check if CRC presence is signalled
  // in the header. If not it might be a stray (noise) packet.*
  //uint8_t crc_present = spiRead(RH_RF95_REG_1C_HOP_CHANNEL);

  if (_mode == RHModeRx && irq_flags & RH_RF95_RX_DONE) {
    // Have received a packet
    uint8_t len = spiRead(RH_RF95_REG_13_RX_NB_BYTES);

    //  Reset the fifo read ptr to the beginning of the packet
    spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, spiRead(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
    spiBurstRead(RH_RF95_REG_00_FIFO, _buf, len);
    _bufLen = len;
    spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

    // We have received a message.
    validateRxBuf(); 
    if (_rxBufValid)
      setModeIdle(); // Got one 
  } else if (_mode == RHModeTx && irq_flags & RH_RF95_TX_DONE) {
    _txGood++;
    setModeIdle();
  } else if (_mode == RHModeCad && irq_flags & RH_RF95_CAD_DONE) {
    _cad = irq_flags & RH_RF95_CAD_DETECTED;
    setModeIdle();
  }
    // Sigh: on some processors, for some unknown reason, doing this only once does not actually
    // clear the radio's interrupt flag. So we do it twice. Why?
    spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
    spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
}


// // C++ level interrupt handler for this instance
// // LORA is unusual in that it has several interrupt lines, and not a single, combined one.
// // On MiniWirelessLoRa, only one of the several interrupt lines (DI0) from the RFM95 is usefuly 
// // connnected to the processor.
// // We use this to get RxDone and TxDone interrupts
// void handleInterrupt() {
//     // Read the interrupt register
//     uint8_t irq_flags = spiRead(RH_RF95_REG_12_IRQ_FLAGS);
//     // Read the RegHopChannel register to check if CRC presence is signalled
//     // in the header. If not it might be a stray (noise) packet.*
//     uint8_t crc_present = spiRead(RH_RF95_REG_1C_HOP_CHANNEL);

//     if (_mode == RHModeRx
//   && ((irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR))
//       | !(crc_present & RH_RF95_RX_PAYLOAD_CRC_IS_ON)))
// //    if (_mode == RHModeRx && irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR))
//     {
//   _rxBad++;
//     }
//     else if (_mode == RHModeRx && irq_flags & RH_RF95_RX_DONE)
//     {
//   // Have received a packet
//   uint8_t len = spiRead(RH_RF95_REG_13_RX_NB_BYTES);

//   // Reset the fifo read ptr to the beginning of the packet
//   spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, spiRead(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
//   spiBurstRead(RH_RF95_REG_00_FIFO, _buf, len);
//   _bufLen = len;
//   spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

//   // Remember the last signal to noise ratio, LORA mode
//   // Per page 111, SX1276/77/78/79 datasheet
//   _lastSNR = (int8_t)spiRead(RH_RF95_REG_19_PKT_SNR_VALUE) / 4;

//   // Remember the RSSI of this packet, LORA mode
//   // this is according to the doc, but is it really correct?
//   // weakest receiveable signals are reported RSSI at about -66
//   _lastRssi = spiRead(RH_RF95_REG_1A_PKT_RSSI_VALUE);
//   // Adjust the RSSI, datasheet page 87
//   if (_lastSNR < 0)
//       _lastRssi = _lastRssi + _lastSNR;
//   else
//       _lastRssi = (int)_lastRssi * 16 / 15;
//   if (_usingHFport)
//       _lastRssi -= 157;
//   else
//       _lastRssi -= 164;
      
//   // We have received a message.
//   validateRxBuf(); 
//   if (_rxBufValid)
//       setModeIdle(); // Got one 
//     }
//     else if (_mode == RHModeTx && irq_flags & RH_RF95_TX_DONE)
//     {
//   _txGood++;
//   setModeIdle();
//     }
//     else if (_mode == RHModeCad && irq_flags & RH_RF95_CAD_DONE)
//     {
//         _cad = irq_flags & RH_RF95_CAD_DETECTED;
//         setModeIdle();
//     }
//     // Sigh: on some processors, for some unknown reason, doing this only once does not actually
//     // clear the radio's interrupt flag. So we do it twice. Why?
//     spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
//     spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
// }
 
int16_t packetnum = 0;  // packet counter, we increment per xmission

bool send(const uint8_t* data, uint8_t len) {
  if (len > RH_RF95_MAX_MESSAGE_LEN)
    return false;

  waitPacketSent(); // Make sure we dont interrupt an outgoing message
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
 if (available()) {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    buf[len - 1] = 0;
    
    if (recv(buf, &len)) {
      nrf_gpio_pin_clear(LED);
      printf("Got something");
      printf("\n");
      // Send a reply
      uint8_t data[] = "And hello back to you";
      send(data, sizeof(data));
      waitPacketSent();
      printf("Sent a reply");
      nrf_gpio_pin_set(LED);
    } else {
      printf("Receive failed");
    }
  }
}

static void gpio_init(void) {
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(RFM95_INT, &in_config, handleInterrupts);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(RFM95_INT, true);
}

int main(void) {
  // initialize RTT library
  ret_code_t error_code = NRF_SUCCESS;
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // initialize GPIO driver/interrupts
  gpio_init();

  // manually-controlled (simple) output, initially set
  nrfx_gpiote_out_config_t out_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
  error_code = nrfx_gpiote_out_init(LED, &out_config);
  APP_ERROR_CHECK(error_code);

  spi_instance = &instance;

  nrf_drv_spi_config_t config = {
    .sck_pin = SPI_SCLK,
    .mosi_pin = SPI_MOSI,
    .miso_pin = SPI_MISO,
    .ss_pin = RFM95_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_1M,
    .mode = NRF_DRV_SPI_MODE_1,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };

  spi_config = config;

  nrf_gpio_pin_dir_set(RFM95_RST, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_write(RFM95_RST, 1);
  printf("Arduino LoRa RX Test!");
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

  if (!setFrequency(RF95_FREQ)) {
    printf("setFrequency failed");
    while (1);
  }

  setTxPower(23, false);

  while (1) {
    printf("sparkfun flash! \n");
    nrf_delay_ms(1000); 
    loop();
  }
}

