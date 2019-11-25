#define RH_FLAGS_RESERVED                 0xf0
#define RH_FLAGS_APPLICATION_SPECIFIC     0x0f
#define RH_FLAGS_NONE                     0

// Default timeout for waitCAD() in ms
#define RH_CAD_DEFAULT_TIMEOUT            10000
#define RH_SPI_WRITE_MASK 0x80





typedef enum {
	RHModeInitialising = 0, ///< Transport is initialising. Initial default value until init() is called..
	RHModeSleep,            ///< Transport hardware is in low power sleep mode (if supported)
	RHModeIdle,             ///< Transport is idle.
	RHModeTx,               ///< Transport is in the process of transmitting a message.
	RHModeRx,               ///< Transport is in the process of receiving a message.
	RHModeCad               ///< Transport is in the process of detecting channel activity (if supported)
    
} RHMode;

typedef enum
    {
    DataMode0 = 0, ///< SPI Mode 0: CPOL = 0, CPHA = 0
    DataMode1,     ///< SPI Mode 1: CPOL = 0, CPHA = 1
    DataMode2,     ///< SPI Mode 2: CPOL = 1, CPHA = 0
    DataMode3,     ///< SPI Mode 3: CPOL = 1, CPHA = 1
    } DataMode;

typedef enum
    {
    Frequency1MHz = 0,  ///< SPI bus frequency close to 1MHz
    Frequency2MHz,      ///< SPI bus frequency close to 2MHz
    Frequency4MHz,      ///< SPI bus frequency close to 4MHz
    Frequency8MHz,      ///< SPI bus frequency close to 8MHz
    Frequency16MHz      ///< SPI bus frequency close to 16MHz
    } Frequency;

typedef enum
    {
    BitOrderMSBFirst = 0,  ///< SPI MSB first
    BitOrderLSBFirst,      ///< SPI LSB first
    } BitOrder;

typedef struct {

    // RHGenericDriver 

    RHMode              _mode,


	uint8_t             _thisAddress,
    
    /// Whether the transport is in promiscuous mode
    bool                _promiscuous,

    /// TO header in the last received mesasge
    volatile uint8_t    _rxHeaderTo,

    /// FROM header in the last received mesasge
    volatile uint8_t    _rxHeaderFrom,

    /// ID header in the last received mesasge
    volatile uint8_t    _rxHeaderId,

    /// FLAGS header in the last received mesasge
    volatile uint8_t    _rxHeaderFlags,

    /// TO header to send in all messages
    uint8_t             _txHeaderTo,

    /// FROM header to send in all messages
    uint8_t             _txHeaderFrom,

    /// ID header to send in all messages
    uint8_t             _txHeaderId,

    /// FLAGS header to send in all messages
    uint8_t             _txHeaderFlags,

    /// The value of the last received RSSI value, in some transport specific units
    volatile int16_t     _lastRssi,

    /// Count of the number of bad messages (eg bad checksum etc) received
    volatile uint16_t   _rxBad,

    /// Count of the number of successfully transmitted messaged
    volatile uint16_t   _rxGood,

    /// Count of the number of bad messages (correct checksum etc) received
    volatile uint16_t   _txGood,
    
    /// Channel activity detected
    volatile bool       _cad,

    /// Channel activity timeout in ms
    unsigned int        _cad_timeout,
    

    //nrf_drv_SPI

    

    /// The pin number of the Slave Select pin that is used to select the desired device.
    uint8_t             _slaveSelectPin;

    /// True when there is a valid message in the buffer
    volatile bool       _rxBufValid;

    /// The configured interrupt pin connected to this instance
    uint8_t             _interruptPin;

    /// The index into _deviceForInterrupt[] for this device (if an interrupt is already allocated)
    /// else 0xff
    uint8_t             _myInterruptIndex;

	
}RH_RF95;