#include <Arduino.h>
#include "SPISlave_T4.h"

#define SPI_DEBUG 1
#define SPI_FRAME_SIZE 26
#define SPI_FRAMES_BUFFER_SIZE 20

#define SYNC_BYTE_0 0xAA
#define SYNC_BYTE_1 0x55

SPISlave_T4<&SPI, SPI_8_BITS> mySPI;

enum class SPIState {
  IDLE,
  SYNCING,
  TRANSCEIVING
};

// Functional member variables
static volatile uint32_t _spiInputBuffer[SPI_FRAME_SIZE];
static volatile uint8_t _spiInputBufferIndex = 0;
static volatile bool _frameSynced = false;
static volatile uint32_t _spiInputFramesBuffer[SPI_FRAMES_BUFFER_SIZE][SPI_FRAME_SIZE];
static volatile uint8_t _spiInputFramesIndex = 0;
static volatile bool _frameReady = false;
static volatile SPIState _spiState = SPIState::IDLE;

// Monitoring counters
static volatile uint32_t _interruptCalls = 0;
static volatile uint32_t _lastByteReceived = 0;
static volatile uint32_t _totalBytesReceived = 0;
static volatile uint32_t _totalFramesReceived = 0;
static volatile uint32_t _bytesLostSyncing = 0;
static volatile uint32_t _partialFrameErrorCount = 0;
static volatile uint32_t _bytesReceivedInLastInterrupt = 0;
static volatile bool _uncheckedInterruptDebugData = false;
static volatile uint32_t _fcfsReceived = 0;
static volatile uint32_t _txErrorCount = 0;

void handleMessage() {
  _interruptCalls++;
  if (!_uncheckedInterruptDebugData) _bytesReceivedInLastInterrupt = 0;
  while (mySPI.isDataAvailable()) { // data ready in receive FIFO

    /* Receive */

    uint32_t rxData = mySPI.popr(); // Read the next incoming byte - automatically clears the RDF flag
    _lastByteReceived = rxData; // Record it for debug purposes
    // Update debug counters
    _totalBytesReceived++;
    if (!_uncheckedInterruptDebugData) _bytesReceivedInLastInterrupt++;

    // Doing our own frame syncing because the CS management from the Raspberry Pi seems unreliable
    if (!_frameSynced) {
      _spiState = SPIState::SYNCING;
      // Look for the sync header bytes 0xAA, 0x55 to establish frame alignment
      if (_spiInputBufferIndex == 0 && rxData == SYNC_BYTE_0) {
        _spiInputBuffer[_spiInputBufferIndex] = rxData;
        _spiInputBufferIndex = 1;
      } else if (_spiInputBufferIndex == 1 && rxData == SYNC_BYTE_1) {
        _spiInputBuffer[_spiInputBufferIndex] = rxData;
        _spiInputBufferIndex = 2;
        // Sync established
        _frameSynced = true;
      } else {
        _bytesLostSyncing++;
        _spiInputBufferIndex = 0;
      }
    } else {
      _spiState = SPIState::TRANSCEIVING;
      // Check sync
      if ((_spiInputBufferIndex == 0 && (rxData != SYNC_BYTE_0)) || (_spiInputBufferIndex == 1 && (rxData != SYNC_BYTE_1))) {
        _frameSynced = false;
        _bytesLostSyncing++;
        _spiInputBufferIndex = 0;
      } else {
        // Store it in the circular input buffer
        _spiInputBuffer[_spiInputBufferIndex] = rxData;
        _spiInputBufferIndex++;
        if (_spiInputBufferIndex >= SPI_FRAME_SIZE) {
          // Save the frame
          memcpy((void*)_spiInputFramesBuffer[_spiInputFramesIndex], (const void*)_spiInputBuffer, sizeof(_spiInputBuffer));
          // Flag that a new frame is ready for processing
          _frameReady = true;
          _spiInputFramesIndex++;
          if (_spiInputFramesIndex >= SPI_FRAMES_BUFFER_SIZE) {
            _spiInputFramesIndex = 0; // Wrap around the circular buffer
          }
          _spiInputBufferIndex = 0; // Reset the buffer index for the next frame
          // Up the monitoring counter
          _totalFramesReceived++;
        }
      }
    }

  }

  /* Transmit */

  if (LPSPI4_SR & LPSPI_SR_TDF) {
    // The number of words in the transmit FIFO is less than the watermark, so fill it up
    LPSPI4_TDR = 0xBE; // Loopback for now
  }

  /* Error checking */

  if (LPSPI4_SR & LPSPI_SR_TEF) {
    // Transmit error flag is set, indicating a transmit FIFO underflow or other error
    // Clear it and record the error
    LPSPI4_SR = LPSPI_SR_TEF;
    _txErrorCount++;
  }

  // Flag that monitoring data is available
  _uncheckedInterruptDebugData = true;

  if (LPSPI4_SR & LPSPI_SR_FCF) {
    // Write to the FCF flag to clear it (write-1-to-clear register)
    LPSPI4_SR = LPSPI_SR_FCF;
    // Don't do anything with it because it's been found to be unreliable
    _fcfsReceived++;
  }

  // Clear off the Word flag - we don't care about it
  if (LPSPI4_SR & LPSPI_SR_WCF) {
    LPSPI4_SR = LPSPI_SR_WCF; // Clear the WCF flag
  }
}

void printTCRRegisterDetail() {
  Serial.printf("[main] [SPI DEBUG] LPSPI4_TCR: 0x%08X\r\n", LPSPI4_TCR);

  uint32_t cpol = (LPSPI4_TCR & LPSPI_TCR_CPOL) ? 1U : 0U;
  uint32_t cpha = (LPSPI4_TCR & LPSPI_TCR_CPHA) ? 1U : 0U;
  uint32_t prescale = (LPSPI4_TCR & LPSPI_TCR_PRESCALE(0x07)) / LPSPI_TCR_PRESCALE(1); // 3-bit field
  uint32_t pcs = (LPSPI4_TCR & LPSPI_TCR_PCS(0x07)) / LPSPI_TCR_PCS(1); // 3-bit field
  uint32_t lsbf = (LPSPI4_TCR & LPSPI_TCR_LSBF) ? 1U : 0U;
  uint32_t bysw = (LPSPI4_TCR & LPSPI_TCR_BYSW) ? 1U : 0U;
  uint32_t cont = (LPSPI4_TCR & LPSPI_TCR_CONT) ? 1U : 0U;
  uint32_t contc = (LPSPI4_TCR & LPSPI_TCR_CONTC) ? 1U : 0U;
  uint32_t rxmsk = (LPSPI4_TCR & LPSPI_TCR_RXMSK) ? 1U : 0U;
  uint32_t txmsk = (LPSPI4_TCR & LPSPI_TCR_TXMSK) ? 1U : 0U;
  uint32_t width = (LPSPI4_TCR & LPSPI_TCR_WIDTH(0x03)) / LPSPI_TCR_WIDTH(1); // 2-bit field
  uint32_t framesz = (LPSPI4_TCR & LPSPI_TCR_FRAMESZ(0xFFFF)) / LPSPI_TCR_FRAMESZ(1); // 16-bit field

  Serial.printf("[main] [SPI DEBUG]   CPOL: %u  | CPHA: %u  | PRESCALE: %u\r\n", cpol, cpha, prescale); 
  Serial.printf("[main] [SPI DEBUG]   PCS: %u   | LSBF: %u  | BYSW: %u\r\n", pcs, lsbf, bysw);
  Serial.printf("[main] [SPI DEBUG]   CONT: %u  | CONTC: %u | RXMSK: %u | TXMSK: %u\r\n", cont, contc, rxmsk, txmsk);
  Serial.printf("[main] [SPI DEBUG]   WIDTH: %u | FRAMESZ: %u\r\n", width, framesz);
  Serial.flush();
}

void printSRRegisterDetail() {
  Serial.printf("[main] [SPI DEBUG] LPSPI4_SR: 0x%08X\r\n", LPSPI4_SR);
  Serial.printf("[main] [SPI DEBUG]   ");
  if (LPSPI4_SR & LPSPI_SR_TDF) Serial.printf("TDF ");
  if (LPSPI4_SR & LPSPI_SR_RDF) Serial.printf("RDF ");
  if (LPSPI4_SR & LPSPI_SR_WCF) Serial.printf("WCF ");
  if (LPSPI4_SR & LPSPI_SR_FCF) Serial.printf("FCF ");
  if (LPSPI4_SR & LPSPI_SR_TCF) Serial.printf("TCF ");
  if (LPSPI4_SR & LPSPI_SR_TEF) Serial.printf("TEF ");
  if (LPSPI4_SR & LPSPI_SR_REF) Serial.printf("REF ");
  if (LPSPI4_SR & LPSPI_SR_DMF) Serial.printf("DMF ");
  if (LPSPI4_SR & LPSPI_SR_MBF) Serial.printf("MBF ");
  Serial.printf("\r\n");
  Serial.flush();
}

void printFSRRegisterDetail() {
  Serial.printf("[main] [SPI DEBUG] LPSPI4_FSR: 0x%08X\r\n", LPSPI4_FSR);

  uint32_t tx_count = (LPSPI4_FSR & LPSPI_FSR_TXCOUNT(0x1F)) / LPSPI_FSR_TXCOUNT(1); // 5-bit field (4-0)
  uint32_t rx_count = (LPSPI4_FSR & LPSPI_FSR_RXCOUNT(0x1F)) / LPSPI_FSR_RXCOUNT(1); // 5-bit field (20-16)
  
  Serial.printf("[main] [SPI DEBUG]   TXCOUNT: %u | RXCOUNT: %u\r\n", tx_count, rx_count);
  Serial.flush();
}

void printRSRRegisterDetail() {
  Serial.printf("[main] [SPI DEBUG] LPSPI4_RSR: 0x%08X\r\n", LPSPI4_RSR);

  uint32_t startOfFrame = (LPSPI4_RSR & LPSPI_RSR_SOF) ? 1U : 0U; // 1-bit field (0)
  uint32_t rxEmpty = (LPSPI4_RSR & LPSPI_RSR_RXEMPTY) ? 1U : 0U; // 1-bit field (1)

  Serial.printf("[main] [SPI DEBUG]   SOF: %u | RXEMPTY: %u\r\n", startOfFrame, rxEmpty);
  Serial.flush();
}

void printSPIParameterRegister() {
  Serial.printf("[main] [SPI DEBUG] LPSPI4_PARAM: 0x%08X\r\n", LPSPI4_PARAM);

  uint32_t rx_fifo_size = (LPSPI4_PARAM >> 8) & 0xFF; // 8-bit field (15-8)
  uint32_t tx_fifo_size = LPSPI4_PARAM & 0xFF; // 8-bit field (7-0)

  Serial.printf("[main] [SPI DEBUG]   RX MAX FIFO SIZE: %u | TX MAX FIFO SIZE: %u\r\n", 1 << rx_fifo_size, 1 << tx_fifo_size);
  Serial.flush();
}

void printFCRRegisterDetail() {
  Serial.printf("[main] [SPI DEBUG] LPSPI4_FCR: 0x%08X\r\n", LPSPI4_FCR);

  uint32_t rx_watermark = (LPSPI4_FCR & LPSPI_FCR_RXWATER(0x0F)) / LPSPI_FCR_RXWATER(1); // 4-bit field (19-16)
  uint32_t tx_watermark = (LPSPI4_FCR & LPSPI_FCR_TXWATER(0x0F)) / LPSPI_FCR_TXWATER(1); // 4-bit field (3-0)

  Serial.printf("[main] [SPI DEBUG]   RX WATERMARK: %u | TX WATERMARK: %u\r\n", rx_watermark, tx_watermark);
  Serial.flush();
}

void printIERRegisterDetail() {
  Serial.printf("[main] [SPI DEBUG] LPSPI4_IER: 0x%08X\r\n", LPSPI4_IER);
  Serial.printf("[main] [SPI DEBUG]   ");
  if (LPSPI4_IER & LPSPI_IER_TDIE) Serial.printf("TDIE ");
  if (LPSPI4_IER & LPSPI_IER_RDIE) Serial.printf("RDIE ");
  if (LPSPI4_IER & LPSPI_IER_WCIE) Serial.printf("WCIE ");
  if (LPSPI4_IER & LPSPI_IER_FCIE) Serial.printf("FCIE ");
  if (LPSPI4_IER & LPSPI_IER_TCIE) Serial.printf("TCIE ");
  if (LPSPI4_IER & LPSPI_IER_TEIE) Serial.printf("TEIE ");
  if (LPSPI4_IER & LPSPI_IER_REIE) Serial.printf("REIE ");
  if (LPSPI4_IER & LPSPI_IER_DMIE) Serial.printf("DMIE ");
  Serial.printf("\r\n");
  Serial.flush();
}

void printCFGR1RegisterDetail() {
  Serial.printf("[main] [SPI DEBUG] LPSPI4_CFGR1: 0x%08X\r\n", LPSPI4_CFGR1);
  
  uint32_t masterSlaveMode = (LPSPI4_CFGR1 & LPSPI_CFGR1_MASTER) ? 1U : 0U; // 1-bit field (0)
  uint32_t samplePoint = (LPSPI4_CFGR1 & LPSPI_CFGR1_SAMPLE) ? 1U : 0U; // 1-bit field (1)
  uint32_t autoPCS = (LPSPI4_CFGR1 & LPSPI_CFGR1_AUTOPCS) ? 1U : 0U; // 1-bit field (2)
  uint32_t noStall = (LPSPI4_CFGR1 & LPSPI_CFGR1_NOSTALL) ? 1U : 0U; // 1-bit field (3)
  uint32_t pcs0Polarity = (LPSPI4_CFGR1 & LPSPI_CFGR1_PCSPOL(0)) ? 1U : 0U; // 1-bit field (8)
  uint32_t pcs1Polarity = (LPSPI4_CFGR1 & LPSPI_CFGR1_PCSPOL(1)) ? 1U : 0U; // 1-bit field (9)
  uint32_t pcs2Polarity = (LPSPI4_CFGR1 & LPSPI_CFGR1_PCSPOL(2)) ? 1U : 0U; // 1-bit field (10)
  uint32_t pcs3Polarity = (LPSPI4_CFGR1 & LPSPI_CFGR1_PCSPOL(3)) ? 1U : 0U; // 1-bit field (11)
  uint32_t matchConfig = (LPSPI4_CFGR1 & LPSPI_CFGR1_MATCFG(0x07)) / LPSPI_CFGR1_MATCFG(1); // 3-bit field (18-16)
  uint32_t pinConfig = (LPSPI4_CFGR1 & LPSPI_CFGR1_PINCFG(0x03)) / LPSPI_CFGR1_PINCFG(1); // 2-bit field (25-24)
  uint32_t outConfig = (LPSPI4_CFGR1 & LPSPI_CFGR1_OUTCFG) ? 1U : 0U; // 1-bit field (26)
  uint32_t pcsConfig = (LPSPI4_CFGR1 & LPSPI_CFGR1_PCSCFG) ? 1U : 0U; // 1-bit field (27)

  Serial.printf("[main] [SPI DEBUG]   MASTER/SLAVE: %s | SAMPLE: %u | AUTOPCS: %u | NOSTALL: %u\r\n", 
                masterSlaveMode ? "MASTER" : "SLAVE", samplePoint, autoPCS, noStall);
  Serial.printf("[main] [SPI DEBUG]   PCSPOL[0-3]: %u %u %u %u | MATCFG: 0x%03X | PINCFG: 0x%02X | OUTCFG: %u | PCSCFG: %u\r\n",
                pcs0Polarity, pcs1Polarity, pcs2Polarity, pcs3Polarity, matchConfig, pinConfig, outConfig, pcsConfig);
  Serial.flush();
}

void spiRegisterAudit() {
  // 1. Snapshot all the raw IOMUX Multiplexer settings
  uint32_t muxCS   = IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00; // Pin 10
  uint32_t muxMOSI = IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02; // Pin 11
  uint32_t muxMISO = IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01; // Pin 12
  uint32_t muxSCK  = IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03; // Pin 13

  // 2. Snapshot all the raw Pad Control settings (pulls, speed, hysteresis)
  uint32_t padCS   = IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00; // Pin 10 (0x401F832C)
  uint32_t padMOSI = IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02; // Pin 11 (0x401F8334)
  uint32_t padMISO = IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01; // Pin 12 (0x401F8330)
  uint32_t padSCK  = IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03; // Pin 13 (0x401F8338)

  // 3. Snapshot the core GPIO7 direction bit states
  // Pin 10 = GPIO7 bit 0, Pin 11 = GPIO7 bit 2, Pin 12 = GPIO7 bit 1, Pin 13 = GPIO7 bit 3
  bool dirCS   = (GPIO7_GDIR >> 0) & 0x01;
  bool dirMOSI = (GPIO7_GDIR >> 2) & 0x01;
  bool dirMISO = (GPIO7_GDIR >> 1) & 0x01;
  bool dirSCK  = (GPIO7_GDIR >> 3) & 0x01;

  Serial.println("\n================= TEENSY SPI HARDWARE BUS SILICON AUDIT =================");
  // --- PIN 10: CHIP SELECT ---
  Serial.printf("PIN 10 [CS]   -> MUX (ALT): %u | GPIO Dir: %s | PAD Reg: 0x%04X ", 
                muxCS & 0x07, dirCS ? "OUTPUT" : "INPUT", padCS);
  Serial.printf("(Schmitt: %s, Pulls: %s)\n", 
                (padCS & (1 << 16)) ? "ON" : "OFF",
                ((padCS >> 14) & 0x03) == 0 ? "NONE" : ((padCS >> 14) & 0x03) == 1 ? "100k-DOWN" : "100k-UP");

  // --- PIN 11: MOSI ---
  Serial.printf("PIN 11 [MOSI] -> MUX (ALT): %u | GPIO Dir: %s | PAD Reg: 0x%04X ", 
                muxMOSI & 0x07, dirMOSI ? "OUTPUT" : "INPUT", padMOSI);
  Serial.printf("(Schmitt: %s, Pulls: %s)\n", 
                (padMOSI & (1 << 16)) ? "ON" : "OFF",
                ((padMOSI >> 14) & 0x03) == 0 ? "NONE" : ((padMOSI >> 14) & 0x03) == 1 ? "100k-DOWN" : "100k-UP");

  // --- PIN 12: MISO ---
  Serial.printf("PIN 12 [MISO] -> MUX (ALT): %u | GPIO Dir: %s | PAD Reg: 0x%04X ", 
                muxMISO & 0x07, dirMISO ? "OUTPUT" : "INPUT", padMISO);
  Serial.printf("(Schmitt: %s, Pulls: %s)\n", 
                (padMISO & (1 << 16)) ? "ON" : "OFF",
                ((padMISO >> 14) & 0x03) == 0 ? "NONE" : ((padMISO >> 14) & 0x03) == 1 ? "100k-DOWN" : "100k-UP");

  // --- PIN 13: SCK ---
  Serial.printf("PIN 13 [SCK]  -> MUX (ALT): %u | GPIO Dir: %s | PAD Reg: 0x%04X ", 
                muxSCK & 0x07, dirSCK ? "OUTPUT" : "INPUT", padSCK);
  Serial.printf("(Schmitt: %s, Pulls: %s)\n", 
                (padSCK & (1 << 16)) ? "ON" : "OFF",
                ((padSCK >> 14) & 0x03) == 0 ? "NONE" : ((padSCK >> 14) & 0x03) == 1 ? "100k-DOWN" : "100k-UP");

  // --- CHIP-LEVEL CONTROL OVERRIDES ---
  Serial.printf("Daisy Chains -> SDO_SELECT: 0x%X | SDI_SELECT: 0x%X\n", 
                IOMUXC_LPSPI4_SDO_SELECT_INPUT, IOMUXC_LPSPI4_SDI_SELECT_INPUT);
  Serial.printf("LPSPI4 Status -> CR: 0x%08X | IER: 0x%08X | SR: 0x%08X\n", 
                LPSPI4_CR, LPSPI4_IER, LPSPI4_SR);
  Serial.println("=========================================================================");
  // Core config
  printCFGR1RegisterDetail();
  printTCRRegisterDetail();
  printIERRegisterDetail();
  // FIFO sizes and details
  printSPIParameterRegister();
  printFCRRegisterDetail();
  Serial.println("=========================================================================");
  Serial.flush();
}

void setup() {
  // Start up the serial interface for debugging and output
  Serial.begin(115200);

  // WARNING: USING THE LED BEFORE SORTING OUT SPI SEEMS TO MESS UP THE SPI PINS.  SO DON'T DO IT.
  // Flash the LED while waiting for a serial connection (10 second timeout)
  // pinMode(LED_BUILTIN, OUTPUT);
  unsigned long timeout = millis();
  while (!Serial && (millis() - timeout < 10000)) {
    // digitalWrite(LED_BUILTIN, (millis() / 250) % 2); 
  }
  // digitalWrite(LED_BUILTIN, LOW);
  // pinMode(LED_BUILTIN, INPUT);

  
  if (Serial) {
    // If we've got a connection then wait a second for Serial buffers to catch up
    delay(1000);
    Serial.println("[main] Serial terminal connected"); Serial.flush();
  }

  // Trigger interrupts on any data received
  mySPI.setIERTriggerMode(true, false);
  
  // Start up the SPI slave interface
  if (SPI_DEBUG) {Serial.println("[main] [SPI DEBUG] Starting SPI slave interface..."); Serial.flush();}
  mySPI.begin();
	// Enable MOSI to MOSI and MISO to MISO rather than the other way around which is the default.
  // if (SPI_DEBUG) {Serial.println("[main] [SPI DEBUG] Swapping SPI pins to enable MOSI to MOSI and MISO to MISO..."); Serial.flush();}
  // mySPI.swapPins(false); // false = don't enable sniffer mode
 	// Register the callback function to handle incoming SPI messages
  if (SPI_DEBUG) {Serial.println("[main] [SPI DEBUG] Registering SPI callback function..."); Serial.flush();}
  mySPI.onReceive(handleMessage);
  // Check setup on all the SPI registers
  if (SPI_DEBUG) spiRegisterAudit();

  Serial.println("[main] Initialisation complete."); Serial.flush();
}

void loop() {
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime >= 4000) {
    lastPrintTime = millis();
    if (SPI_DEBUG) {
      // Print out the important run-time status register details
      printSRRegisterDetail();
      printFSRRegisterDetail();
      printRSRRegisterDetail();
      Serial.printf("[main] [SPI DEBUG] SPI State: %s\r\n", 
                    _spiState == SPIState::IDLE ? "IDLE" : (_spiState == SPIState::SYNCING ? "SYNCING" : "TRANSCEIVING"));
      Serial.printf("[main] [SPI DEBUG] Number of interrupt calls: %lu | TX Errors: %lu\r\n", _interruptCalls, _txErrorCount);
      if (_uncheckedInterruptDebugData) {
        _uncheckedInterruptDebugData = false;
        Serial.printf("[main] [SPI DEBUG] Bytes received in last interrupt: %lu | Total bytes received: %lu | Bytes lost syncing: %lu\r\n", _bytesReceivedInLastInterrupt, _totalBytesReceived, _bytesLostSyncing);
        Serial.printf("[main] [SPI DEBUG] Last byte received: %02X\r\n", _lastByteReceived);
        // Serial.printf("[main] [SPI DEBUG] Input Buffer: ");
        // for (int i = 0; i < SPI_FRAME_SIZE; i++) {
        //   Serial.printf("%02X ", _spiInputBuffer[i]);
        // }
        // Serial.println();
        Serial.flush();
      }
      if (_frameReady) {
        _frameReady = false;
        Serial.printf("[main] [SPI DEBUG] Total frames received: %lu\r\n", _totalFramesReceived);
        Serial.printf("[main] [SPI DEBUG] Last full frame: ");
        uint32_t lastFrameIndex = _spiInputFramesIndex;
        for (int i = 0; i < SPI_FRAME_SIZE; i++) {
          Serial.printf("%02X ", _spiInputFramesBuffer[lastFrameIndex][i]);
        }
        Serial.println();
        Serial.flush();
      }
    }
  }
}


