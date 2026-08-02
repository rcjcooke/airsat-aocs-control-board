#include "OBCConnection.h"
#include <SPISlave_T4.h> 

static SPISlave_T4<&SPI, SPI_8_BITS> hardwareSPI;

// Keep your existing state and structure variables exactly the same...
static volatile CommandFrame   _incomingFrame;
static volatile TelemetryFrame _outgoingFrame;
static CommandPayload          _verifiedCommand; 
static volatile bool           _newCommandReady = false;
static volatile uint8_t*       _rxPtr = (uint8_t*)&_incomingFrame;
static volatile uint8_t*       _txPtr = (uint8_t*)&_outgoingFrame;
static volatile uint8_t        _rxIndex = 0;
static volatile bool           _frameSynced = false;
static volatile uint16_t       _localErrorCount = 0;
static volatile uint32_t       _totalBytesReceived = 0;
static volatile uint32_t       _totalInterruptsReceived = 0;
static volatile uint32_t       _csFallingEdgeCount = 0;
static volatile uint8_t _diagnosticRxMirror[26];
static volatile uint8_t _diagnosticMirrorIndex = 0;

// Internal Private Math Helpers stay identical...
static uint16_t calculateFletcher16(const uint8_t* data, size_t count) {
  uint16_t sum1 = 0, sum2 = 0;
  for (size_t i = 0; i < count; ++i) {
    sum1 = (sum1 + data[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }
  return (sum2 << 8) | sum1;
}

static void updateTxBuffer() {
  _outgoingFrame.payload.error_count = _localErrorCount;
  _outgoingFrame.checksum = calculateFletcher16((const uint8_t*)&_outgoingFrame, 24);
}

// ------------------------------------------------------------------------
// THE DIRECT HANDLER: Remove any library hooks
// ------------------------------------------------------------------------
void rawLPSPI4_InterruptHandler() {
  while (LPSPI4_SR & LPSPI_SR_RDF) {
    uint8_t incomingByte = LPSPI4_RDR;
    _totalBytesReceived++; 

    // --- NEW: DIAGNOSTIC CAPTURE BLOCK ---
    // Log the byte into a circular diagnostic mirror for main loop inspection
    _diagnosticRxMirror[_diagnosticMirrorIndex] = incomingByte;
    _diagnosticMirrorIndex++;
    if (_diagnosticMirrorIndex >= 26) {
        _diagnosticMirrorIndex = 0; // Wrap back to index zero safely
    }
    // -------------------------------------

    if (!_frameSynced) {
      if (_rxIndex == 0 && incomingByte == 0xAA) {
        _rxPtr[_rxIndex] = incomingByte;
        _rxIndex = 1;
      } else if (_rxIndex == 1 && incomingByte == 0x55) {
        _rxPtr[_rxIndex] = incomingByte;
        _rxIndex = 2;
        _frameSynced = true;
      } else {
        if (_rxIndex > 0) _localErrorCount++;
        _rxIndex = 0;
      }
    } else {
      _rxPtr[_rxIndex] = incomingByte;
      _rxIndex++;

      if (_rxIndex >= sizeof(CommandFrame)) {
        uint16_t calculated = calculateFletcher16((const uint8_t*)&_incomingFrame, 24);
        
        if (calculated == _incomingFrame.checksum) {
          if (_incomingFrame.payload.flags == 0x22) {
              // Telemetry Poll
          } 
          else if (_incomingFrame.payload.flags == 0x11) {
              memcpy(&_verifiedCommand, (const void*)&_incomingFrame.payload, sizeof(CommandPayload));
              _newCommandReady = true;
          } else {
            _localErrorCount++;
          }
        } else {
          _localErrorCount++;
        }
        _frameSynced = false;
        _rxIndex = 0;
        updateTxBuffer(); 
      }
    }
    LPSPI4_TDR = _txPtr[_rxIndex];
  }
}

// Keep your diagnostic helper functions for main.cpp...
void csISR() {
  if (digitalReadFast(10) == LOW) {
    _csFallingEdgeCount++;
  }
}

void initOBCConnection() {
  _rxIndex = 0;
  _frameSynced = false;
  _localErrorCount = 0;
  _totalBytesReceived = 0;
  _csFallingEdgeCount = 0;
  
  _outgoingFrame.sync[0] = 0xAA; 
  _outgoingFrame.sync[1] = 0x55;
  memset((void*)&_outgoingFrame.payload, 0, sizeof(TelemetryPayload));
  updateTxBuffer();

  Serial.println("[OBC LINK] Executing core system clock tree overrides...");
  Serial.flush();

  // 1. Force turn on the hardware clock gates for the LPSPI4 peripheral block
  CCM_CSCMR1 &= ~(0x38 | 0x07);
  __asm__ volatile("dmb");
  CCM_CCGR1 |= CCM_CCGR1_LPSPI4(CCM_CCGR_ON);
  __asm__ volatile("dmb");

  // 2. Map Pins 10-13 to Function 3 (LPSPI4 Mux Mappings)
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; // Pin 10 -> CS
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3; // Pin 11 -> MOSI
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x3; // Pin 12 -> MISO
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; // Pin 13 -> SCK

  // INPUT PINS: Pin 11 (MOSI) & Pin 13 (SCK)
  // - Bit 16 = 1 -> Schmitt Trigger Input Buffer ENABLED (Fixes the 2.6V analog floor!)
  // - Bits 14-15 = 01 -> 100k Ohm Pull-DOWN enabled (Clamps line safely to 0V idle)
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02 = 0x011030; // Pin 11 (MOSI Input)
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = 0x011030; // Pin 13 (SCK Input)

  // OUTPUT PIN: Pin 12 (MISO)
  // - Bit 16 = 0 -> Hysteresis DISABLED (Not an input)
  // - Bits 14-15 = 00 -> Pull-Up / Pull-Down completely DISABLED (High-Impedance Output)
  // - Bits 3-5 = 111 -> Drive Strength pushed to absolute MAX (Enables crisp high-speed rising edges)
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01 = 0x000038; // Pin 12 (MISO Output)
  __asm__ volatile("dmb");

  // Pin 11 maps to Bit 2 of GPIO7 (Should be 0 for Input)
  // Pin 12 maps to Bit 1 of GPIO7 (Must be 1 for Output!)
  // We clear Bit 2 and force Bit 1 high to guarantee correct data flow:
  GPIO7_GDIR &= ~(1 << 2); // Force Pin 11 (MOSI) to be an INPUT register lane
  GPIO7_GDIR |=  (1 << 1); // Force Pin 12 (MISO) to be an OUTPUT register lane
  __asm__ volatile("dmb");

  // ------------------------------------------------------------------------
  // THE CORRECT SLAVE DAISY CHAIN OVERRIDE
  // ------------------------------------------------------------------------
  // Ensure the Master mode monitoring register is set to neutral to avoid conflict
  IOMUXC_LPSPI4_SDO_SELECT_INPUT = 0x0; 
  
  // FIXED: Explicitly configure the TRUE Slave Data Input routing register!
  // Setting IOMUXC_LPSPI4_SDI_SELECT_INPUT to 0x0 tells the core processor to 
  // route the incoming bitstream coming from the physical GPIO_B0_01 pad (Teensy Pin 12).
  // Wait! In tonton81's library mapping, the physical trace pins are cross-bound.
  // Let's explicitly force BOTH select register avenues to 0x0 to force a full 
  // clear on all multiplexed input channels, isolating the hardware pins completely:
  IOMUXC_LPSPI4_SDI_SELECT_INPUT = 0x0; 
  __asm__ volatile("dmb");
  // ------------------------------------------------------------------------

  // 4. Force configure the LPSPI4 hardware control registers directly
  LPSPI4_CR = 0;               // Disable module completely to reset state engines
  LPSPI4_CFGR0 = 0;            // Clear tracking frames
  LPSPI4_CFGR1 = 0;            // Set MASTER = 0 (Explicit Hardware Slave Mode)
  
  // Configure Clock Phase and Polarity for SPI Mode 0 (CPOL=0, CPHA=0)
  LPSPI4_TCR = LPSPI_TCR_FRAMESZ(7); // Force explicit 8-bit parsing word lengths

  // Enable Transmitter and Receiver FIFO structural lines
  LPSPI4_FCR = LPSPI_FCR_TXWATER(0) | LPSPI_FCR_RXWATER(0);
  
  // Re-enable the hardware module safely as an active Slave listener
  LPSPI4_CR = LPSPI_CR_MEN; 
  __asm__ volatile("dmb");

  // Force the Interrupt Enable Register to unmask the Receive Data Ready flag
  LPSPI4_IER = LPSPI_IER_RDIE; 
  LPSPI4_SR = 0x3F00; // Clear status register

  // 5. Forcefully bind the hardware vector table directly to our custom implementation
  attachInterruptVector(IRQ_LPSPI4, rawLPSPI4_InterruptHandler);
  
  NVIC_CLEAR_PENDING(IRQ_LPSPI4);
  NVIC_ENABLE_IRQ(IRQ_LPSPI4);

  // 6. Populate the hardware FIFO directly using raw register access 
  LPSPI4_TDR = _txPtr[_rxIndex];

  // Connect local CS diagnostic edge tracker
  attachInterrupt(10, csISR, CHANGE);
}


// Keep your standard public wrapper interfaces intact...
bool isOBCCommandAvailable() { return _newCommandReady; }
CommandPayload getLatestOBCCommand() {
  CommandPayload temp;
  noInterrupts(); temp = _verifiedCommand; _newCommandReady = false; interrupts();
  return temp;
}
void updateOBCTelemetry(const TelemetryPayload& freshTelem) {
  noInterrupts(); memcpy((void*)&_outgoingFrame.payload, &freshTelem, sizeof(TelemetryPayload)); updateTxBuffer(); interrupts();
}
uint32_t getOBCTotalBytesReceived() { return _totalBytesReceived; }
uint32_t getOBCTotalInterruptsReceived() { return _totalInterruptsReceived; }
uint16_t getOBCRxErrorCount() { return _localErrorCount; }
uint32_t getOBCCSFallingEdges() { return _csFallingEdgeCount; }
uint32_t getOBCCSLineState() { return digitalReadFast(10); }

void getOBCRawRxBufferSnapshot(uint8_t* destinationBuffer, size_t maxBytes) {
  if (maxBytes > 26) maxBytes = 26;
  
  noInterrupts(); // Temporarily mask interrupts to prevent a partial write mid-copy
  memcpy(destinationBuffer, (const void*)_diagnosticRxMirror, maxBytes);
  interrupts();   // Re-enable interrupts immediately
}

