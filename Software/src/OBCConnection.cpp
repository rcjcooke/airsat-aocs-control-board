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

void rawLPSPI4_InterruptHandler() {
  // Process all incoming hardware data sitting inside the peripheral queue
  while (LPSPI4_SR & LPSPI_SR_RDF) {
    uint8_t incomingByte = LPSPI4_RDR;
    _totalBytesReceived++; // This will now climb rapidly!

    // Log the byte immediately into your circular ring buffer snapshot
    _diagnosticRxMirror[_diagnosticMirrorIndex] = incomingByte;
    _diagnosticMirrorIndex++;
    if (_diagnosticMirrorIndex >= 26) {
        _diagnosticMirrorIndex = 0;
    }

    if (!_frameSynced) {
      // Look for the production header byte index markers
      if (_rxIndex == 0 && incomingByte == 0xAA) {
        _rxPtr[_rxIndex] = incomingByte;
        _rxIndex = 1;
      } else if (_rxIndex == 1 && incomingByte == 0x55) {
        _rxPtr[_rxIndex] = incomingByte;
        _rxIndex = 2;
        _frameSynced = true; // Sync established!
      } else {
        _rxIndex = 0;
      }
    } else {
      // Store the active data byte payload stream
      _rxPtr[_rxIndex] = incomingByte;
      _rxIndex++;

      // Once the full 26-byte boundary frame is collected, parse it
      if (_rxIndex >= sizeof(CommandFrame)) {
        uint16_t calculated = calculateFletcher16((const uint8_t*)&_incomingFrame, 24);
        
        if (calculated == _incomingFrame.checksum) {
          // If the poll flag is 0x11, unpack the live floating point targets
          if (_incomingFrame.payload.flags == 0x11) {
              _verifiedCommand.torque = _incomingFrame.payload.torque;
              memcpy(_verifiedCommand.thrust, (const void*)_incomingFrame.payload.thrust, sizeof(_verifiedCommand.thrust));
              _newCommandReady = true;
          }
        } else {
          _localErrorCount++; // Tracks corrupted packets
        }
        
        // ATOMIC BOUNDARY RESET: Safely reset state flags for the next frame
        _frameSynced = false;
        _rxIndex = 0;
        updateTxBuffer(); 
      }
    }

    // Pre-load the exact next scheduled byte into the raw transmitter register
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

  Serial.println("[OBC LINK] Enforcing definitive 4-pin hardware alignment overrides...");
  Serial.flush();

  // 1. Initialize clock gates for the LPSPI4 peripheral block
  CCM_CSCMR1 &= ~(0x38 | 0x07);
  __asm__ volatile("dmb");
  CCM_CCGR1 |= CCM_CCGR1_LPSPI4(CCM_CCGR_ON);
  __asm__ volatile("dmb");

  // 2. FIXED: Force Pin 10 back to Function 3 (LPSPI4 PCS0 Chip Select input)
  // This removes the ALT 5 trap and hands the pin completely to the SPI engine!
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; // Pin 10 -> CS
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3; // Pin 11 -> MOSI
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x3; // Pin 12 -> MISO
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; // Pin 13 -> SCK

  // 3. Configure physical electrical properties (Force Schmitt trigger unmask inputs)
  // INPUT PINS: Pin 10 (CS), Pin 11 (MOSI), and Pin 13 (SCK)
  // - Bit 16 = 1    -> Schmitt Trigger Input Buffer ENABLED (Fixes the analog dead-zone)
  // - Bits 14-15 = 01 -> 100k Ohm Pull-DOWN register lane explicitly ENABLED!
  // - Bits 4-5 = 11   -> Maximum speed filtering enabled
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 = 0x015030; // Pin 10 (CS):  Schmitt ON + 100k Pull-DOWN reference
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02 = 0x015030; // Pin 11 (MOSI): Schmitt ON + 100k Pull-DOWN reference
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = 0x015030; // Pin 13 (SCK):  Schmitt ON + 100k Pull-DOWN reference

  // OUTPUT PIN: Pin 12 (MISO)
  // Keep it configured with pull-ups/pull-downs completely off, but with 
  // maximum drive strength so it can push its telemetry back cleanly.
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01 = 0x000038; // Pin 12 (MISO): High-Drive Output + Pulls disabled
  __asm__ volatile("dmb");

  // 4. FIXED: Map the internal input shift register channel directly to Pin 11 pad matrix
  // Under the NXP silicon manual, setting this to 0x1 maps the data pipeline straight to GPIO_B0_02!
  IOMUXC_LPSPI4_SDO_SELECT_INPUT = 0x0; 
  IOMUXC_LPSPI4_SDI_SELECT_INPUT = 0x1; 
  __asm__ volatile("dmb");

  // 5. Force the GPIO7 General Direction Register states explicitly
  GPIO7_GDIR &= ~(1 << 0); // Pin 10 (CS)   = INPUT register lane
  GPIO7_GDIR &= ~(1 << 2); // Pin 11 (MOSI) = INPUT register lane
  GPIO7_GDIR &= ~(1 << 3); // Pin 13 (SCK)  = INPUT register lane
  GPIO7_GDIR |=  (1 << 1); // Pin 12 (MISO) = OUTPUT register lane
  __asm__ volatile("dmb");

  // 6. Force configure the LPSPI4 hardware control registers directly
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
  LPSPI4_IER = 0x1; 
  LPSPI4_SR = 0x3F00; // Clear status register flags

  // 7. Forcefully bind the hardware vector table directly to our custom handler
  // We completely remove 'attachInterrupt(10, ...)' because the pin is no longer in GPIO mode!
  attachInterruptVector(IRQ_LPSPI4, rawLPSPI4_InterruptHandler);
  
  NVIC_CLEAR_PENDING(IRQ_LPSPI4);
  NVIC_ENABLE_IRQ(IRQ_LPSPI4);

  // 8. Populate the hardware FIFO directly using raw register access 
  LPSPI4_TDR = _txPtr[_rxIndex];
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

