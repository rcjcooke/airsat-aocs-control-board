#include "OBCConnection.h"
#include <SPISlave_T4.h> // Library stays strictly here

// Instantiate the driver and buffers all in the same compilation translation unit
static SPISlave_T4<&SPI, SPI_8_BITS> hardwareSPI;

static volatile CommandFrame   _incomingFrame;
static volatile TelemetryFrame _outgoingFrame;

static CommandPayload _verifiedCommand; 
static volatile bool  _newCommandReady = false;

static volatile uint8_t* _rxPtr = (uint8_t*)&_incomingFrame;
static volatile uint8_t* _txPtr = (uint8_t*)&_outgoingFrame;
static volatile uint8_t  _rxIndex = 0;
static volatile bool     _frameSynced = false;
static volatile uint16_t _localErrorCount = 0;

// Internal Private Math Helpers
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

// The direct hardware ISR callback
void mySPIISRHandler() {
  while (hardwareSPI.active()) {
    if (hardwareSPI.available()) {
      uint8_t incomingByte = hardwareSPI.popr();

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
      hardwareSPI.pushr(_txPtr[_rxIndex]);
    }
  }
}

void initOBCConnection() {
  _rxIndex = 0;
  _frameSynced = false;
  _localErrorCount = 0;
  
  _outgoingFrame.sync[0] = 0xAA; 
  _outgoingFrame.sync[1] = 0x55;
  memset((void*)&_outgoingFrame.payload, 0, sizeof(TelemetryPayload));
  updateTxBuffer();

  // 1. Core initialization clears past the old NVIC lock smoothly!
  hardwareSPI.begin();
  
  // 2. Register your user-space handler safely now that the NVIC vector is stable
  hardwareSPI.onReceive(mySPIISRHandler);
}


bool isOBCCommandAvailable() {
  return _newCommandReady;
}

CommandPayload getLatestOBCCommand() {
  CommandPayload temp;
  noInterrupts();
  temp = _verifiedCommand;
  _newCommandReady = false;
  interrupts();
  return temp;
}

void updateOBCTelemetry(const TelemetryPayload& freshTelem) {
  noInterrupts();
  memcpy((void*)&_outgoingFrame.payload, &freshTelem, sizeof(TelemetryPayload));
  updateTxBuffer();
  interrupts();
}

uint16_t getOBCRxErrorCount() {
  return _localErrorCount;
}
