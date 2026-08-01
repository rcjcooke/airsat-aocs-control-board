#include "OBCConnection.h"
#include <SPISlave_T4.h>

// Instantiate the underlying hardware driver for standard SPI port (Pins 10-13)
static SPISlave_T4<&SPI, SPI_8_BITS> hardwareSPI;
OBCConnection* g_obcConnectionPtr = nullptr;

// Global wrapper function because SPISlave_T4 demands a non-member function pointer for callbacks
void globalSPIISRWrapper() {
  if (g_obcConnectionPtr != nullptr) {
    g_obcConnectionPtr->handleInterrupt();
  }
}

OBCConnection::OBCConnection() 
  : _newCommandReady(false), _rxIndex(0), _frameSynced(false), _localErrorCount(0) {
  _rxPtr = (uint8_t*)&_incomingFrame;
  _txPtr = (uint8_t*)&_outgoingFrame;
  
  // Pre-seed frames with sync codes
  _outgoingFrame.sync[0] = 0xAA;
  _outgoingFrame.sync[1] = 0x55;
  memset((void*)&_outgoingFrame.payload, 0, sizeof(TelemetryPayload));
}

void OBCConnection::begin() {
  g_obcConnectionPtr = this;
  
  updateTxBuffer();
  hardwareSPI.onReceive(globalSPIISRWrapper);
  hardwareSPI.begin();
  hardwareSPI.pushr(_txPtr[_rxIndex]); // Seed first byte into hardware FIFO
}

bool OBCConnection::isCommandAvailable() {
  return _newCommandReady;
}

CommandPayload OBCConnection::getLatestCommand() {
  CommandPayload temp;
  noInterrupts();
  temp = _verifiedCommand;
  _newCommandReady = false;
  interrupts();
  return temp;
}

void OBCConnection::updateTelemetry(const TelemetryPayload& newTelem) {
  noInterrupts();
  // Safe deep copy from non-volatile argument into volatile transmission struct
  memcpy((void*)&_outgoingFrame.payload, &newTelem, sizeof(TelemetryPayload));
  updateTxBuffer();
  interrupts();
}

uint16_t OBCConnection::getRxErrorCount() const {
  return _localErrorCount;
}

uint16_t OBCConnection::calculateFletcher16(const uint8_t* data, size_t count) {
  uint16_t sum1 = 0;
  uint16_t sum2 = 0;
  for (size_t i = 0; i < count; ++i) {
    sum1 = (sum1 + data[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }
  return (sum2 << 8) | sum1;
}

void OBCConnection::updateTxBuffer() {
  _outgoingFrame.payload.error_count = _localErrorCount;
  // Compute checksum over Sync (2 bytes) + Payload (22 bytes) = 24 bytes
  _outgoingFrame.checksum = calculateFletcher16((const uint8_t*)&_outgoingFrame, 24);
}

void OBCConnection::handleInterrupt() {
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
            // Inspect index tracking flag byte
            if (_incomingFrame.payload.flags == 0x22) {
              // Telemetry Poll Only: Return fresh telemetry to Pi but skip updating actuators
            } 
            else if (_incomingFrame.payload.flags == 0x11) {
              // Valid Command: Copy data across volatile barrier into the main workspace
              memcpy(&_verifiedCommand, (const void*)&_incomingFrame.payload, sizeof(CommandPayload));
              _newCommandReady = true;
            } 
            else {
              // Catch-all safety gate: Malformed or uninitialized flags result in a discarded frame
              _localErrorCount++;
            }
          } else {
            _localErrorCount++;
          }

          _frameSynced = false;
          _rxIndex = 0;
          updateTxBuffer(); // Ensure error counts are updated immediately on structural changes
        }
      }

      hardwareSPI.pushr(_txPtr[_rxIndex]);
    }
  }
}
