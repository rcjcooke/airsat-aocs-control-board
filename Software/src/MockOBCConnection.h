#ifndef MOCK_OBC_CONNECTION_H
#define MOCK_OBC_CONNECTION_H

#include <Arduino.h>

#include "AOCSPacketStructures.h"
#include "AOCSControllerTelemetry.h"
#include "SPIConnection.h"

class MockOBCConnection {
 public:
  MockOBCConnection() = default;
 
  void begin() {
    // No-op for mock
  }

  void activateSPI() {
    // No-op for mock
  }

  void service() {
    // Simulate receiving a command every 1 seconds
    static uint32_t lastCommandTime = 0;
    if (millis() - lastCommandTime >= 1000) {
      lastCommandTime = millis();
      // Simulate a new command received
      // In a real implementation, this would be set based on actual SPI communication
      m_newCommandReady = true;
      // Linearly vary the torque between 0.015N and -0.015N for testing
      float torque = 0.015f * sin(millis() / 1000.0f);
      m_newCommand = CommandPayload{torque, {1.0f, 1.0f, 1.0f, 1.0f}, 0, 0}; // Example command payload
    }
  }
  
  bool hasNewCommand() const {
    return m_newCommandReady;
  }

  CommandPayload takeLatestCommand() {
    m_newCommandReady = false;
    return m_newCommand; // Return the latest command in mock
  }

  void updateTelemetry(const AOCSControllerTelemetry& telemetry) {
    // No-op for mock
  }

  bool isConnected() const {
    return true; // Always connected in mock
  }

  uint32_t rxErrorCount() const {
    return 0; // No errors in mock
  }

  uint32_t totalBytesReceived() const {
    return 0; // No bytes received in mock
  }

  uint8_t syncDropCount() const {
    return 0; // No sync drops in mock
  }

  uint32_t commandCount() const {
    return 0; // No commands in mock
  }

  uint32_t noOpCount() const {
    return 0; // No no-ops in mock
  }

  uint32_t malformedPacketCount() const {
    return 0; // No malformed packets in mock
  }

  uint32_t discardedCommandsCount() const {
    return 0; // No discarded commands in mock
  }

  SPIConnection& spiConnection() {
    // Return a reference to a dummy SPIConnection object
    static SPIConnection dummySPIConnection(false);
    return dummySPIConnection;
  }

  void copyLastRxPayload(uint8_t* destinationBuffer, size_t maxBytes) const {
    if (destinationBuffer == nullptr || maxBytes == 0) {
      return;
    }
    // Copy the latest command payload to the destination buffer
    size_t bytesToCopy = std::min(maxBytes, sizeof(CommandPayload));
    memcpy(destinationBuffer, &m_newCommand, bytesToCopy);
  }


 private:
  mutable bool m_newCommandReady = false;
  mutable CommandPayload m_newCommand;

};


#endif // MOCK_OBC_CONNECTION_H