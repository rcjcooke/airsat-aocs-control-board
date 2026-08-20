// TODO: Add a timeout on the OBC connection
#ifndef AIRSAT_OBC_CONNECTION_H
#define AIRSAT_OBC_CONNECTION_H

#include <Arduino.h>

#include "SPIConnection.h"
#include "AOCSControllerTelemetry.h"
#include "AOCSPacketStructures.h"

class OBCConnection {
 public:
  static constexpr uint8_t kCommandMailboxDepth = 2;

  explicit OBCConnection(bool spiDebug = false,
                         uint8_t payloadSize = static_cast<uint8_t>(sizeof(CommandPayload)));

  void begin();
  void service();
  bool hasNewCommand() const;
  CommandPayload takeLatestCommand();
  void updateTelemetry(const AOCSControllerTelemetry& telemetry);

  bool isConnected() const;
  uint32_t rxErrorCount() const;
  uint32_t totalBytesReceived() const;
  uint8_t syncDropCount() const;
  uint32_t commandCount() const;
  uint32_t noOpCount() const;
  uint32_t malformedPacketCount() const;
  uint32_t discardedCommandsCount() const;
  
  void copyLastRxPayload(uint8_t* destinationBuffer, size_t maxBytes) const;
  SPIConnection& spiConnection();

 private:
  static OBCConnection* s_instance;
  static void onPayloadReceivedCallbackISR(const uint8_t* payload, uint8_t payloadSize);

  void onPayloadReceivedISR(const uint8_t* payload, uint8_t payloadSize);
  void queueTelemetryPayload();

  SPIConnection m_spiConnection;
  CommandPayload m_commandMailbox[kCommandMailboxDepth];
  TelemetryPayload m_telemetryPayload;
  uint8_t m_commandReadIndex;
  uint8_t m_commandWriteIndex;

  bool m_newCommandReady;
  uint32_t m_commandCount;
  uint32_t m_noOpCount;
  uint32_t m_malformedPacketCount;
  uint32_t m_discardedCommandsCount;
  CommandPayload m_lastReceivedCommand;
  bool m_hasLastReceivedCommand;
};

#endif // AIRSAT_OBC_CONNECTION_H
