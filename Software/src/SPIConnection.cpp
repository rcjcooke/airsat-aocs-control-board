#include "SPIConnection.h"

#include <SPISlave_T4.h>

#include <string.h>

namespace {
SPISlave_T4<&SPI, SPI_8_BITS> g_spi;

uint8_t clampPayloadSize(uint8_t requestedPayloadSize) {
  if (requestedPayloadSize == 0) {
    return 1;
  }
  if (requestedPayloadSize > SPIConnection::kMaxPayloadSize) {
    return SPIConnection::kMaxPayloadSize;
  }
  return requestedPayloadSize;
}
}  // namespace

SPIConnection* SPIConnection::s_instance = nullptr;

SPIConnection::SPIConnection(bool debugEnabled, uint8_t payloadSize)
    : m_isActive(false),
      m_debugEnabled(debugEnabled),
      m_payloadSize(clampPayloadSize(payloadSize)),
      m_frameSize(static_cast<uint8_t>(m_payloadSize + kFrameOverhead)),
      m_state(State::Idle),
      m_spiInputBuffer{0},
      m_spiOutputBuffer{0},
      m_spiBufferIndex(0),
      m_frameSynced(false),
      m_lastRxPayload{0},
      m_nextTxFrame{0},
      m_interruptCalls(0),
      m_lastByteReceived(0),
      m_totalBytesReceived(0),
      m_totalFramesReceived(0),
      m_checksumFailureCount(0),
      m_syncDropCount(0),
      m_bytesLostSyncing(0),
      m_bytesReceivedInLastInterrupt(0),
      m_uncheckedInterruptDebugData(false),
      m_fcfsReceived(0),
      m_txErrorCount(0),
      m_payloadReadyHandler(nullptr) {
  buildTxFrameFromPayload(nullptr, 0);
}

void SPIConnection::begin() {
  s_instance = this;

  m_state = State::Idle;
  m_spiBufferIndex = 0;
  m_frameSynced = false;
  m_uncheckedInterruptDebugData = false;
  m_isActive = false;

  memcpy((void*)m_spiOutputBuffer, (const void*)m_nextTxFrame, m_frameSize);

  g_spi.setIERTriggerMode(true, false);
  g_spi.begin();
  g_spi.onReceive(handleMessageISR);

  if (m_debugEnabled) {
    Serial.println("[SPI] [DEBUG] Transport initialised");
  }
}

void SPIConnection::activate() {
  m_isActive = true;
  if (m_debugEnabled) {
    Serial.println("[SPI] [DEBUG] Message processing activated");
  }
}

void SPIConnection::setPayloadReadyHandler(PayloadReadyHandler handler) {
  noInterrupts();
  m_payloadReadyHandler = handler;
  interrupts();
}

void SPIConnection::setNextTxPayload(const uint8_t* payload, size_t payloadSize) {
  noInterrupts();
  buildTxFrameFromPayload(payload, payloadSize);
  interrupts();
}

void SPIConnection::copyLastRxPayload(uint8_t* destination, size_t maxBytes) const {
  if (destination == nullptr || maxBytes == 0) {
    return;
  }

  const size_t boundedSize = (maxBytes < m_payloadSize) ? maxBytes : m_payloadSize;

  noInterrupts();
  memcpy(destination, (const void*)m_lastRxPayload, boundedSize);
  interrupts();
}

uint8_t SPIConnection::payloadSize() const {
  return m_payloadSize;
}

uint8_t SPIConnection::frameSize() const {
  return m_frameSize;
}

uint32_t SPIConnection::totalBytesReceived() const {
  uint32_t totalBytes = 0;
  noInterrupts();
  totalBytes = m_totalBytesReceived;
  interrupts();
  return totalBytes;
}

uint8_t SPIConnection::syncDropCount() const {
  // 8-bit so no need for protection with interrupts
  return m_syncDropCount;
}

uint8_t SPIConnection::checksumFailureCount() const {
  // 8-bit so no need for protection with interrupts
  return m_checksumFailureCount;
}

SPIConnection::Stats SPIConnection::statsSnapshot() const {
  noInterrupts();
  const Stats snapshot = {
      m_interruptCalls,
      m_lastByteReceived,
      m_totalBytesReceived,
      m_totalFramesReceived,
      m_bytesLostSyncing,
      m_bytesReceivedInLastInterrupt,
      m_fcfsReceived,
      m_txErrorCount};
  interrupts();
  return snapshot;
}

SPIConnection::State SPIConnection::state() const {
  // 8-bit so no need for protection with interrupts
  return m_state;
}

uint16_t SPIConnection::calculateFletcher16(const uint8_t* data, size_t count) {
  uint16_t sum1 = 0;
  uint16_t sum2 = 0;

  for (size_t i = 0; i < count; ++i) {
    sum1 = static_cast<uint16_t>((sum1 + data[i]) % 255);
    sum2 = static_cast<uint16_t>((sum2 + sum1) % 255);
  }

  return static_cast<uint16_t>((sum2 << 8) | sum1);
}

void SPIConnection::buildTxFrameFromPayload(const uint8_t* payload, size_t payloadSize) {
  const size_t boundedSize = (payloadSize < m_payloadSize) ? payloadSize : m_payloadSize;

  memset(m_nextTxFrame, 0, m_frameSize);
  m_nextTxFrame[0] = kSyncByte0;
  m_nextTxFrame[1] = kSyncByte1;

  if (payload != nullptr && boundedSize > 0) {
    memcpy(&m_nextTxFrame[kSyncSize], payload, boundedSize);
  }

  const uint16_t checksum = calculateFletcher16(m_nextTxFrame, m_frameSize - kChecksumSize);
  m_nextTxFrame[m_frameSize - 2] = static_cast<uint8_t>(checksum & 0xFF);
  m_nextTxFrame[m_frameSize - 1] = static_cast<uint8_t>((checksum >> 8) & 0xFF);
}

void SPIConnection::validateAndDispatchFrame() {
  const uint16_t calculatedChecksum =
      calculateFletcher16(m_spiInputBuffer, m_frameSize - kChecksumSize);
  const uint16_t receivedChecksum =
      static_cast<uint16_t>(m_spiInputBuffer[m_frameSize - 2]) |
      (static_cast<uint16_t>(m_spiInputBuffer[m_frameSize - 1]) << 8);

  if (calculatedChecksum != receivedChecksum) {
    ++m_checksumFailureCount;
    return;
  }

  memcpy((void*)m_lastRxPayload, (const void*)&m_spiInputBuffer[kSyncSize], m_payloadSize);

  if (m_isActive) {
    // SAFE COPY TO PREVENT NULLPTR FAULTS:
    PayloadReadyHandler handlerCopy = (PayloadReadyHandler) m_payloadReadyHandler;
    if (handlerCopy != nullptr) {
      handlerCopy((const uint8_t*)m_lastRxPayload, m_payloadSize);
    }
  }

  ++m_totalFramesReceived;
}

void SPIConnection::handleMessageISR() {
  if (s_instance != nullptr) {
    s_instance->handleMessage();
  }
}

void SPIConnection::handleMessage() {
  ++m_interruptCalls;
  if (!m_uncheckedInterruptDebugData) {
    m_bytesReceivedInLastInterrupt = 0;
  }

  if (LPSPI4_SR & LPSPI_SR_FCF) {
    // Hardware Frame Complete Flag just asserted (CS went High - i.e. frame complete)
    LPSPI4_SR = LPSPI_SR_FCF; // Clear flag
    ++m_fcfsReceived;

    // Update the output buffer to the next frame to transmit.
    memcpy((void*)m_spiOutputBuffer, (const void*)m_nextTxFrame, m_frameSize);
    // Force reset tracking pointers so the next frame starts immediately
    m_spiBufferIndex = 0;
    m_frameSynced = false;

    // Pre-load the absolute first sync byte (0xAA) into the hardware register
    // So it sits on the physical MISO pin BEFORE the Pi drops CS low again
    LPSPI4_TDR = m_spiOutputBuffer[0];
  }

  while (g_spi.isDataAvailable()) {
    uint32_t rawRegValue = g_spi.popr();
    const uint8_t rxData = static_cast<uint8_t>(rawRegValue & 0xFF); // Mask strictly to the bottom 8 bits

    m_lastByteReceived = rxData;
    ++m_totalBytesReceived;

    if (!m_uncheckedInterruptDebugData) {
      ++m_bytesReceivedInLastInterrupt;
    }

    if (!m_frameSynced) {
      m_state = State::Syncing;
      
      if (m_spiBufferIndex == 0) {
        if (rxData == kSyncByte0) {
          m_spiInputBuffer[0] = rxData;
          m_spiBufferIndex = 1;
        }
        // If it's not SyncByte0, we stay at index 0 and keep hunting
      } else if (m_spiBufferIndex == 1) {
        if (rxData == kSyncByte1) {
          m_spiInputBuffer[1] = rxData;
          m_spiBufferIndex = 2;
          m_frameSynced = true; // Lock into the frame transmission
        } else {
          // We saw SyncByte0, but the next byte was NOT SyncByte1.
          // This is a verified sync failure.
          ++m_syncDropCount; 
          ++m_bytesLostSyncing;
          
          // Re-evaluate this failed byte. If it happens to be another SyncByte0,
          // keep our index at 1. Otherwise, reset back to hunting.
          if (rxData == kSyncByte0) {
            m_spiInputBuffer[0] = rxData;
            m_spiBufferIndex = 1;
          } else {
            m_spiBufferIndex = 0;
          }
        }
      }
    } else {
      // TRANSCEIVING STATE: Rely purely on counting bytes until the packet finishes
      m_state = State::Transceiving;
      m_spiInputBuffer[m_spiBufferIndex] = rxData;
      ++m_spiBufferIndex;

      if (m_spiBufferIndex >= m_frameSize) {
        // Entire packet size reached. Validate the Fletcher16 checksum.
        validateAndDispatchFrame(); 
        
        // Reset state pointers for the next payload hunt
        m_spiBufferIndex = 0;
        m_frameSynced = false; 
      }
    }
  }

  if (LPSPI4_SR & LPSPI_SR_TDF) {
    // nextTxIndex tracks the exact byte offset we need to stage next
    uint8_t nextTxIndex = m_spiBufferIndex; 
    
    if (nextTxIndex < m_frameSize) {
      LPSPI4_TDR = m_spiOutputBuffer[nextTxIndex];
    } else {
      LPSPI4_TDR = 0x00; // Fallback padding
    }
  }

  if (LPSPI4_SR & LPSPI_SR_TEF) {
    LPSPI4_SR = LPSPI_SR_TEF;
    ++m_txErrorCount;
  }

  m_uncheckedInterruptDebugData = true;

  if (LPSPI4_SR & LPSPI_SR_WCF) {
    LPSPI4_SR = LPSPI_SR_WCF;
  }
}

void SPIConnection::printTCRRegisterDetail() const {
  Serial.printf("[SPI] [DEBUG] LPSPI4_TCR: 0x%08X\r\n", LPSPI4_TCR);

  const uint32_t cpol = (LPSPI4_TCR & LPSPI_TCR_CPOL) ? 1U : 0U;
  const uint32_t cpha = (LPSPI4_TCR & LPSPI_TCR_CPHA) ? 1U : 0U;
  const uint32_t prescale = (LPSPI4_TCR & LPSPI_TCR_PRESCALE(0x07)) / LPSPI_TCR_PRESCALE(1);
  const uint32_t pcs = (LPSPI4_TCR & LPSPI_TCR_PCS(0x07)) / LPSPI_TCR_PCS(1);
  const uint32_t lsbf = (LPSPI4_TCR & LPSPI_TCR_LSBF) ? 1U : 0U;
  const uint32_t bysw = (LPSPI4_TCR & LPSPI_TCR_BYSW) ? 1U : 0U;
  const uint32_t cont = (LPSPI4_TCR & LPSPI_TCR_CONT) ? 1U : 0U;
  const uint32_t contc = (LPSPI4_TCR & LPSPI_TCR_CONTC) ? 1U : 0U;
  const uint32_t rxmsk = (LPSPI4_TCR & LPSPI_TCR_RXMSK) ? 1U : 0U;
  const uint32_t txmsk = (LPSPI4_TCR & LPSPI_TCR_TXMSK) ? 1U : 0U;
  const uint32_t width = (LPSPI4_TCR & LPSPI_TCR_WIDTH(0x03)) / LPSPI_TCR_WIDTH(1);
  const uint32_t framesz = (LPSPI4_TCR & LPSPI_TCR_FRAMESZ(0xFFFF)) / LPSPI_TCR_FRAMESZ(1);

  Serial.printf("[SPI] [DEBUG]   CPOL: %u  | CPHA: %u  | PRESCALE: %u\r\n", cpol, cpha, prescale);
  Serial.printf("[SPI] [DEBUG]   PCS: %u   | LSBF: %u  | BYSW: %u\r\n", pcs, lsbf, bysw);
  Serial.printf("[SPI] [DEBUG]   CONT: %u  | CONTC: %u | RXMSK: %u | TXMSK: %u\r\n", cont, contc, rxmsk, txmsk);
  Serial.printf("[SPI] [DEBUG]   WIDTH: %u | FRAMESZ: %u\r\n", width, framesz);
}

void SPIConnection::printSRRegisterDetail() const {
  Serial.printf("[SPI] [DEBUG] LPSPI4_SR: 0x%08X\r\n", LPSPI4_SR);
  Serial.printf("[SPI] [DEBUG]   ");
  if (LPSPI4_SR & LPSPI_SR_TDF) {
    Serial.printf("TDF ");
  }
  if (LPSPI4_SR & LPSPI_SR_RDF) {
    Serial.printf("RDF ");
  }
  if (LPSPI4_SR & LPSPI_SR_WCF) {
    Serial.printf("WCF ");
  }
  if (LPSPI4_SR & LPSPI_SR_FCF) {
    Serial.printf("FCF ");
  }
  if (LPSPI4_SR & LPSPI_SR_TCF) {
    Serial.printf("TCF ");
  }
  if (LPSPI4_SR & LPSPI_SR_TEF) {
    Serial.printf("TEF ");
  }
  if (LPSPI4_SR & LPSPI_SR_REF) {
    Serial.printf("REF ");
  }
  if (LPSPI4_SR & LPSPI_SR_DMF) {
    Serial.printf("DMF ");
  }
  if (LPSPI4_SR & LPSPI_SR_MBF) {
    Serial.printf("MBF ");
  }
  Serial.printf("\r\n");
}

void SPIConnection::printFSRRegisterDetail() const {
  Serial.printf("[SPI] [DEBUG] LPSPI4_FSR: 0x%08X\r\n", LPSPI4_FSR);

  const uint32_t txCount = (LPSPI4_FSR & LPSPI_FSR_TXCOUNT(0x1F)) / LPSPI_FSR_TXCOUNT(1);
  const uint32_t rxCount = (LPSPI4_FSR & LPSPI_FSR_RXCOUNT(0x1F)) / LPSPI_FSR_RXCOUNT(1);

  Serial.printf("[SPI] [DEBUG]   TXCOUNT: %u | RXCOUNT: %u\r\n", txCount, rxCount);
}

void SPIConnection::printRSRRegisterDetail() const {
  Serial.printf("[SPI] [DEBUG] LPSPI4_RSR: 0x%08X\r\n", LPSPI4_RSR);

  const uint32_t startOfFrame = (LPSPI4_RSR & LPSPI_RSR_SOF) ? 1U : 0U;
  const uint32_t rxEmpty = (LPSPI4_RSR & LPSPI_RSR_RXEMPTY) ? 1U : 0U;

  Serial.printf("[SPI] [DEBUG]   SOF: %u | RXEMPTY: %u\r\n", startOfFrame, rxEmpty);
}

void SPIConnection::printSPIParameterRegister() const {
  Serial.printf("[SPI] [DEBUG] LPSPI4_PARAM: 0x%08X\r\n", LPSPI4_PARAM);

  const uint32_t rxFifoSize = (LPSPI4_PARAM >> 8) & 0xFF;
  const uint32_t txFifoSize = LPSPI4_PARAM & 0xFF;

  Serial.printf("[SPI] [DEBUG]   RX MAX FIFO SIZE: %u | TX MAX FIFO SIZE: %u\r\n", 1U << rxFifoSize, 1U << txFifoSize);
}

void SPIConnection::printFCRRegisterDetail() const {
  Serial.printf("[SPI] [DEBUG] LPSPI4_FCR: 0x%08X\r\n", LPSPI4_FCR);

  const uint32_t rxWatermark = (LPSPI4_FCR & LPSPI_FCR_RXWATER(0x0F)) / LPSPI_FCR_RXWATER(1);
  const uint32_t txWatermark = (LPSPI4_FCR & LPSPI_FCR_TXWATER(0x0F)) / LPSPI_FCR_TXWATER(1);

  Serial.printf("[SPI] [DEBUG]   RX WATERMARK: %u | TX WATERMARK: %u\r\n", rxWatermark, txWatermark);
}

void SPIConnection::printIERRegisterDetail() const {
  Serial.printf("[SPI] [DEBUG] LPSPI4_IER: 0x%08X\r\n", LPSPI4_IER);
  Serial.printf("[SPI] [DEBUG]   ");

  if (LPSPI4_IER & LPSPI_IER_TDIE) {
    Serial.printf("TDIE ");
  }
  if (LPSPI4_IER & LPSPI_IER_RDIE) {
    Serial.printf("RDIE ");
  }
  if (LPSPI4_IER & LPSPI_IER_WCIE) {
    Serial.printf("WCIE ");
  }
  if (LPSPI4_IER & LPSPI_IER_FCIE) {
    Serial.printf("FCIE ");
  }
  if (LPSPI4_IER & LPSPI_IER_TCIE) {
    Serial.printf("TCIE ");
  }
  if (LPSPI4_IER & LPSPI_IER_TEIE) {
    Serial.printf("TEIE ");
  }
  if (LPSPI4_IER & LPSPI_IER_REIE) {
    Serial.printf("REIE ");
  }
  if (LPSPI4_IER & LPSPI_IER_DMIE) {
    Serial.printf("DMIE ");
  }

  Serial.printf("\r\n");
}

void SPIConnection::printCFGR1RegisterDetail() const {
  Serial.printf("[SPI] [DEBUG] LPSPI4_CFGR1: 0x%08X\r\n", LPSPI4_CFGR1);

  const uint32_t masterSlaveMode = (LPSPI4_CFGR1 & LPSPI_CFGR1_MASTER) ? 1U : 0U;
  const uint32_t samplePoint = (LPSPI4_CFGR1 & LPSPI_CFGR1_SAMPLE) ? 1U : 0U;
  const uint32_t autoPCS = (LPSPI4_CFGR1 & LPSPI_CFGR1_AUTOPCS) ? 1U : 0U;
  const uint32_t noStall = (LPSPI4_CFGR1 & LPSPI_CFGR1_NOSTALL) ? 1U : 0U;
  const uint32_t pcs0Polarity = (LPSPI4_CFGR1 & LPSPI_CFGR1_PCSPOL(0)) ? 1U : 0U;
  const uint32_t pcs1Polarity = (LPSPI4_CFGR1 & LPSPI_CFGR1_PCSPOL(1)) ? 1U : 0U;
  const uint32_t pcs2Polarity = (LPSPI4_CFGR1 & LPSPI_CFGR1_PCSPOL(2)) ? 1U : 0U;
  const uint32_t pcs3Polarity = (LPSPI4_CFGR1 & LPSPI_CFGR1_PCSPOL(3)) ? 1U : 0U;
  const uint32_t matchConfig = (LPSPI4_CFGR1 & LPSPI_CFGR1_MATCFG(0x07)) / LPSPI_CFGR1_MATCFG(1);
  const uint32_t pinConfig = (LPSPI4_CFGR1 & LPSPI_CFGR1_PINCFG(0x03)) / LPSPI_CFGR1_PINCFG(1);
  const uint32_t outConfig = (LPSPI4_CFGR1 & LPSPI_CFGR1_OUTCFG) ? 1U : 0U;
  const uint32_t pcsConfig = (LPSPI4_CFGR1 & LPSPI_CFGR1_PCSCFG) ? 1U : 0U;

  Serial.printf("[SPI] [DEBUG]   MASTER/SLAVE: %s | SAMPLE: %u | AUTOPCS: %u | NOSTALL: %u\r\n",
                masterSlaveMode ? "MASTER" : "SLAVE",
                samplePoint,
                autoPCS,
                noStall);
  Serial.printf("[SPI] [DEBUG]   PCSPOL[0-3]: %u %u %u %u | MATCFG: 0x%03X | PINCFG: 0x%02X | OUTCFG: %u | PCSCFG: %u\r\n",
                pcs0Polarity,
                pcs1Polarity,
                pcs2Polarity,
                pcs3Polarity,
                matchConfig,
                pinConfig,
                outConfig,
                pcsConfig);
}

void SPIConnection::spiRegisterAudit() const {
  const uint32_t muxCS = IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00;
  const uint32_t muxMOSI = IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02;
  const uint32_t muxMISO = IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01;
  const uint32_t muxSCK = IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03;

  const uint32_t padCS = IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00;
  const uint32_t padMOSI = IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02;
  const uint32_t padMISO = IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01;
  const uint32_t padSCK = IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03;

  const bool dirCS = (GPIO7_GDIR >> 0) & 0x01;
  const bool dirMOSI = (GPIO7_GDIR >> 2) & 0x01;
  const bool dirMISO = (GPIO7_GDIR >> 1) & 0x01;
  const bool dirSCK = (GPIO7_GDIR >> 3) & 0x01;

  Serial.println("\n================= TEENSY SPI HARDWARE BUS SILICON AUDIT =================");
  Serial.printf("PIN 10 [CS]   -> MUX (ALT): %u | GPIO Dir: %s | PAD Reg: 0x%04X ",
                muxCS & 0x07,
                dirCS ? "OUTPUT" : "INPUT",
                padCS);
  Serial.printf("(Schmitt: %s, Pulls: %s)\n",
                (padCS & (1 << 16)) ? "ON" : "OFF",
                ((padCS >> 14) & 0x03) == 0 ? "NONE" : ((padCS >> 14) & 0x03) == 1 ? "100k-DOWN" : "100k-UP");

  Serial.printf("PIN 11 [MOSI] -> MUX (ALT): %u | GPIO Dir: %s | PAD Reg: 0x%04X ",
                muxMOSI & 0x07,
                dirMOSI ? "OUTPUT" : "INPUT",
                padMOSI);
  Serial.printf("(Schmitt: %s, Pulls: %s)\n",
                (padMOSI & (1 << 16)) ? "ON" : "OFF",
                ((padMOSI >> 14) & 0x03) == 0 ? "NONE" : ((padMOSI >> 14) & 0x03) == 1 ? "100k-DOWN" : "100k-UP");

  Serial.printf("PIN 12 [MISO] -> MUX (ALT): %u | GPIO Dir: %s | PAD Reg: 0x%04X ",
                muxMISO & 0x07,
                dirMISO ? "OUTPUT" : "INPUT",
                padMISO);
  Serial.printf("(Schmitt: %s, Pulls: %s)\n",
                (padMISO & (1 << 16)) ? "ON" : "OFF",
                ((padMISO >> 14) & 0x03) == 0 ? "NONE" : ((padMISO >> 14) & 0x03) == 1 ? "100k-DOWN" : "100k-UP");

  Serial.printf("PIN 13 [SCK]  -> MUX (ALT): %u | GPIO Dir: %s | PAD Reg: 0x%04X ",
                muxSCK & 0x07,
                dirSCK ? "OUTPUT" : "INPUT",
                padSCK);
  Serial.printf("(Schmitt: %s, Pulls: %s)\n",
                (padSCK & (1 << 16)) ? "ON" : "OFF",
                ((padSCK >> 14) & 0x03) == 0 ? "NONE" : ((padSCK >> 14) & 0x03) == 1 ? "100k-DOWN" : "100k-UP");

  Serial.printf("Daisy Chains -> SDO_SELECT: 0x%X | SDI_SELECT: 0x%X\n",
                IOMUXC_LPSPI4_SDO_SELECT_INPUT,
                IOMUXC_LPSPI4_SDI_SELECT_INPUT);
  Serial.printf("LPSPI4 Status -> CR: 0x%08X | IER: 0x%08X | SR: 0x%08X\n",
                LPSPI4_CR,
                LPSPI4_IER,
                LPSPI4_SR);
  Serial.println("=========================================================================");

  printCFGR1RegisterDetail();
  printTCRRegisterDetail();
  printIERRegisterDetail();
  printSPIParameterRegister();
  printFCRRegisterDetail();

  Serial.println("=========================================================================");
}
