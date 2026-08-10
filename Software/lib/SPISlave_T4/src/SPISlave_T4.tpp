#include <SPISlave_T4.h>
#include "Arduino.h"
#include "SPI.h"

#define SLAVE_CR spiAddr[4]
#define SLAVE_FCR spiAddr[22]
#define SLAVE_IER spiAddr[6]
#define SLAVE_CFGR0 spiAddr[8]
#define SLAVE_CFGR1 spiAddr[9]
#define SLAVE_TDR spiAddr[25]
#define SLAVE_RDR spiAddr[29]
#define SLAVE_SR spiAddr[5]
#define SLAVE_TCR_REFRESH spiAddr[24] = (0UL << 27) | LPSPI_TCR_FRAMESZ(bits - 1)
#define SLAVE_PORT_ADDR volatile uint32_t *spiAddr = &(*(volatile uint32_t*)(0x40394000 + (0x4000 * _portnum)))
#define SLAVE_PINS_ADDR volatile uint32_t *spiAddr = &(*(volatile uint32_t*)(0x401F84EC + (_portnum * 0x10)))

void lpspi4_slave_isr() {
  // Defensive guard: avoid null dereference during startup/teardown races.
  if (_LPSPI4 == nullptr) {
    // Clear sticky flags and leave quickly.
    LPSPI4_SR = 0x3F00;
    asm volatile ("dsb");
    return;
  }
  _LPSPI4->SLAVE_ISR();
}

SPISlave_T4_FUNC SPISlave_T4_OPT::SPISlave_T4() {
  if ( port == &SPI ) {
    _LPSPI4 = this;
    _portnum = 3;
    // Enable the LPSI4 clock
    CCM_CCGR1 |= CCM_CCGR1_LPSPI4(CCM_CCGR_ON);
    // Default to triggering on any data available or data required
    _ierTriggerMode = LPSPI_IER_RDIE | LPSPI_IER_TDIE;
    nvic_irq = 32 + _portnum;
    _VectorsRam[16 + nvic_irq] = lpspi4_slave_isr;

    // Set up pins and muxes
    IOMUXC_LPSPI4_PCS0_SELECT_INPUT = 0x0; /* Select Pin 10 for CS */
    IOMUXC_LPSPI4_SCK_SELECT_INPUT  = 0x0; /* Select Pin 13 for CLK */
    IOMUXC_LPSPI4_SDI_SELECT_INPUT  = 0x0; /* Maps internal Data Input (SDI) back to Pin 11/12 hardware block */
    IOMUXC_LPSPI4_SDO_SELECT_INPUT  = 0x0; /* Maps internal Data Output (SDO) back to Pin 11/12 hardware block */

    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; /* Pin 10 - PCS0 (CS) */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x3; /* Pin 12 - SDI  (Data Input) */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3; /* Pin 11 - SDO  (Data Output) */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; /* Pin 13 - SCK  (Clock) */

    // Revert Electrical pad settings to match their true direction:
    // IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 = 0x0001B0B0; // Pin 10 CS Pull-up
    // IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = 0x000130B0; // Pin 13 CLK Pull-down
    
    // // Pin 12 is the REAL hardware input (SDI): Needs Pull-down and Hysteresis
    // IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01 = 0x000130B0; 

    // // Pin 11 is the REAL hardware output (SDO): Needs high drive strength, no pulls
    // IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02 = 0x000010B0; 



  } 
}

SPISlave_T4_FUNC void SPISlave_T4_OPT::swapPins(bool enable) {
  SLAVE_PORT_ADDR;
  SLAVE_CR &= ~LPSPI_CR_MEN; /* Disable Module */
  SLAVE_CFGR1 = (SLAVE_CFGR1 & 0xFCFFFFFF) | (enable) ? (3UL << 24) : (0UL << 24);
  SLAVE_CR |= LPSPI_CR_MEN; /* Enable Module */
  if ( sniffer_enabled ) sniffer();
}


SPISlave_T4_FUNC void SPISlave_T4_OPT::sniffer(bool enable) {
  SLAVE_PORT_ADDR;
  sniffer_enabled = enable;
  if ( port == &SPI ) {
    if ( sniffer_enabled ) {
      if ( SLAVE_CFGR1 & (3UL << 24) ) { /* if pins are swapped */
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; /* LPSPI4 SCK (CLK) */
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x0; /* LPSPI4 SDI (MISO) */
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3; /* LPSPI4 SDO (MOSI) */
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; /* LPSPI4 PCS0 (CS) */
      }
      else {
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; /* LPSPI4 SCK (CLK) */
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x3; /* LPSPI4 SDI (MISO) */
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x0; /* LPSPI4 SDO (MOSI) */
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; /* LPSPI4 PCS0 (CS) */
      }
    }
    else {
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; /* LPSPI4 SCK (CLK) */
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x3; /* LPSPI4 SDI (MISO) */
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3; /* LPSPI4 SDO (MOSI) */
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; /* LPSPI4 PCS0 (CS) */
    }
  }
}

SPISlave_T4_FUNC u_int32_t SPISlave_T4_OPT::getStatus() {
  SLAVE_PORT_ADDR;
  return SLAVE_SR;
}

SPISlave_T4_FUNC bool SPISlave_T4_OPT::isDataAvailable() {
  SLAVE_PORT_ADDR;
  return isDataAvailable(SLAVE_SR);
}

SPISlave_T4_FUNC bool SPISlave_T4_OPT::isDataAvailable(u_int32_t status) {
  return (status & LPSPI_SR_RDF);
}

SPISlave_T4_FUNC void SPISlave_T4_OPT::resetTxFIFO() {
  SLAVE_PORT_ADDR;
  SLAVE_CR |= LPSPI_CR_RTF; 
  asm volatile ("dsb");
}

SPISlave_T4_FUNC bool SPISlave_T4_OPT::active() {
  SLAVE_PORT_ADDR;
  return ( !(SLAVE_SR & (1UL << 9)) ) ? 1 : 0;
}


SPISlave_T4_FUNC bool SPISlave_T4_OPT::available() {
  SLAVE_PORT_ADDR;
  return ( (SLAVE_SR & (1UL << 8)) ) ? 1 : 0;
}

// SPISlave_T4_FUNC bool SPISlave_T4_OPT::isTransmitError() {
//   SLAVE_PORT_ADDR;
//   return ( (SLAVE_SR & (1UL << 11)) ) ? 1 : 0;
// }

SPISlave_T4_FUNC void SPISlave_T4_OPT::pushr(uint32_t data) {
  SLAVE_PORT_ADDR;
  SLAVE_TDR = data;
}


SPISlave_T4_FUNC uint32_t SPISlave_T4_OPT::popr() {
  SLAVE_PORT_ADDR;
  // uint32_t data = SLAVE_RDR;
  // SLAVE_SR = (1UL << 8); /* Clear WCF */
  return SLAVE_RDR;
}


SPISlave_T4_FUNC void SPISlave_T4_OPT::SLAVE_ISR() {

  // SLAVE_PORT_ADDR;

  // Copy callback pointer first so ISR does not race a mid-flight reassignment.
  _SPI_ptr handler = _spihandler;
  if (handler) {
    handler();
    // SLAVE_SR = 0x3F00;
    asm volatile ("dsb");
    return;
  }

  // while ( !(SLAVE_SR & (1UL << 9)) ) { /* FCF: Frame Complete Flag, set when PCS deasserts */
  //   if ( SLAVE_SR & (1UL << 11) ) { /* transmit error, clear flag, check cabling */
  //     SLAVE_SR = (1UL << 11);
  //     transmit_errors++;
  //   }
  //   if ( (SLAVE_SR & (1UL << 8)) ) { /* WCF set */
  //     uint32_t val = SLAVE_RDR;
  //     Serial.print(val); Serial.print(" ");
  //     SLAVE_TDR = val;
  //     SLAVE_SR = (1UL << 8); /* Clear WCF */
  //   }
  // }
  // Serial.println();
  // SLAVE_SR = 0x3F00; /* Clear remaining flags on exit */
  // asm volatile ("dsb");
}

SPISlave_T4_FUNC void SPISlave_T4_OPT::setIERTriggerMode(bool anyData, bool frameComplete) {
  SLAVE_PORT_ADDR;
  _ierTriggerMode = anyData ? LPSPI_IER_RDIE : 0; // | LPSPI_IER_TDIE
  _ierTriggerMode |= frameComplete ? LPSPI_IER_FCIE : 0;
  SLAVE_IER = _ierTriggerMode;
}

SPISlave_T4_FUNC void SPISlave_T4_OPT::begin(uint32_t initialByte) {
  // Set up Transmit Command Register (TCR) _before_ enabling LPSPI in Slave mode

  SLAVE_PORT_ADDR;
  SLAVE_CR = LPSPI_CR_RST; /* Reset Module */
  SLAVE_CR = 0; /* Disable Module */
  SLAVE_FCR = 0; // No watermarks - we want to know on 0 bytes both ways
  // BUG FIX: SLAVE_IER was incorrectly set to trigger on TDIE only
  SLAVE_IER = _ierTriggerMode; /* RX Interrupt */
  SLAVE_CFGR0 = 0;
  SLAVE_CFGR1 = 0; // Bit 0 = 0 is what sets this up as a Slave device
  SLAVE_CR |= LPSPI_CR_MEN | LPSPI_CR_DBGEN; /* Enable Module, Debug Mode */
  SLAVE_SR = 0x3F00; /* Clear status register */
  // Note: This will zero off all TCR bits except the frame size - which is ok but does mask some other configuration options
  SLAVE_TCR_REFRESH;
  SLAVE_TDR = initialByte;
  NVIC_ENABLE_IRQ(nvic_irq);
  NVIC_SET_PRIORITY(nvic_irq, 144);
}

SPISlave_T4_FUNC void SPISlave_T4_OPT::begin() {
  begin(0x0);
}