#include <Arduino.h>
#include <SPI.h>

/*
 * ADS1299 EMPTY-FOOTPRINT ROUTING TEST
 *
 * Board: Seeed Studio XIAO nRF52840
 *
 * IMPORTANT:
 * - ADS1299 must NOT be installed for the output/input-pull tests.
 * - Disconnect electrodes and external analog inputs.
 * - Upload using USB, then disconnect USB before powering from the
 *   battery-input bench supply.
 */

// ============================================================
// PCB pin mapping from your schematic
// ============================================================

constexpr uint8_t PIN_ADS_PWDN  = D0;   // XIAO D0 -> ADS1299 pin 35
constexpr uint8_t PIN_ADS_CS    = D1;   // XIAO D1 -> ADS1299 pin 39
constexpr uint8_t PIN_ADS_DRDY  = D2;   // XIAO D2 <- ADS1299 pin 47
constexpr uint8_t PIN_ADS_START = D3;   // XIAO D3 -> ADS1299 pin 38
constexpr uint8_t PIN_ADS_RESET = D4;   // XIAO D4 -> ADS1299 pin 36

// D7 is labelled SLEEP on your XIAO sheet, but its destination is not
// shown in the ADS1299 schematic. This code deliberately does not drive it.

constexpr uint8_t PIN_ADS_SCLK  = D8;   // XIAO D8 -> ADS1299 pin 40
constexpr uint8_t PIN_ADS_DOUT  = D9;   // XIAO D9 <- ADS1299 pin 43
constexpr uint8_t PIN_ADS_DIN   = D10;  // XIAO D10 -> ADS1299 pin 34

// ============================================================
// Choose ONE test, upload, and probe the associated pad
// ============================================================

enum TestMode : uint8_t {
  TEST_ALL_LOW       = 0,
  TEST_PWDN_TOGGLE   = 1,
  TEST_CS_TOGGLE     = 2,
  TEST_START_TOGGLE  = 3,
  TEST_RESET_TOGGLE  = 4,
  TEST_DRDY_TRACE    = 5,
  TEST_DOUT_TRACE    = 6,
  TEST_SPI_BURST     = 7,
  TEST_OPERATING_IDLE = 8
};

// Change only this line:
constexpr TestMode TEST_MODE = TEST_SPI_BURST;

// ADS1299 uses SPI mode 1:
// CPOL = 0, CPHA = 1.
// Start conservatively at 1 MHz.
SPISettings adsSpiSettings(1000000, MSBFIRST, SPI_MODE1);

// ============================================================
// Basic pin-state functions
// ============================================================

void configureOutputs() {
  pinMode(PIN_ADS_PWDN, OUTPUT);
  pinMode(PIN_ADS_CS, OUTPUT);
  pinMode(PIN_ADS_START, OUTPUT);
  pinMode(PIN_ADS_RESET, OUTPUT);
  pinMode(PIN_ADS_SCLK, OUTPUT);
  pinMode(PIN_ADS_DIN, OUTPUT);

  // These are ADS1299 outputs and must remain nRF inputs.
  pinMode(PIN_ADS_DRDY, INPUT);
  pinMode(PIN_ADS_DOUT, INPUT);
}

void setAllLow() {
  SPI.end();
  configureOutputs();

  digitalWrite(PIN_ADS_PWDN, LOW);
  digitalWrite(PIN_ADS_CS, LOW);
  digitalWrite(PIN_ADS_START, LOW);
  digitalWrite(PIN_ADS_RESET, LOW);
  digitalWrite(PIN_ADS_SCLK, LOW);
  digitalWrite(PIN_ADS_DIN, LOW);
}

void setOperatingIdle() {
  SPI.end();
  configureOutputs();

  // Normal inactive/idle state after supplies have stabilized.
  digitalWrite(PIN_ADS_PWDN, HIGH);   // ADS active
  digitalWrite(PIN_ADS_RESET, HIGH);  // not held in reset
  digitalWrite(PIN_ADS_START, LOW);   // conversions stopped
  digitalWrite(PIN_ADS_CS, HIGH);     // SPI deselected
  digitalWrite(PIN_ADS_SCLK, LOW);    // mode-1 clock idles low
  digitalWrite(PIN_ADS_DIN, LOW);     // quiet MOSI

  pinMode(PIN_ADS_DRDY, INPUT);
  pinMode(PIN_ADS_DOUT, INPUT);
}

void togglePin(uint8_t pin) {
  digitalWrite(pin, LOW);
  delay(500);

  digitalWrite(pin, HIGH);
  delay(500);
}

/*
 * DOUT and DRDY are normally outputs from the ADS1299.
 *
 * Since the ADS is currently missing, these traces would otherwise float.
 * Alternating the nRF's internal pull-down and pull-up lets you confirm
 * continuity from the nRF all the way to the empty ADS pad.
 */
void inputTraceTest(uint8_t pin) {
  pinMode(pin, INPUT_PULLDOWN);
  delay(2000);

  pinMode(pin, INPUT_PULLUP);
  delay(2000);
}

void sendSpiTestBurst() {
  /*
   * Valid ADS1299-style sequence:
   *
   * 0x11 = SDATAC
   * 0x20 = RREG starting at register 0x00
   * 0x00 = read one register
   * 0x00 = dummy byte to generate readback clocks
   *
   * With the ADS absent, there will be no valid MISO response.
   */

  SPI.beginTransaction(adsSpiSettings);

  digitalWrite(PIN_ADS_CS, LOW);
  delayMicroseconds(10);

  SPI.transfer(0x11);  // SDATAC
  delayMicroseconds(10);

  SPI.transfer(0x20);  // RREG ID
  SPI.transfer(0x00);  // one register
  delayMicroseconds(10);

  SPI.transfer(0x00);  // dummy clocks

  // ADS1299 requires at least four master-clock periods before CS rises.
  delayMicroseconds(10);

  digitalWrite(PIN_ADS_CS, HIGH);
  SPI.endTransaction();
}

void setup() {
  // No Serial wait, so the test runs from battery/bench power without USB.

  switch (TEST_MODE) {
    case TEST_ALL_LOW:
      setAllLow();
      break;

    case TEST_PWDN_TOGGLE:
    case TEST_CS_TOGGLE:
    case TEST_START_TOGGLE:
    case TEST_RESET_TOGGLE:
      setOperatingIdle();
      break;

    case TEST_DRDY_TRACE:
    case TEST_DOUT_TRACE:
      setOperatingIdle();
      break;

    case TEST_SPI_BURST:
      setOperatingIdle();

      // Hardware SPI uses D8/D9/D10 on the XIAO.
      SPI.begin();

      pinMode(PIN_ADS_CS, OUTPUT);
      digitalWrite(PIN_ADS_CS, HIGH);

      pinMode(PIN_ADS_PWDN, OUTPUT);
      pinMode(PIN_ADS_RESET, OUTPUT);
      pinMode(PIN_ADS_START, OUTPUT);

      digitalWrite(PIN_ADS_PWDN, HIGH);
      digitalWrite(PIN_ADS_RESET, HIGH);
      digitalWrite(PIN_ADS_START, LOW);
      break;

    case TEST_OPERATING_IDLE:
      setOperatingIdle();
      break;
  }
}

void loop() {
  switch (TEST_MODE) {
    case TEST_ALL_LOW:
      // Hold continuously.
      delay(1000);
      break;

    case TEST_PWDN_TOGGLE:
      togglePin(PIN_ADS_PWDN);
      break;

    case TEST_CS_TOGGLE:
      togglePin(PIN_ADS_CS);
      break;

    case TEST_START_TOGGLE:
      togglePin(PIN_ADS_START);
      break;

    case TEST_RESET_TOGGLE:
      togglePin(PIN_ADS_RESET);
      break;

    case TEST_DRDY_TRACE:
      inputTraceTest(PIN_ADS_DRDY);
      break;

    case TEST_DOUT_TRACE:
      inputTraceTest(PIN_ADS_DOUT);
      break;

    case TEST_SPI_BURST:
      sendSpiTestBurst();
      delay(500);
      break;

    case TEST_OPERATING_IDLE:
      delay(1000);
      break;
  }
}