#include <Arduino.h>
#include <SPI.h>

/*
 * ADS1299-4 FIRST COMMUNICATION TEST
 *
 * Board:
 *   Seeed Studio XIAO nRF52840
 *
 * Test purpose:
 *   1. Follow a conservative ADS1299 wake/reset sequence.
 *   2. Stop RDATAC mode.
 *   3. Read and validate identification/default registers.
 *   4. Test both register writing and register reading.
 *   5. Repeatedly read ID so SPI can be captured on an oscilloscope.
 *
 * Do not connect electrodes or a person during this test.
 */

// ------------------------------------------------------------
// PCB pin mapping
// ------------------------------------------------------------
constexpr uint8_t PIN_ADS_PWDN  = D0;   // XIAO -> ADS1299 pin 35
constexpr uint8_t PIN_ADS_CS    = D1;   // XIAO -> ADS1299 pin 39
constexpr uint8_t PIN_ADS_DRDY  = D2;   // ADS1299 -> XIAO, pin 47
constexpr uint8_t PIN_ADS_START = D3;   // XIAO -> ADS1299 pin 38
constexpr uint8_t PIN_ADS_RESET = D4;   // XIAO -> ADS1299 pin 36

constexpr uint8_t PIN_ADS_SCLK  = D8;   // XIAO SCK  -> ADS1299 pin 40
constexpr uint8_t PIN_ADS_DOUT  = D9;   // ADS DOUT  -> XIAO MISO
constexpr uint8_t PIN_ADS_DIN   = D10;  // XIAO MOSI -> ADS DIN

// ------------------------------------------------------------
// ADS1299 commands
// ------------------------------------------------------------
constexpr uint8_t CMD_SDATAC = 0x11;
constexpr uint8_t CMD_RDATAC = 0x10;

// ------------------------------------------------------------
// ADS1299 registers
// ------------------------------------------------------------
constexpr uint8_t REG_ID      = 0x00;
constexpr uint8_t REG_CONFIG1 = 0x01;
constexpr uint8_t REG_CONFIG2 = 0x02;
constexpr uint8_t REG_CONFIG3 = 0x03;
constexpr uint8_t REG_LOFF    = 0x04;
constexpr uint8_t REG_CH1SET  = 0x05;
constexpr uint8_t REG_CH2SET  = 0x06;
constexpr uint8_t REG_CH3SET  = 0x07;
constexpr uint8_t REG_CH4SET  = 0x08;

// ADS1299 requires CPOL = 0, CPHA = 1.
SPISettings adsSpiSettings(1000000, MSBFIRST, SPI_MODE1);

void printHexByte(uint8_t value) {
  Serial.print("0x");
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void establishPowerUpState() {
  // Preload output states before switching pins to outputs.
  // TI recommends keeping digital inputs low while power stabilizes.
  digitalWrite(PIN_ADS_PWDN, LOW);
  digitalWrite(PIN_ADS_RESET, LOW);
  digitalWrite(PIN_ADS_START, LOW);
  digitalWrite(PIN_ADS_CS, LOW);
  digitalWrite(PIN_ADS_SCLK, LOW);
  digitalWrite(PIN_ADS_DIN, LOW);

  pinMode(PIN_ADS_PWDN, OUTPUT);
  pinMode(PIN_ADS_RESET, OUTPUT);
  pinMode(PIN_ADS_START, OUTPUT);
  pinMode(PIN_ADS_CS, OUTPUT);
  pinMode(PIN_ADS_SCLK, OUTPUT);
  pinMode(PIN_ADS_DIN, OUTPUT);

  // These are ADS1299 outputs. Never drive them from the XIAO.
  pinMode(PIN_ADS_DRDY, INPUT);
  pinMode(PIN_ADS_DOUT, INPUT);
}

void beginAdsTransaction() {
  SPI.beginTransaction(adsSpiSettings);

  digitalWrite(PIN_ADS_CS, LOW);

  // More than enough CS-to-SCLK setup time.
  delayMicroseconds(2);
}

void endAdsTransaction() {
  /*
   * TI requires at least four ADS master-clock cycles between the
   * final SCLK edge and CS rising.
   *
   * At 2.048 MHz:
   * 4 / 2.048 MHz = approximately 1.95 us.
   */
  delayMicroseconds(3);

  digitalWrite(PIN_ADS_CS, HIGH);

  // Keep CS high long enough to reset/separate transactions.
  delayMicroseconds(3);

  SPI.endTransaction();
}

void sendCommand(uint8_t command) {
  beginAdsTransaction();

  SPI.transfer(command);

  endAdsTransaction();
}

uint8_t readRegister(uint8_t address) {
  beginAdsTransaction();

  // RREG first byte: 001rrrrr
  SPI.transfer(0x20 | (address & 0x1F));

  // Conservative command-decode gap.
  delayMicroseconds(3);

  // Number of registers minus one: zero means read one register.
  SPI.transfer(0x00);

  delayMicroseconds(3);

  // Clock out the register value.
  const uint8_t value = SPI.transfer(0x00);

  endAdsTransaction();

  return value;
}

void writeRegister(uint8_t address, uint8_t value) {
  beginAdsTransaction();

  // WREG first byte: 010rrrrr
  SPI.transfer(0x40 | (address & 0x1F));

  delayMicroseconds(3);

  // Number of registers minus one: zero means write one register.
  SPI.transfer(0x00);

  delayMicroseconds(3);

  SPI.transfer(value);

  endAdsTransaction();
}

void wakeAndResetAds() {
  /*
   * By this point the XIAO and regulators have started.
   * Give the rails a little additional time in the all-low state.
   */
  delay(20);

  // Normal inactive state for CS.
  digitalWrite(PIN_ADS_CS, HIGH);

  // Enable the external SPI peripheral.
  SPI.begin();

  // Exit ADS power-down and release RESET.
  digitalWrite(PIN_ADS_PWDN, HIGH);
  digitalWrite(PIN_ADS_RESET, HIGH);

  /*
   * External clock is 2.048 MHz.
   * Datasheet tPOR is 2^18 clock periods = 128 ms.
   * Use 250 ms for conservative first bring-up.
   */
  delay(250);

  // Hardware reset pulse. Minimum is 2 master-clock periods.
  digitalWrite(PIN_ADS_RESET, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_ADS_RESET, HIGH);

  // Datasheet requires 18 master-clock periods after reset.
  delayMicroseconds(20);

  /*
   * The ADS1299 starts in RDATAC mode.
   * RREG and WREG are ignored until SDATAC has been sent.
   */
  sendCommand(CMD_SDATAC);
}

void noiseTesting() {
  // Stop conversions while configuring.
  digitalWrite(PIN_ADS_START, LOW);

  // Ensure register commands are accepted.
  sendCommand(CMD_SDATAC);

  writeRegister(REG_CONFIG3, 0xE0);  // Internal 4.5 V reference enabled
  writeRegister(REG_CONFIG1, 0x96);  // 250 SPS
  writeRegister(REG_CONFIG2, 0xC0);  // Internal test generator disabled

  writeRegister(REG_CH1SET, 0x81);   // Powered down, internally shorted
  writeRegister(REG_CH2SET, 0x81);
  writeRegister(REG_CH3SET, 0x81);
  writeRegister(REG_CH4SET, 0x01);   // Enabled, gain 1, internally shorted

  /*
   * Internal-reference startup time is specified as 150 ms.
   * Use 200 ms conservatively.
   */
  delay(200);

  // Start conversions using the physical START pin.
  digitalWrite(PIN_ADS_START, HIGH);

  // Enter continuous-read mode.
  sendCommand(CMD_RDATAC);

  Serial.println("Beginning Channel 4 noise stream");
}

void squareWaveTesting(){
   digitalWrite(PIN_ADS_START, LOW);
  sendCommand(CMD_SDATAC);

  writeRegister(REG_CONFIG3, 0xE0);  // Internal reference enabled
  writeRegister(REG_CONFIG1, 0x96);  // 250 SPS
  writeRegister(REG_CONFIG2, 0xD0);  // Internal test signal enabled

  writeRegister(REG_CH1SET, 0x81);
  writeRegister(REG_CH2SET, 0x81);
  writeRegister(REG_CH3SET, 0x81);
  writeRegister(REG_CH4SET, 0x05);   // Gain 1, internal test signal

  delay(200);

  digitalWrite(PIN_ADS_START, HIGH);
  sendCommand(CMD_RDATAC);
}

int32_t signExtend24(
    uint8_t msb,
    uint8_t middle,
    uint8_t lsb
) {
  uint32_t raw =
      (static_cast<uint32_t>(msb) << 16) |
      (static_cast<uint32_t>(middle) << 8) |
      static_cast<uint32_t>(lsb);

  // If bit 23 is set, the number is negative.
  if (raw & 0x00800000UL) {
    raw |= 0xFF000000UL;
  }

  return static_cast<int32_t>(raw);
}

constexpr size_t ADS1299_FRAME_BYTES = 15;

bool readChannel4Sample(int32_t& channel4) {
  // No new conversion is available yet.
  if (digitalRead(PIN_ADS_DRDY) != LOW) {
    return false;
  }

  uint8_t frame[ADS1299_FRAME_BYTES];

  beginAdsTransaction();

  /*
   * In RDATAC mode, do not send RDATA.
   * Merely supply SCLK while keeping DIN low.
   */
  for (size_t i = 0; i < ADS1299_FRAME_BYTES; ++i) {
    frame[i] = SPI.transfer(0x00);
  }

  endAdsTransaction();

  /*
   * The upper four status bits should normally be 1100.
   * This catches obvious byte-alignment or SPI errors.
   */
  if ((frame[0] & 0xF0) != 0xC0) {
    return false;
  }

  channel4 = signExtend24(
      frame[12],
      frame[13],
      frame[14]
  );

  return true;
}

void printRegister(
    const char* name,
    uint8_t address,
    uint8_t expected
) {
  const uint8_t value = readRegister(address);

  Serial.print(name);
  Serial.print(" = ");
  printHexByte(value);

  if (value == expected) {
    Serial.println("  PASS");
  } else {
    Serial.print("  EXPECTED ");
    printHexByte(expected);
    Serial.println("  FAIL");
  }
}

void setup() {
  establishPowerUpState();
    Serial.begin(115200);

  Serial.println("waiting for powerup...");
  delay(5000);
  Serial.println("finished initial delay");

  // Do not wait forever because the program must still start without Serial.
  const uint32_t waitStart = millis();
  while (!Serial && (millis() - waitStart < 4000)) {
    delay(10);
  }

  Serial.println();
  Serial.println("ADS1299-4 communication test");
  Serial.println("--------------------------------");

  wakeAndResetAds();

  const uint8_t id = readRegister(REG_ID);

  Serial.print("ID = ");
  printHexByte(id);

  /*
   * ID bits:
   * bit 4     = 1
   * bits 3:2  = 11 for ADS1299 family
   * bits 1:0  = 00 for four-channel ADS1299-4
   *
   * Revision bits 7:5 can vary, so do not require the entire
   * byte to equal one hard-coded value.
   */
  if ((id & 0x1F) == 0x1C) {
    Serial.println("  PASS: ADS1299-4 detected");
  } else {
    Serial.println("  FAIL: ID does not identify ADS1299-4");
  }

  Serial.println();
  Serial.println("Reset-register test:");

  printRegister("CONFIG1", REG_CONFIG1, 0x96);
  printRegister("CONFIG2", REG_CONFIG2, 0xC0);
  printRegister("CONFIG3", REG_CONFIG3, 0x60);
  printRegister("LOFF   ", REG_LOFF,    0x00);
  printRegister("CH1SET ", REG_CH1SET,  0x61);
  printRegister("CH2SET ", REG_CH2SET,  0x61);
  printRegister("CH3SET ", REG_CH3SET,  0x61);
  printRegister("CH4SET ", REG_CH4SET,  0x61);

  Serial.println();
  Serial.println("Write/read/restore test on CONFIG2:");

  writeRegister(REG_CONFIG2, 0xD0);
  const uint8_t writtenValue = readRegister(REG_CONFIG2);

  Serial.print("CONFIG2 after writing 0xD0 = ");
  printHexByte(writtenValue);

  if (writtenValue == 0xD0) {
    Serial.println("  PASS");
  } else {
    Serial.println("  FAIL");
  }

  // Restore the reset/default value.
  writeRegister(REG_CONFIG2, 0xC0);

  const uint8_t restoredValue = readRegister(REG_CONFIG2);

  Serial.print("CONFIG2 after restoring 0xC0 = ");
  printHexByte(restoredValue);

  if (restoredValue == 0xC0) {
    Serial.println("  PASS");
  } else {
    Serial.println("  FAIL");
  }

  delay(5000);
   noiseTesting();
   //squareWaveTesting();

}

void loop() {
  int32_t channel4;

  if (readChannel4Sample(channel4)) {
    // One numeric value per line for Arduino Serial Plotter.
    
    Serial.println(channel4);
  }
}