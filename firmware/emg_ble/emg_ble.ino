#include <Arduino.h>
#include <SPI.h>
#include <ArduinoBLE.h>

// Explicit declaration prevents Arduino's automatic prototype generator
// from using AdsReadResult before the type has been declared.
enum class AdsReadResult : uint8_t {
  NoData,
  ValidSample,
  InvalidFrame
};

AdsReadResult readChannel4Sample(int32_t& channel4);

/*
 * Nykin EMG: ADS1299-4 Channel 4 acquisition + BLE streaming
 *
 * Board/package assumptions:
 *   - Board: Seeed Studio XIAO nRF52840
 *   - Board package: "Seeed nRF52 mbed-enabled Boards"
 *   - Library: official ArduinoBLE library
 *
 * The application uses only fixed-size storage during normal operation.
 * ArduinoBLE itself may allocate its GATT objects internally during startup.
 *
 * Do not connect electrodes or a person during initial firmware validation.
 */

// ------------------------------------------------------------
// User-selectable compile-time settings
// ------------------------------------------------------------
constexpr bool USE_INTERNAL_SQUARE_WAVE = false;
// Used only when USE_INTERNAL_SQUARE_WAVE is false.
constexpr bool ENABLE_BIAS = false;
constexpr bool ENABLE_USB_SAMPLE_OUTPUT = true;
constexpr bool ENABLE_STREAM_DIAGNOSTICS = false;

// ------------------------------------------------------------
// General constants
// ------------------------------------------------------------
constexpr uint32_t SERIAL_BAUD_RATE = 115200;
constexpr uint16_t SAMPLE_RATE_HZ = 250;
constexpr uint8_t SAMPLES_PER_BLE_PACKET = 4;
constexpr size_t BLE_PACKET_BYTES = 20;
constexpr size_t STATUS_PACKET_BYTES = 20;
constexpr size_t RING_BUFFER_SIZE = 128;

constexpr uint32_t STATUS_UPDATE_INTERVAL_MS = 1000;
constexpr uint32_t DIAGNOSTIC_INTERVAL_MS = 5000;
constexpr uint32_t PARTIAL_PACKET_TIMEOUT_MS = 40;
constexpr uint32_t MIN_BLE_ATTEMPT_INTERVAL_US = 1000;

constexpr char BLE_DEVICE_NAME[] = "Nykin-EMG";
constexpr char BLE_SERVICE_UUID[] =
    "c91d0001-4c82-4b1f-ae27-92b8c429fc01";
constexpr char BLE_SAMPLE_CHARACTERISTIC_UUID[] =
    "c91d0002-4c82-4b1f-ae27-92b8c429fc01";
constexpr char BLE_STATUS_CHARACTERISTIC_UUID[] =
    "c91d0003-4c82-4b1f-ae27-92b8c429fc01";
constexpr uint8_t BLE_PROTOCOL_VERSION = 1;

static_assert(BLE_PACKET_BYTES ==
                  4 + (SAMPLES_PER_BLE_PACKET * sizeof(int32_t)),
              "BLE packet layout must remain exactly 20 bytes");
static_assert(RING_BUFFER_SIZE >= SAMPLES_PER_BLE_PACKET,
              "Ring buffer must hold at least one BLE packet");

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
constexpr uint8_t CMD_RDATA  = 0x12;

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
constexpr uint8_t REG_BIAS_SENSP = 0x0D;
constexpr uint8_t REG_BIAS_SENSN = 0x0E;

constexpr size_t ADS1299_FRAME_BYTES = 15;

// ADS1299 requires CPOL = 0, CPHA = 1.
SPISettings adsSpiSettings(1000000, MSBFIRST, SPI_MODE1);

// ------------------------------------------------------------
// BLE GATT objects
// ------------------------------------------------------------
BLEService emgService(BLE_SERVICE_UUID);

// Fixed 20-byte value. BLERead is useful for generic BLE debugging tools;
// BLENotify is used by the Python receiver for streaming.
BLECharacteristic sampleCharacteristic(
    BLE_SAMPLE_CHARACTERISTIC_UUID,
    BLERead | BLENotify,
    BLE_PACKET_BYTES,
    true);

// Binary 20-byte status snapshot, readable by a central.
BLECharacteristic statusCharacteristic(
    BLE_STATUS_CHARACTERISTIC_UUID,
    BLERead,
    STATUS_PACKET_BYTES,
    true);

// ------------------------------------------------------------
// Fixed-size sample ring buffer
// ------------------------------------------------------------
int32_t sampleRing[RING_BUFFER_SIZE];
size_t ringHead = 0;   // next write position
size_t ringTail = 0;   // oldest sample position
size_t ringCount = 0;

// ------------------------------------------------------------
// Runtime state and diagnostics
// ------------------------------------------------------------
bool bleReady = false;
bool previousBleConnected = false;
bool previousBleSubscribed = false;

uint16_t packetSequence = 0;
uint32_t totalValidSamples = 0;
uint32_t invalidAdsFrames = 0;
uint32_t droppedSamples = 0;
uint32_t bleNotificationFailures = 0;

uint32_t lastValidSampleMillis = 0;
uint32_t lastStatusUpdateMillis = 0;
uint32_t lastDiagnosticMillis = 0;
uint32_t lastBleAttemptMicros = 0;

// DRDY is captured on its falling edge so exactly one read is requested per
// conversion. If BLE or USB work lasts longer than one 4 ms sample period,
// later edges are counted rather than allowing a partially overwritten RDATAC
// frame to appear as a large spike.
volatile bool adsSamplePending = false;
volatile uint32_t drdyInterruptCount = 0;
volatile uint32_t drdyOverrunCount = 0;

void onAdsDrdyFalling() {
  ++drdyInterruptCount;

  if (adsSamplePending) {
    ++drdyOverrunCount;
  }

  adsSamplePending = true;
}

bool claimPendingAdsSample() {
  noInterrupts();
  const bool pending = adsSamplePending;
  adsSamplePending = false;
  interrupts();
  return pending;
}

uint32_t getDrdyOverrunCount() {
  noInterrupts();
  const uint32_t value = drdyOverrunCount;
  interrupts();
  return value;
}

// ------------------------------------------------------------
// Utility functions
// ------------------------------------------------------------
void printHexByte(uint8_t value) {
  Serial.print("0x");
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void writeUint16LE(uint8_t* destination, uint16_t value) {
  destination[0] = static_cast<uint8_t>(value & 0xFFU);
  destination[1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
}

void writeUint32LE(uint8_t* destination, uint32_t value) {
  destination[0] = static_cast<uint8_t>(value & 0xFFUL);
  destination[1] = static_cast<uint8_t>((value >> 8) & 0xFFUL);
  destination[2] = static_cast<uint8_t>((value >> 16) & 0xFFUL);
  destination[3] = static_cast<uint8_t>((value >> 24) & 0xFFUL);
}

void writeInt32LE(uint8_t* destination, int32_t value) {
  writeUint32LE(destination, static_cast<uint32_t>(value));
}

// ------------------------------------------------------------
// ADS1299 low-level communication
// ------------------------------------------------------------
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

void normalOperation() {
  // Stop conversions while configuring.
  digitalWrite(PIN_ADS_START, LOW);

  // Ensure register commands are accepted.
  sendCommand(CMD_SDATAC);

  writeRegister(REG_CONFIG3, ENABLE_BIAS ? 0xEC : 0xE0);
  writeRegister(REG_CONFIG1, 0x96);  // Keep existing 250-SPS stream timing
  writeRegister(REG_CONFIG2, 0xC0);  // Internal test generator disabled

  writeRegister(REG_CH1SET, 0x81);   // Powered down, internally shorted
  writeRegister(REG_CH2SET, 0x81);
  writeRegister(REG_CH3SET, 0x81);
  writeRegister(REG_CH4SET, 0x50);   // Enabled, gain 12, normal electrode input

  // Channel 4 contributes to the BIAS common-mode feedback only when enabled.
  writeRegister(REG_BIAS_SENSP, ENABLE_BIAS ? 0x08 : 0x00);
  writeRegister(REG_BIAS_SENSN, ENABLE_BIAS ? 0x08 : 0x00);

  /*
   * Internal-reference startup time is specified as 150 ms.
   * Use 200 ms conservatively.
   */
  delay(200);

  // Clear any stale edge before conversions begin.
  noInterrupts();
  adsSamplePending = false;
  interrupts();

  // Start continuous conversions using the physical START pin. Keep the ADS
  // in SDATAC mode; each DRDY event is read with the RDATA command. RDATA
  // snapshots the latest completed conversion and is not corrupted if a new
  // DRDY edge occurs while the frame is being clocked out.
  digitalWrite(PIN_ADS_START, HIGH);

  Serial.print("Beginning Channel 4 normal electrode stream; BIAS ");
  Serial.println(ENABLE_BIAS ? "enabled" : "disabled");
}

void squareWaveTesting() {
  digitalWrite(PIN_ADS_START, LOW);
  sendCommand(CMD_SDATAC);

  writeRegister(REG_CONFIG3, 0xE0);  // Internal reference enabled
  writeRegister(REG_CONFIG1, 0x96);  // 250 SPS with the existing setup
  writeRegister(REG_CONFIG2, 0xD0);  // Internal test signal enabled

  writeRegister(REG_CH1SET, 0x81);
  writeRegister(REG_CH2SET, 0x81);
  writeRegister(REG_CH3SET, 0x81);
  writeRegister(REG_CH4SET, 0x05);   // Gain 1, internal test signal

  delay(200);

  noInterrupts();
  adsSamplePending = false;
  interrupts();

  // Remain in SDATAC mode and read each completed conversion with RDATA.
  digitalWrite(PIN_ADS_START, HIGH);

  Serial.println("Beginning Channel 4 internal square-wave stream (RDATA mode)");
}

int32_t signExtend24(uint8_t msb, uint8_t middle, uint8_t lsb) {
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

AdsReadResult readChannel4Sample(int32_t& channel4) {
  // Consume at most one request for each DRDY falling edge.
  if (!claimPendingAdsSample()) {
    return AdsReadResult::NoData;
  }

  uint8_t frame[ADS1299_FRAME_BYTES];

  beginAdsTransaction();

  /*
   * Use command-based RDATA instead of RDATAC. In RDATA mode the command
   * snapshots the latest completed conversion into the output shift register.
   * TI specifies that the following read may overlap the next DRDY occurrence
   * without corrupting the frame. The byte received while sending CMD_RDATA is
   * discarded; the next 15 bytes are status + Channels 1-4.
   */
  SPI.transfer(CMD_RDATA);

  for (size_t i = 0; i < ADS1299_FRAME_BYTES; ++i) {
    frame[i] = SPI.transfer(0x00);
  }

  endAdsTransaction();

  // Preserve the existing status-byte alignment validation.
  if ((frame[0] & 0xF0) != 0xC0) {
    return AdsReadResult::InvalidFrame;
  }

  channel4 = signExtend24(frame[12], frame[13], frame[14]);
  return AdsReadResult::ValidSample;
}

void printRegister(const char* name, uint8_t address, uint8_t expected) {
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

// ------------------------------------------------------------
// Ring-buffer operations
// ------------------------------------------------------------
void pushSampleToRing(int32_t sample) {
  if (ringCount == RING_BUFFER_SIZE) {
    // Drop the oldest sample so the ring always contains the newest data.
    ringTail = (ringTail + 1) % RING_BUFFER_SIZE;
    --ringCount;
    ++droppedSamples;
  }

  sampleRing[ringHead] = sample;
  ringHead = (ringHead + 1) % RING_BUFFER_SIZE;
  ++ringCount;
}

int32_t peekRingSample(size_t offset) {
  const size_t index = (ringTail + offset) % RING_BUFFER_SIZE;
  return sampleRing[index];
}

void removeSamplesFromRing(size_t count) {
  if (count > ringCount) {
    count = ringCount;
  }

  ringTail = (ringTail + count) % RING_BUFFER_SIZE;
  ringCount -= count;
}

void discardBufferedSamples() {
  // Old data collected before subscription should not delay the live stream.
  droppedSamples += static_cast<uint32_t>(ringCount);
  ringTail = ringHead;
  ringCount = 0;
}

// ------------------------------------------------------------
// BLE setup and status
// ------------------------------------------------------------
void buildStatusPacket(uint8_t packet[STATUS_PACKET_BYTES]) {
  for (size_t i = 0; i < STATUS_PACKET_BYTES; ++i) {
    packet[i] = 0;
  }

  uint8_t flags = 0;
  if (bleReady) {
    flags |= 0x01;
  }
  if (bleReady && BLE.connected()) {
    flags |= 0x02;
  }
  if (bleReady && sampleCharacteristic.subscribed()) {
    flags |= 0x04;
  }
  if (USE_INTERNAL_SQUARE_WAVE) {
    flags |= 0x08;
  }

  packet[0] = BLE_PROTOCOL_VERSION;
  packet[1] = flags;
  writeUint16LE(&packet[2], SAMPLE_RATE_HZ);
  writeUint32LE(&packet[4], droppedSamples + getDrdyOverrunCount());
  writeUint32LE(&packet[8], invalidAdsFrames);
  writeUint32LE(&packet[12], totalValidSamples);
  writeUint16LE(&packet[16], static_cast<uint16_t>(ringCount));
  writeUint16LE(&packet[18], packetSequence);
}

void updateStatusCharacteristic() {
  if (!bleReady) {
    return;
  }

  uint8_t statusPacket[STATUS_PACKET_BYTES];
  buildStatusPacket(statusPacket);
  statusCharacteristic.writeValue(statusPacket, sizeof(statusPacket));
}

bool initializeBle() {
  Serial.println();
  Serial.println("BLE initialization:");

  if (!BLE.begin()) {
    Serial.println("BLE initialization result: FAIL");
    Serial.println("ADS1299 acquisition will continue without BLE.");
    return false;
  }

  BLE.setLocalName(BLE_DEVICE_NAME);
  BLE.setDeviceName(BLE_DEVICE_NAME);
  BLE.setAdvertisedService(emgService);

  emgService.addCharacteristic(sampleCharacteristic);
  emgService.addCharacteristic(statusCharacteristic);
  BLE.addService(emgService);

  uint8_t emptyPacket[BLE_PACKET_BYTES] = {0};
  emptyPacket[0] = BLE_PROTOCOL_VERSION;
  sampleCharacteristic.writeValue(emptyPacket, sizeof(emptyPacket));

  bleReady = true;
  updateStatusCharacteristic();

  if (!BLE.advertise()) {
    bleReady = false;
    Serial.println("BLE initialization result: FAIL (advertising did not start)");
    Serial.println("ADS1299 acquisition will continue without BLE.");
    return false;
  }

  Serial.println("BLE initialization result: PASS");
  Serial.print("BLE device name: ");
  Serial.println(BLE_DEVICE_NAME);
  Serial.print("Service UUID: ");
  Serial.println(BLE_SERVICE_UUID);
  Serial.print("Sample characteristic UUID: ");
  Serial.println(BLE_SAMPLE_CHARACTERISTIC_UUID);
  Serial.print("Status characteristic UUID: ");
  Serial.println(BLE_STATUS_CHARACTERISTIC_UUID);

  return true;
}

void serviceBleConnectionState() {
  if (!bleReady) {
    return;
  }

  const bool connected = BLE.connected();
  const bool subscribed = connected && sampleCharacteristic.subscribed();

  if (connected != previousBleConnected) {
    if (ENABLE_STREAM_DIAGNOSTICS && Serial) {
      Serial.println(connected ? "# BLE central connected"
                               : "# BLE central disconnected");
    }

    if (!connected) {
      // Explicitly resume advertising so another connection can be made.
      BLE.advertise();
    }
  }

  if (subscribed && !previousBleSubscribed) {
    // Begin each subscription with current data rather than a stale backlog.
    discardBufferedSamples();

    if (ENABLE_STREAM_DIAGNOSTICS && Serial) {
      Serial.println("# BLE sample notifications subscribed");
    }
  } else if (!subscribed && previousBleSubscribed) {
    if (ENABLE_STREAM_DIAGNOSTICS && Serial) {
      Serial.println("# BLE sample notifications unsubscribed");
    }
  }

  previousBleConnected = connected;
  previousBleSubscribed = subscribed;
}

// ------------------------------------------------------------
// Acquisition and BLE streaming services
// ------------------------------------------------------------
bool serviceAds1299Acquisition() {
  int32_t channel4 = 0;
  const AdsReadResult result = readChannel4Sample(channel4);

  if (result == AdsReadResult::NoData) {
    return false;
  }

  if (result == AdsReadResult::InvalidFrame) {
    ++invalidAdsFrames;
    return true;
  }

  ++totalValidSamples;
  lastValidSampleMillis = millis();
  pushSampleToRing(channel4);

  // Keep one numeric value per line for Arduino Serial Plotter.
  if (ENABLE_USB_SAMPLE_OUTPUT && Serial) {
    Serial.println(channel4);
  }

  return true;
}

void buildSamplePacket(uint8_t packet[BLE_PACKET_BYTES], uint8_t sampleCount) {
  for (size_t i = 0; i < BLE_PACKET_BYTES; ++i) {
    packet[i] = 0;
  }

  packet[0] = BLE_PROTOCOL_VERSION;
  packet[1] = sampleCount;
  writeUint16LE(&packet[2], packetSequence);

  for (uint8_t i = 0; i < sampleCount; ++i) {
    writeInt32LE(&packet[4 + (i * sizeof(int32_t))], peekRingSample(i));
  }
}

void serviceBleTransmission() {
  if (!bleReady || !BLE.connected() || !sampleCharacteristic.subscribed()) {
    return;
  }

  if (ringCount == 0) {
    return;
  }

  const uint32_t nowMicros = micros();
  if (static_cast<uint32_t>(nowMicros - lastBleAttemptMicros) <
      MIN_BLE_ATTEMPT_INTERVAL_US) {
    return;
  }

  uint8_t sampleCount = 0;

  if (ringCount >= SAMPLES_PER_BLE_PACKET) {
    sampleCount = SAMPLES_PER_BLE_PACKET;
  } else {
    // Normally packets contain four samples. If valid ADS data has stopped,
    // flush the smaller remainder after a short timeout.
    const uint32_t quietTime = millis() - lastValidSampleMillis;
    if (quietTime >= PARTIAL_PACKET_TIMEOUT_MS) {
      sampleCount = static_cast<uint8_t>(ringCount);
    } else {
      return;
    }
  }

  lastBleAttemptMicros = nowMicros;

  uint8_t packet[BLE_PACKET_BYTES];
  buildSamplePacket(packet, sampleCount);

  /*
   * This is intentionally outside readChannel4Sample(). ArduinoBLE versions
   * have returned either a positive success value or the number of bytes
   * written, so any positive result is accepted as success.
   */
  const int writeResult = sampleCharacteristic.writeValue(packet, sizeof(packet));

  if (writeResult > 0) {
    removeSamplesFromRing(sampleCount);
    ++packetSequence;  // uint16_t wraps naturally after 65535.
  } else {
    // Keep the samples queued for a later retry.
    ++bleNotificationFailures;
  }
}

void serviceStatusAndDiagnostics() {
  const uint32_t now = millis();

  if (bleReady &&
      static_cast<uint32_t>(now - lastStatusUpdateMillis) >=
          STATUS_UPDATE_INTERVAL_MS) {
    lastStatusUpdateMillis = now;
    updateStatusCharacteristic();
  }

  if (ENABLE_STREAM_DIAGNOSTICS && Serial &&
      static_cast<uint32_t>(now - lastDiagnosticMillis) >=
          DIAGNOSTIC_INTERVAL_MS) {
    lastDiagnosticMillis = now;

    // Lines start with '#', but disable diagnostics for a clean Serial Plotter.
    Serial.print("# valid=");
    Serial.print(totalValidSamples);
    Serial.print(" invalid_frames=");
    Serial.print(invalidAdsFrames);
    Serial.print(" ble_buffer_drops=");
    Serial.print(droppedSamples);
    Serial.print(" drdy_overruns=");
    Serial.print(getDrdyOverrunCount());
    Serial.print(" ring=");
    Serial.print(ringCount);
    Serial.print(" ble_failures=");
    Serial.print(bleNotificationFailures);
    Serial.print(" packet_seq=");
    Serial.println(packetSequence);
  }
}

// ------------------------------------------------------------
// Arduino setup and loop
// ------------------------------------------------------------
void setup() {
  establishPowerUpState();
  Serial.begin(SERIAL_BAUD_RATE);

  Serial.println("waiting for powerup...");
  delay(5000);
  Serial.println("finished initial delay");

  // Do not wait forever because acquisition must start without USB Serial.
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
   * Revision bits 7:5 can vary, so do not require the entire byte
   * to equal one hard-coded value.
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

  // Restore the reset/default value used by the existing firmware test.
  writeRegister(REG_CONFIG2, 0xC0);

  const uint8_t restoredValue = readRegister(REG_CONFIG2);

  Serial.print("CONFIG2 after restoring 0xC0 = ");
  printHexByte(restoredValue);

  if (restoredValue == 0xC0) {
    Serial.println("  PASS");
  } else {
    Serial.println("  FAIL");
  }

  // Preserve the existing startup pause. There are no long delays after
  // normal streaming begins.
  delay(5000);

  initializeBle();

  // Capture the falling edge, not merely the DRDY low level. This prevents
  // accidental duplicate reads and lets us count samples missed while another
  // operation was still running.
  attachInterrupt(
      digitalPinToInterrupt(PIN_ADS_DRDY),
      onAdsDrdyFalling,
      FALLING);

  Serial.println();
  if (USE_INTERNAL_SQUARE_WAVE) {
    Serial.println("ADS1299 mode: internal square wave");
    squareWaveTesting();
  } else {
    Serial.print("ADS1299 mode: normal Channel 4 electrodes; BIAS ");
    Serial.println(ENABLE_BIAS ? "enabled" : "disabled");
    normalOperation();
  }

  lastValidSampleMillis = millis();
  lastStatusUpdateMillis = millis();
  lastDiagnosticMillis = millis();
}

void loop() {
  /*
   * Read the ADS first. BLE and USB work is then performed immediately after
   * a conversion has been captured, giving almost a full 4 ms before the next
   * 250-SPS DRDY edge.
   */
  const bool handledAdsEvent = serviceAds1299Acquisition();

  static uint32_t lastBleMaintenanceMillis = 0;
  const uint32_t nowMillis = millis();

  // At normal operation this runs once per sample (every 4 ms), which is more
  // than frequent enough for ArduinoBLE. The 10 ms fallback keeps BLE alive if
  // ADS conversions stop entirely.
  if (bleReady &&
      (handledAdsEvent ||
       static_cast<uint32_t>(nowMillis - lastBleMaintenanceMillis) >= 10)) {
    lastBleMaintenanceMillis = nowMillis;
    BLE.poll();
    serviceBleConnectionState();
    serviceBleTransmission();
  }

  serviceStatusAndDiagnostics();
}