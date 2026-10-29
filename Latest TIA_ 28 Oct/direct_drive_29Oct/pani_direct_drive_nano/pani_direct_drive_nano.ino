// Client for the existing SPI DAC/output-switch board, not the TIA TMUX.
// Python owns pulse timing, acquisition, charge threshold and stopping.
// Commands at 115200 baud: 0 0.8 / 0 -0.2 / 0 0 / 0 f
// Python commands stay unchanged: their leading "0" is a fixed command prefix.
// SELECTED_OUTPUT chooses the physical voltage-board output, NOT a Nano pin.
// Wiring retained from the existing direct-drive sketch:
// Nano D10 -> DAC CS; D9 -> output-switch CS; D8 -> unused CS (HIGH).
// Nano hardware SPI: D11 MOSI, D12 MISO if needed, D13 SCK.
#include <SPI.h>
#include <string.h>

// CHANGE ONLY THIS NUMBER, then upload the sketch again:
const byte SELECTED_OUTPUT = 5;  // 0=D0, 1=D1, ... 15=D15.
static_assert(SELECTED_OUTPUT < 16, "SELECTED_OUTPUT must be 0 to 15");

const byte CS_DAC = 10, CS_SW = 9, CS_UNUSED = 8;
SPISettings spi(1000000, MSBFIRST, SPI_MODE2);

// Preserve the existing two-DAC daisy-chain frame ordering and setup.
void dacFrame(byte reg6, unsigned int data6, byte reg7, unsigned int data7) {
  SPI.beginTransaction(spi);
  digitalWrite(CS_DAC, LOW);
  SPI.transfer(reg7); SPI.transfer(data7 >> 8); SPI.transfer(data7 & 255);
  SPI.transfer(reg6); SPI.transfer(data6 >> 8); SPI.transfer(data6 & 255);
  digitalWrite(CS_DAC, HIGH);
  SPI.endTransaction();
}

// Enable only the selected output switch; all other outputs stay floating.
// A zero mask opens every switch. A one-bit mask connects just one output.
void connectOutput(bool connected) {
  const uint16_t mask = connected ? (uint16_t(1) << SELECTED_OUTPUT) : 0;
  SPI.beginTransaction(spi);
  digitalWrite(CS_SW, LOW);
  SPI.transfer((byte)(mask >> 8));  // Switches D8-D15.
  SPI.transfer((byte)(mask & 255)); // Switches D0-D7.
  digitalWrite(CS_SW, HIGH);
  SPI.endTransaction();
}

// First DAC serves D0-D7; second DAC serves D8-D15.
// Each DAC uses channel registers 0x08 to 0x0F. Send NOP to the other DAC.
void setOutputCode(unsigned int code) {
  const byte reg = 0x08 + (SELECTED_OUTPUT % 8);
  if (SELECTED_OUTPUT < 8) dacFrame(reg, code, 0x00, 0);
  else dacFrame(0x00, 0, reg, code);
}

void driveOutput(unsigned int code) {
  setOutputCode(code);
  delayMicroseconds(20);
  connectOutput(true);
}

void handleCommand(const char *command) {
  // DAC codes follow the existing -2.5 to +2.5 V board conversion.
  // Actual voltage accuracy depends on that board's reference and calibration.
  if (strcmp(command, "0 0.8") == 0) driveOutput(43254);
  else if (strcmp(command, "0 -0.2") == 0) driveOutput(30147);
  else if (strcmp(command, "0 0") == 0) driveOutput(32768);
  else if (strcmp(command, "0 f") == 0) connectOutput(false);
  else { Serial.println("ERR command"); return; }
  Serial.println("OK");  // Sent only after the hardware command completes.
}

void setup() {
  digitalWrite(CS_DAC, HIGH); pinMode(CS_DAC, OUTPUT);
  digitalWrite(CS_SW, HIGH); pinMode(CS_SW, OUTPUT);
  digitalWrite(CS_UNUSED, HIGH); pinMode(CS_UNUSED, OUTPUT);
  SPI.begin();
  Serial.begin(115200);
  connectOutput(false);
  dacFrame(0x05, 0x000A, 0x05, 0x000A);
  delay(5);
  dacFrame(0x03, 0x0000, 0x03, 0x0000);
  dacFrame(0x04, 0x0000, 0x04, 0x0000);
  setOutputCode(30147); // Prepare selected output at -0.2 V, still floating.
}

void loop() {
  static char line[20];
  static byte length = 0;
  static bool overflow = false;
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (overflow) Serial.println("ERR length");
      else if (length) { line[length] = '\0'; handleCommand(line); }
      length = 0;
      overflow = false;
    } else if (!overflow) {
      if (length < sizeof(line) - 1) line[length++] = c;
      else overflow = true;
    }
  }
}
