// Client for the existing SPI DAC/output-switch board, not the TIA TMUX.
// Python owns pulse timing, acquisition, charge threshold and stopping.
// Commands at 115200 baud: 0 0.8 / 0 -0.2 / 0 0 / 0 f
// Leading "0" is a fixed Python command prefix, NOT the physical output number.
// Startup: all D0-D15 actively output 0 V after initialization finishes.
// Numeric commands change only SELECTED_OUTPUT; other outputs remain at 0 V.
// "0 f" floats ONLY SELECTED_OUTPUT; other outputs remain connected at 0 V.
// During boot/reset, software cannot guarantee 0 V before initialization finishes.
// D10 -> DAC CS; D9 -> output-switch CS; D8 -> unused CS (HIGH).
// Hardware SPI: D11 MOSI, D12 optional MISO, D13 SCK.
#include <SPI.h>
#include <string.h>

// CHANGE ONLY THIS NUMBER, then upload again. This selects a board output,
// not a Nano pin. With 5, commands such as "0 0.8" control physical D5.
const byte SELECTED_OUTPUT = 0;  // 0=D0, 1=D1, ... 15=D15.
static_assert(SELECTED_OUTPUT < 16, "SELECTED_OUTPUT must be 0 to 15");

const byte CS_DAC = 10, CS_SW = 9, CS_UNUSED = 8;
SPISettings spi(1000000, MSBFIRST, SPI_MODE2);

// First three bytes reach the farther DAC U7; last three stay in U6.
void dacFrame(byte reg6, unsigned int data6, byte reg7, unsigned int data7) {
  SPI.beginTransaction(spi);
  digitalWrite(CS_DAC, LOW);
  SPI.transfer(reg7); SPI.transfer(data7 >> 8); SPI.transfer(data7 & 255);
  SPI.transfer(reg6); SPI.transfer(data6 >> 8); SPI.transfer(data6 & 255);
  digitalWrite(CS_DAC, HIGH);
  SPI.endTransaction();
}

// Bit n=1 connects output Dn; bit n=0 opens its output switch.
// Used directly during startup to keep ALL switches open until DACs are ready.
void switchMask(uint16_t mask) {
  SPI.beginTransaction(spi);
  digitalWrite(CS_SW, LOW);
  SPI.transfer((byte)(mask >> 8));  // Switches D8-D15.
  SPI.transfer((byte)(mask & 255)); // Switches D0-D7.
  digitalWrite(CS_SW, HIGH);
  SPI.endTransaction();
}

// Numeric command: close all switches (unselected DACs already hold 0 V).
// Floating command: open only the selected switch. For D5, mask = 0xFFDF.
void connectOutput(bool connected) {
  const uint16_t mask = connected ? 0xFFFF :
      uint16_t(0xFFFF & ~(uint16_t(1) << SELECTED_OUTPUT));
  switchMask(mask);
}

// Address any channel for initialization: U6=D0-D7; U7=D8-D15.
// Channel registers are 0x08..0x0F; register 0 sends NOP to the other DAC.
void setChannelCode(byte channel, unsigned int code) {
  const byte reg = 0x08 + (channel % 8);
  if (channel < 8) dacFrame(reg, code, 0x00, 0);
  else dacFrame(0x00, 0, reg, code);
}

void setOutputCode(unsigned int code) {
  setChannelCode(SELECTED_OUTPUT, code);
}

void driveOutput(unsigned int code) {
  setOutputCode(code);
  delayMicroseconds(20); // Retain the existing DAC/amplifier settling delay.
  connectOutput(true);
}

void handleCommand(const char *command) {
  // Preserve the working board conversion: Vout = 2*Vdac - 2.5.
  // 32768 is 0 V; code 0 would produce -2.5 V. Calibration remains unchanged.
  if (strcmp(command, "0 0.8") == 0) driveOutput(43254);
  else if (strcmp(command, "0 -0.2") == 0) driveOutput(30147);
  else if (strcmp(command, "0 0") == 0) driveOutput(32768);
  else if (strcmp(command, "0 f") == 0) connectOutput(false);
  else { Serial.println("ERR command"); return; }
  Serial.println("OK"); // Same acknowledgement expected by Python.
}

void setup() {
  digitalWrite(CS_DAC, HIGH); pinMode(CS_DAC, OUTPUT);
  digitalWrite(CS_SW, HIGH); pinMode(CS_SW, OUTPUT);
  digitalWrite(CS_UNUSED, HIGH); pinMode(CS_UNUSED, OUTPUT);
  SPI.begin();
  Serial.begin(115200);
  switchMask(0); // Temporarily isolate outputs while resetting/configuring DACs.
  dacFrame(0x05, 0x000A, 0x05, 0x000A);
  delay(5);
  dacFrame(0x03, 0x0000, 0x03, 0x0000);
  dacFrame(0x04, 0x0000, 0x04, 0x0000);
  // Initialize EVERY DAC channel, not just the selected one, to 0 V.
  for (byte channel = 0; channel < 16; ++channel)
    setChannelCode(channel, 32768);
  delayMicroseconds(100);
  connectOutput(true); // All outputs now actively drive 0 V and wait for Python.
  // No startup print: preserve the existing Python serial protocol.
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
