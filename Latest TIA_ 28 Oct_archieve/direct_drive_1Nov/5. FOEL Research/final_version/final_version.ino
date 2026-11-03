/*
  ACTIVE MATRIX CONTROLLER — COMMENTED VERSION
  Board: Arduino Uno or classic ATmega328PB Nano.

  WIRING (Arduino pin numbers, not board connector contact numbers)
    D10 -> CS1: both daisy-chained data DACs.
    D9  -> CS2: both daisy-chained data switch chips.
    D8  -> CS3: MCP23S08 controlling the two gate multiplexers.
    D11 -> shared MOSI: commands sent from the Arduino.
    D13 -> shared SCLK: clock pulses for all three SPI groups.
    D12 <- optional MISO; this sketch performs no readback.
    GND -> MCU-side ground, preserving the existing isolation wiring.
    X1/X2 and reserved signals are not used.

  ONLY THESE COMMANDS ARE IMPLEMENTED
    all 0.2    Store +0.2 V as the voltage for every pixel during scanning.
    all f      Store a scan pattern with every pixel skipped.
    hold 0 1 2 Immediately hold G0 at VHM and D1 at +2 V.
    hold 0 1 f Immediately hold G0 at VHM; only D1 floats, others output 0 V.
    run        Start/restart continuous scanning from G0 through G15.
    time 5     Set each row's ON duration to 5 ms; range: 1..1000 ms.
    off        Stop scanning, disable all gates, and drive D0..D15 to 0 V.

  Serial Monitor: 115200 baud; select Newline or Both NL & CR.
  Row/column numbers: 0..15. Voltage range: -2.5..+2.5 V.
  Example scan: send all 0.2, then time 5, then run, one command per line.
  Example measurement: send hold 0 1 2; measure G0 and D1; send off to end.

  HOLD AND SCAN ARE TWO DIFFERENT OPERATING MODES
    hold stops scanning and applies one gate/data selection immediately.
    For a numeric hold, other data columns actively output 0 V.
    For hold with f, only the selected data column floats; others output 0 V.
    A later hold replaces the previous selection.
    hold does not overwrite the stored all voltage or floating setting.
    run replaces the held output with the stored uniform scan pattern.
    all changes only the stored scan setting. During scanning it takes
    effect at the next row load. During hold it leaves held outputs alone.
    time changes scanning timing; it does not put a time limit on hold.
    all f does not immediately change a held output. Use hold row col f
    for immediate floating, or off for all data at 0 V with gates disabled.

  WHY THE THREE CS SIGNALS ARE NOT LOW TOGETHER
    Each CS selects a receiver for SPI bytes, not the lifetime of an output.
    CS1 programs DACs; CS2 sets switches; CS3 sets gate selection.
    Each device retains its setting when its CS returns HIGH.
    Gate and data outputs can therefore remain active together while all
    CS signals are HIGH. Only one receiver is selected for each transfer.

  GATE AND ANALOG BEHAVIOR
    A selected gate connects to VHM; VHM must be the TFT ON voltage.
    The gate multiplexers disconnect inactive gates. This code alone does
    not actively drive them to VLM. The existing hardware must return
    each inactive gate to the TFT OFF voltage.
    An open data switch means high impedance, not an actively driven 0 V.
    off disables gates and drives data to 0 V; this does not guarantee
    discharge of isolated pixels. Floating does not erase stored charge either.
    Data voltage is relative to the analog reference of the output circuit;
    it is not automatically the voltage across an electrochemical cell.

  TIMING AND STARTUP
    Before changing data, gates disconnect first, then data switches open.
    DAC settings settle before data connects; the selected gate turns on last.
    The 10 us/100 us delays and SPI settings are retained from the working
    sketch. Actual settling and gate release depend on the physical circuit.
    time is row ON duration; SPI loading adds extra time to every row/frame.
    Startup ready state: no scanning, D0..D15 connected at 0 V, gates disabled.
    Data stays at 0 V until a command changes the output connections.
    During boot/reset, before setup finishes, hardware may still float;
    software cannot guarantee 0 V before DAC/switch initialization completes.
    Power the boards before MCU reset; writes cannot verify board power.
    Serial OK confirms command processing, not measured output voltages.

  Startup revision: connect all data outputs at 0 V after DAC initialization.
  off drives all data to 0 V. Numeric hold drives other columns to 0 V.
  Only f requests a steady floating state; brief switching blanking is retained.
*/

// Arduino hardware SPI library: handles clock mode, speed and byte transfers.
#include <SPI.h>
// Numeric text conversion: strtol for integers, strtod for voltages.
#include <stdlib.h>
// Text parsing: strcmp compares commands; strtok splits words.
#include <string.h>
// isfinite rejects invalid numeric values such as NaN and infinity.
#include <math.h>

// Chip-select pins are active LOW. D10 stays an output for AVR SPI master mode.
const byte CS_DAC = 10, CS_SW = 9, CS_GATE = 8;
// Gate transfers: 100 kHz, most significant bit first, SPI mode 0.
const SPISettings GATE_SPI(100000, MSBFIRST, SPI_MODE0);
// DAC/switch transfers: 1 MHz, most significant bit first, SPI mode 2.
const SPISettings DATA_SPI(1000000, MSBFIRST, SPI_MODE2);
// Stored uniform scan voltage as a transmitted 16-bit DAC word; midpoint = 0 V.
uint16_t fillCode = 0x8000;
// fillConnected=false means skip every pixel. scanning=false means no row advances.
bool fillConnected = false, scanning = false;
// gatePort mirrors commanded GPIO bits; 0x18 disables both banks. scanRow is 0..15.
byte gatePort = 0x18, scanRow = 0;
// Row dwell in milliseconds and the millis() timestamp when the row was loaded.
unsigned long rowTime = 5, rowStarted;

// Deselect every SPI receiver before changing clock settings or selecting another.
void deselect() {
  digitalWrite(CS_DAC, HIGH);
  digitalWrite(CS_SW, HIGH);
  digitalWrite(CS_GATE, HIGH);
}
// Begin a transfer to one device group with its required SPI settings.
void beginSPI(byte cs, const SPISettings &settings) {
  deselect();
  // Apply clock speed, bit order and mode while all chip selects are HIGH.
  SPI.beginTransaction(settings);
  delayMicroseconds(10);
  // Select only this receiver; following clocked bytes belong to this transaction.
  digitalWrite(cs, LOW);
}
// Finish the selected transfer and return data transactions to the gate idle mode.
void endSPI(byte cs) {
  // End the receiver frame. Previously programmed output settings remain stored.
  digitalWrite(cs, HIGH);
  // Release the SPI transaction; on AVR this does not restore an earlier SPI mode.
  SPI.endTransaction();
  if (cs != CS_GATE) { // preserve the working gate-mode idle state
    deselect();
    // Explicitly restore mode 0 with every receiver deselected, as in the working code.
    SPI.beginTransaction(GATE_SPI);
    delayMicroseconds(10);
    SPI.endTransaction();
  }
}
// Write one MCP23S08 register; assumes hardware address A1:A0 = 00.
void gateRegister(byte reg, byte value) {
  beginSPI(CS_GATE, GATE_SPI);
  delayMicroseconds(10);
  // Three-byte frame: 0x40 = write opcode, then register address, then register data.
  SPI.transfer(0x40); SPI.transfer(reg); SPI.transfer(value);
  delayMicroseconds(10);
  endSPI(CS_GATE);
}
// Update the gate-control output latch (OLAT). No physical readback is performed.
void gateWrite(byte value) {
  // Keep GP0..GP4 only. GP0/1/2 = S0/1/2; GP3/4 = bank enables.
  gatePort = value & 0x1F;
  // 0x0A is OLAT: writes the output latch bits to the configured output pins.
  gateRegister(0x0A, gatePort);
}
// Initialize gate control using the same register order as the working sketch.
void gateInit() {
  // IOCON = 0: retain the original expander configuration.
  gateRegister(0x05, 0x00);
  // Preload disabled enables: 0x18 = binary 00011000, GP3=1 and GP4=1.
  gateWrite(0x18);
  // IODIR: 0=output, 1=input. GP0..4 outputs; reserved GP5..7 remain inputs.
  gateRegister(0x00, 0xE0);
}
// Select exactly one row. Both gate multiplexer enables are active LOW.
void gateSelect(byte row) {
  gateWrite(gatePort | 0x18); // disable both banks before changing address
  // Set the common three-bit channel address while both banks are still disabled.
  gateWrite((row % 8) | 0x18);
  delayMicroseconds(10);
  // Rows 0..7: GP3=0, GP4=1. Rows 8..15: GP3=1, GP4=0.
  // For example G0 -> 0x10, G7 -> 0x17, G8 -> 0x08, G15 -> 0x0F.
  gateWrite((row % 8) | (row < 8 ? 0x10 : 0x08));
}
// Set all 16 data switch states. Bit n=1 connects Dn; bit n=0 opens Dn.
void switches(uint16_t mask) {
  beginSPI(CS_SW, DATA_SPI);
  // Send high byte first to the farther chip (D8..15), low byte to D0..7.
  // Examples: mask 0x0002 connects only D1; 0xFFFF connects all; 0 opens all.
  SPI.transfer(mask >> 8); SPI.transfer(mask & 0xFF);
  endSPI(CS_SW);
}
// Send one six-byte daisy-chain frame to U6 (D0..7) and U7 (D8..15).
void dacFrame(byte reg6, uint16_t d6, byte reg7, uint16_t d7) {
  beginSPI(CS_DAC, DATA_SPI);
  // Farther chip U7 receives the first register/data word.
  SPI.transfer(reg7); SPI.transfer(d7 >> 8); SPI.transfer(d7 & 0xFF);
  // Nearer chip U6 receives the last register/data word. Data is high byte first.
  SPI.transfer(reg6); SPI.transfer(d6 >> 8); SPI.transfer(d6 & 0xFF);
  endSPI(CS_DAC);
}
// Program one column DAC without changing its data switch connection.
void dacSet(byte col, uint16_t code) {
  // D0..7: write U6 channel register; U7 receives register 0 (NOP).
  if (col < 8) dacFrame(0x08 + col, code, 0, 0);
  // D8..15: write U7 channel register; U6 receives NOP, preserving its settings.
  else dacFrame(0, 0, 0x08 + col - 8, code);
}
// Disconnect gates first, wait for release, then open all data switches.
void blank() {
  // Force both enable bits HIGH while preserving the current channel address.
  gateWrite(gatePort | 0x18);
  delayMicroseconds(100);
  // A zero switch mask opens every data output. DAC registers themselves retain values.
  switches(0);
}
// Stop with all gate banks disabled and every data output actively at 0 V.
// This does not overwrite the stored scan setting used by a later run command.
void zeroOutputs() {
  scanning = false;
  blank(); // disconnect gates before changing any data voltage
  for (byte col = 0; col < 16; ++col) dacSet(col, 0x8000);
  delayMicroseconds(100);
  switches(0xFFFF); // close every data switch after the zero codes settle
  delayMicroseconds(100);
}
// Immediately apply one row/column selection and leave it until off/hold/run.
void hold(byte row, byte col, uint16_t code, bool connected) {
  // Stop automatic row updates so loop() cannot replace the held output.
  scanning = false;
  blank();
  // Reapply the working gate initialization with both banks disabled.
  gateInit();
  // Clear every DAC to 0 V before connecting the new output pattern.
  for (byte c = 0; c < 16; ++c) dacSet(c, 0x8000);
  // A numeric value overrides the selected DAC; f leaves its switch open.
  if (connected) dacSet(col, code);
  delayMicroseconds(100);
  // Numeric hold: connect all data outputs. f: disconnect ONLY column col.
  // Example col=1: 0xFFFF & ~0x0002 = 0xFFFD, so only D1 floats.
  switches(connected ? 0xFFFF : uint16_t(0xFFFF & ~(uint16_t(1) << col)));
  delayMicroseconds(100);
  // Connect the chosen gate to VHM last, after data settings have settled.
  gateSelect(row);
}
// Apply the stored uniform scan pattern to the current scanRow.
void loadRow() {
  blank();
  // Only program DACs when the stored pattern requests connected data.
  if (fillConnected)
    // Every column gets the same value because all is the only scan-pattern command.
    for (byte col = 0; col < 16; ++col) dacSet(col, fillCode);
  delayMicroseconds(100);
  // Connect all columns for a voltage pattern, or disconnect all for all f.
  switches(fillConnected ? 0xFFFF : 0);
  delayMicroseconds(100);
  // Skipped rows stay disconnected; all f therefore selects no gates during run.
  if (fillConnected) gateSelect(scanRow);
  // Start counting row dwell after SPI programming and analog settling finish.
  rowStarted = millis();
}
// Parse an entire integer token and check its allowed range. &n returns the value.
bool number(const char *s, long lo, long hi, long &n) {
  char *end;
  // Read base-10 digits; end points to the first character that was not consumed.
  n = strtol(s, &end, 10);
  // Require at least one digit, no leftover characters, and a value within lo..hi.
  return end != s && !*end && n >= lo && n <= hi;
}
// Parse either f or a valid voltage, returning both DAC word and switch state.
bool voltage(const char *s, uint16_t &code, bool &connected) {
  // strcmp returns zero for equal strings; only the exact token f requests floating.
  connected = strcmp(s, "f") != 0;
  // For f, return success; midpoint code is a placeholder, not a requested 0 V drive.
  if (!connected) { code = 0x8000; return true; }
  char *end;
  // Convert a numeric token, including signed values such as -0.2.
  double v = strtod(s, &end);
  // Reject missing numbers, trailing text, nonfinite numbers and out-of-range voltages.
  if (end == s || *end || !isfinite(v) || v < -2.5 || v > 2.5) return false;
  // Keep the working circuit conversion: Vout = 2*Vdac - 2.5.
  // DAC word = 32768*(1 + Vout/2.5); adding 0.5 rounds before integer conversion.
  // -2.5 V -> 0, 0 V -> 32768, +2.5 V -> 65536 before clamping.
  // The transmitted word format is inherited from your working DAC implementation.
  unsigned long n = (unsigned long)((v / 2.5 + 1) * 32768 + 0.5);
  // Clamp the positive endpoint to the largest 16-bit word, 65535.
  code = n > 65535UL ? 65535U : (uint16_t)n;
  return true;
}
// Parse one complete line, then run exactly one of the five supported commands.
void command(char *line) {
  // Up to four tokens: hold, row, column, voltage. t stores pointers into line.
  char *t[4]; byte count = 0;
  // Split on spaces/tabs. strtok edits the buffer, replacing separators with zero bytes.
  for (char *p = strtok(line, " \t"); p; p = strtok(NULL, " \t")) {
    // Reject extra arguments before making any output changes.
    if (count == 4) { Serial.println(F("Invalid command")); return; }
    t[count++] = p;
  }
  // Ignore empty or whitespace-only lines.
  if (!count) return;
  // Temporary parsed values; successful validation occurs before they are used.
  long row, col, ms; uint16_t code; bool connected;
  // off: stop scanning, disable gates, then actively drive all data to 0 V.
  if (count == 1 && !strcmp(t[0], "off")) {
    zeroOutputs();
  // run: replace a held state and start scanning from row 0.
  } else if (count == 1 && !strcmp(t[0], "run")) {
    blank(); scanRow = 0; scanning = true; loadRow();
  // all: update only the stored uniform pattern; do not interrupt a manual hold.
  } else if (count == 2 && !strcmp(t[0], "all") && voltage(t[1], code, connected)) {
    fillCode = code; fillConnected = connected; // applied at next row load
  // time: integer row dwell in milliseconds. Does not change SPI speed or hold duration.
  } else if (count == 2 && !strcmp(t[0], "time") && number(t[1], 1, 1000, ms)) {
    rowTime = ms;
  // hold: require exactly four tokens, valid row/column indices and a valid V or f.
  } else if (count == 4 && !strcmp(t[0], "hold") &&
             number(t[1], 0, 15, row) && number(t[2], 0, 15, col) &&
             voltage(t[3], code, connected)) {
    // All arguments are validated; now apply the held selection immediately.
    hold(row, col, code, connected);
  } else { Serial.println(F("Invalid command")); return; }
  // Acknowledge accepted commands. F() keeps literal text in flash on AVR.
  Serial.println(F("OK"));
}
// Runs once after upload, reset or power-up. Initialize while disconnected,
// then actively drive all data outputs to 0 V before announcing Ready.
void setup() {
  // Initialize the three CS pins; preload HIGH before making each an output.
  for (byte pin = 8; pin <= 10; ++pin) {
    // Avoid intentionally selecting a receiver while its CS pin becomes an output.
    digitalWrite(pin, HIGH); pinMode(pin, OUTPUT);
  }
  // Enable hardware SPI and USB serial, then allow the initial startup delay.
  SPI.begin(); Serial.begin(115200); delay(20);
  // Disable gates and open data switches before resetting/programming DACs.
  gateInit(); blank();
  // Soft-reset both DACs with the working reset word; allow 5 ms afterward.
  dacFrame(0x05, 0x000A, 0x05, 0x000A); delay(5);
  // Enable the internal DAC reference using the working register configuration.
  dacFrame(0x03, 0, 0x03, 0);
  // Set the working gain configuration: DAC range 0..2.5 V before analog conversion.
  dacFrame(0x04, 0, 0x04, 0);
  // Finish with gate banks disabled and every data line connected at 0 V.
  gateInit();
  zeroOutputs();
  Serial.println(F("Ready: D0-D15 = 0 V; gates disabled."));
  Serial.println(F("Commands: all V|f, off, hold row col V|f, run, time ms"));
}
// Runs repeatedly: receive serial input and advance scan rows when due.
void loop() {
  // Persistent input buffer: 47 text characters plus terminator. No Arduino String heap.
  static char line[48]; static byte used = 0; static bool overflow = false;
  // Handle one incoming character per loop, keeping scan timing responsive.
  if (Serial.available()) {
    char c = Serial.read();
    // CR or LF ends a command. Both NL & CR works because the second empty line is ignored.
    if (c == '\r' || c == '\n') {
      // Reject the entire oversized line, so a truncated command cannot affect outputs.
      if (overflow) Serial.println(F("Command too long"));
      // Terminate the text with a zero byte, then parse it only after the full line arrives.
      else if (used) { line[used] = 0; command(line); }
      // Prepare for the next line. This does not change any output settings.
      used = 0; overflow = false;
    // Append a character while reserving one byte for the terminating zero.
    } else if (used < sizeof(line) - 1) line[used++] = c;
    // Keep rejecting characters until the end of the oversized line.
    else overflow = true;
  }
  // Advance only during run and after the programmed dwell; unsigned subtraction
  // also handles millis() counter rollover for these short time intervals.
  if (scanning && (unsigned long)(millis() - rowStarted) >= rowTime) {
    // Advance G0 -> G1 -> ... -> G15 -> G0 continuously.
    scanRow = (scanRow + 1) % 16;
    // Blank the previous outputs, load the next row, then restart its dwell timer.
    loadRow();
  }
}
