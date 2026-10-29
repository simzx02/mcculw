const int FIRST_PIN = 22;
const int LAST_PIN  = 37;

void setup() {
  Serial.begin(115200);

  // Python에서 사용하는 22~37번 pin 설정
  for (int pin = FIRST_PIN; pin <= LAST_PIN; pin++) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
}

void loop() {
  if (Serial.available() > 0) {

    char command = Serial.read();

    // H = HIGH, L = LOW
    if (command == 'H' || command == 'L') {

      int pin = Serial.parseInt();

      if (pin >= FIRST_PIN && pin <= LAST_PIN) {

        if (command == 'H') {
          digitalWrite(pin, HIGH);
        }
        else {
          digitalWrite(pin, LOW);
        }
      }
    }

    // 남아 있는 newline 제거
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}
