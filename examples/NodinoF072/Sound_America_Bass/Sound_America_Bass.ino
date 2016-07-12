#include <Arduino.h>

#define LED1      PB15
#define BUZZER    PA4

uint32_t counter;

void setup() {
  Serial.begin(9600); // baudrate is not actually used
  Serial1.begin(9600, SERIAL_HALF_DUPLEX);
  Serial3.begin(1200); // bottom sensor
  Serial4.begin(1200); // top sensor
  
  pinMode(LED1, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  analogWriteResolution(12);

  toneVolume(2048);
}

void loop() {
  digitalWrite(LED1, HIGH);
  delay(100);
  digitalWrite(LED1, LOW);
  delay(100);

  tone(BUZZER, NOTE_C4, 500);
  tone(BUZZER, NOTE_C4, 750);
  tone(BUZZER, NOTE_C4, 250);
  tone(BUZZER, NOTE_C4, 500);
  tone(BUZZER, NOTE_G3, 500);
  tone(BUZZER, NOTE_D4, 750);
  tone(BUZZER, NOTE_D4, 250);
  tone(BUZZER, NOTE_D4, 500);

  tone(BUZZER, NOTE_G3, 500);
  tone(BUZZER, NOTE_D4, 500);
  tone(BUZZER, NOTE_D4, 500);
  tone(BUZZER, NOTE_G3, 500);
  tone(BUZZER, NOTE_G3, 500);
  tone(BUZZER, NOTE_C4, 500);

  tone(BUZZER, NOTE_CS4,500);
  tone(BUZZER, NOTE_D4, 500);
  tone(BUZZER, NOTE_G3, 500);
  tone(BUZZER, NOTE_C4, 750);
  tone(BUZZER, NOTE_C4, 250);
  tone(BUZZER, NOTE_C4, 500);
  tone(BUZZER, NOTE_A3, 500);
  tone(BUZZER, NOTE_D4, 750);

  tone(BUZZER, NOTE_D4, 250);
  tone(BUZZER, NOTE_D4, 500);
  tone(BUZZER, NOTE_D4, 500);
  tone(BUZZER, NOTE_D4, 500);
  tone(BUZZER, NOTE_D4, 500);
  tone(BUZZER, NOTE_D4, 500);

  tone(BUZZER, NOTE_A4, 500);
  tone(BUZZER, NOTE_G4, 500);
  tone(BUZZER, NOTE_A4, 500);
  tone(BUZZER, NOTE_B4, 500);
  tone(BUZZER, NOTE_G4, 500);
  tone(BUZZER, NOTE_G4, 750);
  tone(BUZZER, NOTE_E4, 250);
  tone(BUZZER, NOTE_E4, 500);

  tone(BUZZER, NOTE_G4, 500);
  tone(BUZZER, NOTE_G4, 750);
  tone(BUZZER, NOTE_D4, 250);
  tone(BUZZER, NOTE_D4, 500);
  tone(BUZZER, NOTE_E4, 500);
  tone(BUZZER, NOTE_F4, 500);
  
  tone(BUZZER, NOTE_G4, 500);
  tone(BUZZER, NOTE_A4, 500);
  tone(BUZZER, NOTE_B4, 500);
  tone(BUZZER, NOTE_C5, 500);
  tone(BUZZER, NOTE_C4, 1000);
  tone(BUZZER, NOTE_C4, 500);
  tone(BUZZER, NOTE_F4, 750);
  tone(BUZZER, NOTE_F4, 250);
  tone(BUZZER, NOTE_F4, 500);
  tone(BUZZER, NOTE_D4, 500);
  tone(BUZZER, NOTE_G4, 750);
  tone(BUZZER, NOTE_G4, 250);
  tone(BUZZER, NOTE_G4, 500);

  tone(BUZZER, NOTE_G4, 500);
  tone(BUZZER, NOTE_A4, 500);
  tone(BUZZER, NOTE_C5, 500);
  tone(BUZZER, NOTE_G4, 500);
  tone(BUZZER, NOTE_G3, 500);
  tone(BUZZER, NOTE_C4, 1500);
  
  delay(3000);
}
