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

  toneVolume(512);
}

void loop() {
  digitalWrite(LED1, HIGH);
  delay(100);
  digitalWrite(LED1, LOW);
  delay(100);

  tone(BUZZER, NOTE_C5, 500);
  tone(BUZZER, NOTE_C5, 750);
  tone(BUZZER, NOTE_G4, 250);
  tone(BUZZER, NOTE_G4, 500);
  tone(BUZZER, NOTE_C5, 500);
  tone(BUZZER, NOTE_B4, 750);
  tone(BUZZER, NOTE_G4, 250);
  tone(BUZZER, NOTE_G4, 500);
  
  tone(BUZZER, NOTE_G4, 500);
  tone(BUZZER, NOTE_B4, 500);
  tone(BUZZER, NOTE_B4, 500);
  tone(BUZZER, NOTE_D5, 500);
  tone(BUZZER, NOTE_D5, 500);
  tone(BUZZER, NOTE_C5, 500);

  tone(BUZZER, NOTE_AS4,500);
  tone(BUZZER, NOTE_C5, 500);
  tone(BUZZER, NOTE_B4, 500);
  tone(BUZZER, NOTE_C5, 750);
  tone(BUZZER, NOTE_G4, 250);
  tone(BUZZER, NOTE_G4, 500);
  tone(BUZZER, NOTE_C5, 500);
  tone(BUZZER, NOTE_B4, 750);

  tone(BUZZER, NOTE_B4, 250);
  tone(BUZZER, NOTE_B4, 500);
  tone(BUZZER, NOTE_B4, 500);
  tone(BUZZER, NOTE_B4, 500);
  tone(BUZZER, NOTE_B4, 500);
  tone(BUZZER, NOTE_C5, 500);

  tone(BUZZER, NOTE_C5, 500);
  tone(BUZZER, NOTE_B4, 500);
  tone(BUZZER, NOTE_C5, 500);
  tone(BUZZER, NOTE_D5, 500);
  tone(BUZZER, NOTE_G4, 500);
  tone(BUZZER, NOTE_C5, 750);
  tone(BUZZER, NOTE_C5, 250);
  tone(BUZZER, NOTE_C5, 500);

  tone(BUZZER, NOTE_C5, 500);
  tone(BUZZER, NOTE_D5, 750);
  tone(BUZZER, NOTE_D5, 250);
  tone(BUZZER, NOTE_D5, 500);
  tone(BUZZER, NOTE_C5, 500);
  tone(BUZZER, NOTE_B4, 500);
  
  tone(BUZZER, NOTE_B4, 500);
  tone(BUZZER, NOTE_C5, 500);
  tone(BUZZER, NOTE_D5, 500);
  tone(BUZZER, NOTE_C5, 500);
  tone(BUZZER, NOTE_G4, 500);
  tone(BUZZER, NOTE_A4, 500);
  tone(BUZZER, NOTE_AS4,500);
  tone(BUZZER, NOTE_A4, 750);
  tone(BUZZER, NOTE_A4, 250);
  tone(BUZZER, NOTE_A4, 500);
  tone(BUZZER, NOTE_A4, 500);
  tone(BUZZER, NOTE_G4, 750);
  tone(BUZZER, NOTE_C5, 250);
  tone(BUZZER, NOTE_C5, 500);

  tone(BUZZER, NOTE_G4, 500);
  tone(BUZZER, NOTE_A4, 500);
  tone(BUZZER, NOTE_C5, 500);
  tone(BUZZER, NOTE_G4, 500);
  tone(BUZZER, NOTE_B4, 500);
  tone(BUZZER, NOTE_C5, 1500);
  
  delay(3000);
}
