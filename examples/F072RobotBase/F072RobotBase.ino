#include <Arduino.h>

#define LED1      PB15
#define MOTOR1A   PB0
#define MOTOR1B   PB1
#define MOTOR2A   PB8
#define MOTOR2B   PB9

int dataIn;
uint32_t counter;
uint32_t myindex;

void setup() {
  pinMode(LED1, OUTPUT);
  
  pinMode(MOTOR1A, OUTPUT);
  pinMode(MOTOR1B, OUTPUT);
  pinMode(MOTOR2A, OUTPUT);
  pinMode(MOTOR2B, OUTPUT);

  analogWrite (MOTOR1A, 2500);
  digitalWrite(MOTOR1B, LOW);

  analogWrite (MOTOR2A, 2500);
  digitalWrite(MOTOR2B, LOW);
  
  Serial.begin(9600); // baudrate is not actually used
  Serial1.begin(9600, SERIAL_HALF_DUPLEX);
  Serial3.begin(1200); // bottom sensor
  Serial4.begin(1200); // top sensor
}

void loop() {
  if (counter++ == 20) {
    counter = 0;
    myindex++;
    digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    delay(100);
    Serial1.print("S1:Hello Receiver!\n");
    Serial1.println(myindex, DEC);
    Serial4.write("S4:Hello Receiver!\n");
    Serial4.println(myindex, DEC);
  }

  dataIn = Serial4.read();
  if (dataIn >= 0)
  Serial.write(dataIn);
}
