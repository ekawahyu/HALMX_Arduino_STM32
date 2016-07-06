#include <Arduino.h>

#define LED1      PB15
#define MOTOR1A   PB0
#define MOTOR1B   PB1
#define MOTOR2A   PB8
#define MOTOR2B   PB9

int dataIn;
uint32_t counter;
uint32_t myindex;
uint16_t motor1, motor2;

void setup() {
  pinMode(LED1, OUTPUT);
  
  pinMode(MOTOR1A, OUTPUT);
  pinMode(MOTOR1B, OUTPUT);
  pinMode(MOTOR2A, OUTPUT);
  pinMode(MOTOR2B, OUTPUT);

  motor1 = 0;
  analogWrite (MOTOR1B, motor1);
  digitalWrite(MOTOR1A, LOW);
  
  motor2 = 0;
  analogWrite (MOTOR2B, motor2);
  digitalWrite(MOTOR2A, LOW);
  
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
    motor1 = 2000 + ((motor1 + 100) % 1000);
    motor2 = 2000 + ((motor2 + 100) % 1000);
    analogWrite (MOTOR1B, 5000 - motor1);
    analogWrite (MOTOR2B, motor2);
    Serial1.print("S1:Hello Receiver!\n");
    Serial1.println(myindex, DEC);
    Serial4.write("S4:Hello Receiver!\n");
    Serial4.println(myindex, DEC);
  }

  dataIn = Serial4.read();
  if (dataIn >= 0)
  Serial.write(dataIn);
}
