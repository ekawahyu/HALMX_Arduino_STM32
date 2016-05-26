#define LED1  PC6

int dataIn;

void setup() {
  pinMode(LED1, OUTPUT);
  Serial.begin(9600); // baudrate is not actually used
}

void loop() {
  digitalWrite(LED1, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(380);              // wait for a second
  digitalWrite(LED1, LOW);  // turn the LED off by making the voltage LOW
  delay(768);              // wait for a second
  
  Serial.println("Hello USBSerial on Arduino");
  delay(100);
  dataIn = Serial.read();
  Serial.println(dataIn, DEC);
}
