void setup() {
  // Start serial communication at 9600 baud rate
  Serial.begin(9600);
  
  // Set pin 8 as an output pin
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    // Read the incoming byte from the serial port
    char receivedChar = Serial.read();

    // If '1' is received, turn pin 8 ON
    if (receivedChar == '1') {
      digitalWrite(8, LOW);  // Turn pin 8 ON (HIGH)
    }
    // If '0' is received, turn pin 8 OFF
    else if (receivedChar == '0') {
      digitalWrite(8, HIGH);  // Turn pin 8 OFF (LOW)
    }
  }
}
