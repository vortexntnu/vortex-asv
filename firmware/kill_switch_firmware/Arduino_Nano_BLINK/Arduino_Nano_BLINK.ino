  byte num1 = A4;
  byte num2 = A2;

void setup() {
  // initialize digital pin LED_BUILTIN as an output.

  pinMode(num1, OUTPUT);
  pinMode(num2, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(num1, HIGH);   // turn the LED on 
  digitalWrite(num2, LOW);    // turn the LED off
  delay(500);                       // wait for half a second
  digitalWrite(num1, LOW);    // turn the LED off
  digitalWrite(num2, HIGH);   // turn the LED on  
  delay(500);                       // wait for half a second
}
