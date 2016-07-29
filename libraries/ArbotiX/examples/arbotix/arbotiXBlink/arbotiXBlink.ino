/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 


// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(USER_LED, OUTPUT);   // Pin 0 maps to the USER LED on the ArbotiX Robocontroller. Defined as USER_LED in the library  
}

// the loop routine runs over and over again forever:
void loop() {
  digitalWrite(USER_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(USER_LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);               // wait for a second
}
