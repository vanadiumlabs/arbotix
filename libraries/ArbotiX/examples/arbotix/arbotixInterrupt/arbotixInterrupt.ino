
/***************************
 * ArbotiX Interrupt
 * The hardware interrupt is 
 * only available on pin 2.
 * 
 * '2' in attachInterrupt()
 * happens to be the same  
 * number as the data pin
 *  though this is coincdence.
 *  
 *  Interrupt vectors 0/1
 *  are used by the serial
 *  port for the DYNAMIXEL
 *  chain.
 *
 * 
 ***************************/
 
const byte ledPin = 0;
const byte interruptPin = 2;
volatile byte state = LOW;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(2, blink, CHANGE);
}

void loop() 
{
  
}

void blink() 
{
  digitalWrite(ledPin, digitalRead(interruptPin));
}
