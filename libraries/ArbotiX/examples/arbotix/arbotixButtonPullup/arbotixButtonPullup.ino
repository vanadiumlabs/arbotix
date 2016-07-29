/***************************
 * ArbotiX Internal Pullup
 ***************************/


const int OUTPUT_PIN = 0;
const int INPUT_PIN  = 1;

int rawAnalog;
int servoPosition; // position from control servo


void setup()
{
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(INPUT_PIN, INPUT_PULLUP);

}

void loop()
{
   if(digitalRead(INPUT_PIN) == LOW)
   {
    digitalWrite(OUTPUT_PIN, HIGH);
   }
   else
   {
    digitalWrite(OUTPUT_PIN, LOW);
   }
 
 
}




