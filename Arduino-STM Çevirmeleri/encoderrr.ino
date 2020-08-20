/*kaynak: http://forum.arduino.cc/index.php?topic=421638.0
*/
 
#define encoder0PinA  2
#define encoder0PinB  3
 
volatile int encoder0Pos = 0;
volatile boolean PastA = 0;
volatile boolean PastB = 0;
 
void setup() 
{
  Serial.begin (9600);
 
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT); 
 
  PastA = (boolean)digitalRead(encoder0PinA);
  PastB = (boolean)digitalRead(encoder0PinB);
 
  attachInterrupt(0, doEncoderA, RISING); // pin2 için kesme
  attachInterrupt(1, doEncoderB, CHANGE); // pin3 için kesme
}
 
void loop()
{
  Serial.println(encoder0Pos % 360);
  delay(100);
}
 
void doEncoderA()
{
     PastB ? encoder0Pos--:  encoder0Pos++;
}
 
void doEncoderB()
{
     PastB = !PastB; 
}
