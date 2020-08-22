#define ENC_COUNT_REV 374
#define motorA0 12 //kanal A alttaki pin
#define motorA1 9 //kanal A üstteki pin
#define PWM 3 //pwm
#define ENC_IN 2 //interrupt pini
    //int hiz=255;
    int speedcontrol = 200; //pot değerleri ile hız (analog)
    volatile long encoderValue = 0; //gelen pulse sayısını kaydeder
    int interval = 1000; //1sn lik aralık

    long previousMillis = 0; //önceki zaman değeri
    long currentMillis = 0; //şimdiki zaman değeri

    int rpm = 0; //dakikadaki devir sayısı

    int motorPwm = 0; //motor hızı

void setup() {
  // Setup Serial Monitor
  Serial.begin(9600); 
  // Set encoder as input with internal pullup  
  pinMode(ENC_IN, INPUT_PULLUP);
  // Set PWM and DIR connections as outputs
  //pinMode(PWM, OUTPUT);
  //pinMode(DIR, OUTPUT);
  // Attach interrupt 
  attachInterrupt(digitalPinToInterrupt(ENC_IN), updateEncoder, RISING);
  // Setup initial values for timer
  previousMillis = millis();

  //motor girişleri belirlendi    
  pinMode(motorA0, OUTPUT); //Initiates Motor Channel A pin
  pinMode(motorA1, OUTPUT); //Initiates Brake Channel A pin


}

void loop() {

  digitalWrite(motorA0, HIGH); //Establishes forward direction of Channel A
  digitalWrite(motorA1, LOW);   //Disengage the Brake for Channel A
  analogWrite(PWM, speedcontrol);   //Spins the motor on Channel A at full speed
  
  // Control motor with potentiometer
    motorPwm = map(analogRead(speedcontrol), 0, 1023, 0, 255);
    
    // Write PWM to controller
    analogWrite(PWM, motorPwm);
    // Update RPM value every second
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
 
 
    // Calculate RPM
    rpm = (float)(encoderValue * 60 / ENC_COUNT_REV);
 
    // Only update display when there is a reading
    if (motorPwm > 0 || rpm > 0) {
      Serial.print("PWM VALUE: ");
      Serial.print(motorPwm);
      Serial.print('\t');
      Serial.print(" PULSES: ");
      Serial.print(encoderValue);
      Serial.print('\t');
      Serial.print(" SPEED: ");
      Serial.print(rpm);
      Serial.println(" RPM");
    }
    
    encoderValue = 0;
  }
}
 
  

void updateEncoder()
{
  // Increment value for each pulse from encoder
  encoderValue++;
}
