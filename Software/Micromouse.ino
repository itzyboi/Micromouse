// Pin Definitions
// Motors defined whilst looking from rear of robot.
byte mL = ; // Left motor drive
byte mR = ; // Right motor drive
byte enableML = ; // Left motor enable pin (High is on)
byte enableMR = ; // Right motor enable pin (High is on)

byte encoderL1 = ; // Left motor encoder 1 
byte encoderL2 = ; // Left motor encoder 2
byte encoderR1 = ; // Right motor encoder 1
byte encoderR2 = ; // Right motor encoeer 2

// Constant definitions
int squareWidth = 666;
int 90degrees = 127;


// Global variables
int encoderL1Count = 0;
int encoderL2Count = 0;
int encoderR1Count = 0;
int encoderR2Count = 0;

void setup()
{  
attachInterrupt(digitalPinToInterrupt(encoderL1), encoderL1Counter, HIGH);
attachInterrupt(digitalPinToInterrupt(encoderL2), encoderL2Counter, HIGH);
attachInterrupt(digitalPinToInterrupt(encoderR1), encoderR1Counter, HIGH);
attachInterrupt(digitalPinToInterrupt(encoderR2), encoderR2Counter, HIGH);
}

void loop() 
{
 
}

void forward( int squares)
{
  // Travels forward x squares, x being int squares.
  
  int distance = 0;
  int average = 0;
  
  encoderL1Count = 0;
  encoderL2Count = 0;
  encoderR1Count = 0;
  encoderR2Count = 0;

  distance = squareWidth * squares;

  analogWrite(mL, 255);
  analogWrite(mR, 255);
  digitalWrite(enableML, HIGH);
  digitalWrite(enableMR, HIGH);

  while(average < distance)
  {
    average = (encoderL1Count + encoderL2Count + encoderR1Count + encoderR2Count) / 4;
  }
  // Braking maneuver
  digitalWrite(enableML, LOW);
  digitalWrite(enableMR, LOW);
  analogWrite(mL, 0);
  analogWrite(mR, 0);
  digitalWrite(enableML, HIGH);
  digitalWrite(enableMR, HIGH);  
  delay(1);
  digitalWrite(enableML, LOW);
  digitalWrite(enableMR, LOW);
  return;
}

void 90Clockwise( int turns)
{ 
  int average = 0;
  encoderL1Count = 0;
  encoderL2Count = 0;
  encoderR1Count = 0;
  encoderR2Count = 0;

  int degreesToTurn = 90degrees * turns;

  analogWrite(mL, 255);
  analogWrite(mR, 0);
  digitalWrite(enableML, HIGH);
  digitalWrite(enableMR, HIGH);
  
  while(average < degreestoturn)
  {
    average = (encoderL1Count + encoderL2Count + encoderR1Count + encoderR2Count) / 4;
  }
  // Braking maneuver
  digitalWrite(enableML, LOW);
  digitalWrite(enableMR, LOW);
  analogWrite(mL, 0);
  analogWrite(mR, 255);
  digitalWrite(enableML, HIGH);
  digitalWrite(enableMR, HIGH);  
  delay(1);
  digitalWrite(enableML, LOW);
  digitalWrite(enableMR, LOW);
  return;
}

void 90AntiClockwise( int turns)
{ 
  int average = 0;
  encoderL1Count = 0;
  encoderL2Count = 0;
  encoderR1Count = 0;
  encoderR2Count = 0;

  int degreesToTurn = 90degrees * turns;

  analogWrite(mL, 0);
  analogWrite(mR, 255);
  digitalWrite(enableML, HIGH);
  digitalWrite(enableMR, HIGH);
  
  while(average < degreestoturn)
  {
    average = (encoderL1Count + encoderL2Count + encoderR1Count + encoderR2Count) / 4;
  }
  // Braking maneuver
  digitalWrite(enableML, LOW);
  digitalWrite(enableMR, LOW);
  analogWrite(mL, 255);
  analogWrite(mR, 0);
  digitalWrite(enableML, HIGH);
  digitalWrite(enableMR, HIGH);  
  delay(1);
  digitalWrite(enableML, LOW);
  digitalWrite(enableMR, LOW);
  return;
}

// Encoder counter ISRs

void encoderL1Counter( void )
{
encoderL1Count = encoderL1Count + 1;
}

void encoderL2Counter( void )
{
encoderL1Count = encoderL1Count + 1;
}

void encoderR1Counter( void )
{
encoderL1Count = encoderL1Count + 1;
}

void encoderR2Counter( void )
{
encoderL1Count = encoderL1Count + 1;
}
