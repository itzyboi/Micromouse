// Pin Definitions
// Motors defined whilst looking from rear of robot.
#define mL 10 // Left motor drive
#define mR 11 // Right motor drive
#define enableML 9// Left motor enable pin (High is on)
#define enableMR 12 // Right motor enable pin (High is on)

#define encoderLB 2 // Left motor encoder blue
#define encoderLG 6 // Left motor encoder green
#define encoderRB 3 // Right motor encoder blue
#define encoderRG 7 // Right motor encoder green

// Constants
#define squareWidth 500
#define degrees90C 175
#define degrees90A 202 


// Global variables
volatile int encoderLBCount = 0;
volatile int encoderLGCount = 0;
volatile int encoderRBCount = 0;
volatile int encoderRGCount = 0;

void setup()
{
//Serial.begin(9600);
pinMode(enableMR, OUTPUT);
pinMode(enableML, OUTPUT);
digitalWrite(enableML, LOW); // These go low so early to try to stop robot running away on startup
digitalWrite(enableMR, LOW);
pinMode(mL, OUTPUT);
pinMode(mR, OUTPUT);

pinMode(encoderLB, INPUT);
pinMode(encoderLG, INPUT);
pinMode(encoderRB, INPUT);
pinMode(encoderRG, INPUT);

interrupts();

attachInterrupt(digitalPinToInterrupt(encoderLB), encoderLBCounter, RISING);
attachInterrupt(digitalPinToInterrupt(encoderLG), encoderLGCounter, RISING);
attachInterrupt(digitalPinToInterrupt(encoderRB), encoderRBCounter, RISING);
attachInterrupt(digitalPinToInterrupt(encoderRG), encoderRGCounter, RISING);

}

void loop() 
{
delay(1000);
forward(1);
delay(250);
forward(1);
delay(250);
clockwise90(1);
delay(250);
}

void forward( int squares)
{
  //Serial.println("Forward begin");
  // Travels forward x squares, x being int squares.

  boolean flag1 = 1;
  boolean flag2 = 1;
  int distance = squareWidth * squares;
  
  analogWrite(mL, 195);
  analogWrite(mR, 191);
  encoderLBCount = 0;
  encoderLGCount = 0;
  encoderRBCount = 0;
  encoderRGCount = 0;
  digitalWrite(enableML, HIGH);
  digitalWrite(enableMR, HIGH);

  while(flag1 | flag2)
  {
    if(encoderLBCount >= distance)
      {
        digitalWrite(enableML, LOW);
        flag1 = 0;
      }
    if(encoderRBCount >= distance)
      {
        digitalWrite(enableMR, LOW);
        flag2 = 0;
      }
  }
  return;
}

void clockwise90( int turns)
{ 
  boolean flag1 = 1;
  boolean flag2 = 1;

  int degreestoturn = degrees90C * turns;
  
  analogWrite(mL, 168);
  analogWrite(mR, 95);
  encoderLBCount = 0;
  encoderLGCount = 0;
  encoderRBCount = 0;
  encoderRGCount = 0;
  digitalWrite(enableML, HIGH);
  digitalWrite(enableMR, HIGH);
  
  while(flag1 | flag2)
  {
    if(encoderLBCount >= degreestoturn)
      {
        digitalWrite(enableML, LOW);
        flag1 = 0;
      }
    if(encoderRBCount >= degreestoturn)
      {
        digitalWrite(enableMR, LOW);
        flag2 = 0;
      }
  }
  return;
}

void antiClockwise90( int turns)
{ 
  boolean flag1 = 1;
  boolean flag2 = 1;

  int degreestoturn = degrees90C * turns;
  
  analogWrite(mL, 95);
  analogWrite(mR, 168);
  encoderLBCount = 0;
  encoderLGCount = 0;
  encoderRBCount = 0;
  encoderRGCount = 0;
  digitalWrite(enableML, HIGH);
  digitalWrite(enableMR, HIGH);
  
  while(flag1 | flag2)
  {
    if(encoderLBCount >= degreestoturn)
      {
        digitalWrite(enableML, LOW);
        flag1 = 0;
      }
    if(encoderRBCount >= degreestoturn)
      {
        digitalWrite(enableMR, LOW);
        flag2 = 0;
      }
  }
  return;
}

void encoderLBCounter( void )
{
encoderLBCount++;
}

void encoderLGCounter( void )
{
encoderLGCount++;
}

void encoderRBCounter( void )
{
encoderRBCount++;
}

void encoderRGCounter( void )
{
encoderRGCount++;
}
