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
#define squareWidth 510
#define degrees90 177


// Global variables
volatile int encoderLBCount = 0;
volatile int encoderLGCount = 0;
volatile int encoderRBCount = 0;
volatile int encoderRGCount = 0;
volatile boolean comparasonFlag = 1;

void setup()
{
  Serial.begin(9600);
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


  delay(2000);
}

void loop()
{
  forward(2);
  clockwise90(1);
  forward(2);
  antiClockwise90(2);
  forward(2);
  antiClockwise90(1);
  forward(2);
  clockwise90(2);
}

void forward( int squares)
{
  //Serial.println("Forward begin");
  // Travels forward x squares, x being int squares.
  byte i = 0;
  boolean flag1 = 1;
  boolean flag2 = 1;

  for (i = 0; i < squares; i++)
  {
    analogWrite(mL, 194);
    analogWrite(mR, 191);
    encoderLBCount = 0;
    encoderLGCount = 0;
    encoderRBCount = 0;
    encoderRGCount = 0;
    flag1 = 1;
    flag2 = 1;
    digitalWrite(enableML, HIGH);
    digitalWrite(enableMR, HIGH);
    while (flag1 | flag2)
    {
      //code goes here
      comparasonFlag = 1;
      if ((encoderLBCount >= squareWidth) && (flag1 == 1))
      {
        if (comparasonFlag == 1)
        {
          digitalWrite(enableML, LOW);
          flag1 = 0;
        }
      }
      comparasonFlag = 1;
      if ((encoderRBCount >= squareWidth) && (flag2 == 1))
      {
        if (comparasonFlag == 1)
        {
          digitalWrite(enableMR, LOW);
          flag2 = 0;
        }
      }
      comparasonFlag = 1;
      if ((encoderLBCount >= (squareWidth - 25)) && (flag1 == 1))
      {
        if (comparasonFlag == 1)
        {
          analogWrite(mL, 155);
        }
      }
      comparasonFlag = 1;
      if ((encoderRBCount >= (squareWidth - 25)) && (flag2 == 1))
      {
        if (comparasonFlag == 1)
        {
          analogWrite(mR, 155);
        }
      }
    }
  }
  return;
}

void clockwise90( int turns)
{
  boolean flag1 = 1;
  boolean flag2 = 1;
  byte i = 0;

  for (i = 0; i < turns; i++)
  {
    analogWrite(mL, 168);
    analogWrite(mR, 95);
    encoderLBCount = 0;
    encoderLGCount = 0;
    encoderRBCount = 0;
    encoderRGCount = 0;
    flag1 = 1;
    flag2 = 1;
    digitalWrite(enableML, HIGH);
    digitalWrite(enableMR, HIGH);
    while (flag1 | flag2)
    {
      comparasonFlag = 1;
      if ((encoderLBCount >= degrees90) && (flag1 == 1))
      {
        if (comparasonFlag == 1)
        {
          digitalWrite(enableML, LOW);
          flag1 = 0;
        }
      }
      comparasonFlag = 1;
      if ((encoderRBCount >= degrees90) && ( flag2 == 1))
      {
        if (comparasonFlag == 1)
        {
          digitalWrite(enableMR, LOW);
          flag2 = 0;
        }
      }
      comparasonFlag = 1;
      if ((encoderLBCount >= (degrees90 - 25)) && (flag1 == 1))
      {
        if (comparasonFlag == 1)
        {
          analogWrite(mL, 155);
        }
      }
      comparasonFlag = 1;
      if ((encoderRBCount >= (degrees90 - 25)) && (flag2 == 1))
      {
        if (comparasonFlag == 1)
        {
          analogWrite(mR, 99);
        }
      }
    }
  }
  return;
}

void antiClockwise90( int turns)
{
  boolean flag1 = 1;
  boolean flag2 = 1;
  byte i = 0;

  for (i = 0; i < turns; i++)
  {
    analogWrite(mL, 95);
    analogWrite(mR, 168);
    encoderLBCount = 0;
    encoderLGCount = 0;
    encoderRBCount = 0;
    encoderRGCount = 0;
    flag1 = 1;
    flag2 = 1;
    digitalWrite(enableML, HIGH);
    digitalWrite(enableMR, HIGH);
    while (flag1 | flag2)
    {
      comparasonFlag = 1;
      if ((encoderLBCount >= degrees90) && (flag1 == 1))
      {
        if (comparasonFlag == 1)
        {
          digitalWrite(enableML, LOW);
          flag1 = 0;
        }
      }
      comparasonFlag = 1;
      if ((encoderRBCount >= degrees90) && (flag2 == 1))
      {
        if (comparasonFlag == 1)
        {
          digitalWrite(enableMR, LOW);
          flag2 = 0;
        }
      }
      comparasonFlag = 1;
      if ((encoderLBCount >= (degrees90 - 25)) && (flag1 == 1))
      {
        if (comparasonFlag == 1)
        {
          analogWrite(mL, 99);
        }
      }
      comparasonFlag = 1;
      if ((encoderRBCount >= (degrees90 - 25)) && (flag2 == 1))
      {
        if (comparasonFlag == 1)
        {
          analogWrite(mR, 155);
        }
      }
    }
  }
  return;
}

void encoderLBCounter( void )
{
  encoderLBCount++;
  comparasonFlag = 0;
}

void encoderLGCounter( void )
{
  encoderLGCount++;
  comparasonFlag = 0;
}

void encoderRBCounter( void )
{
  encoderRBCount++;
  comparasonFlag = 0;
}

void encoderRGCounter( void )
{
  encoderRGCount++;
  comparasonFlag = 0;
}
