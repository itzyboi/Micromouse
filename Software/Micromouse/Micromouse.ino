// Pin Definitions
// Motor pins - defined whilst looking from rear of robot.
#define mL 10 // Left motor drive
#define mR 11 // Right motor drive
#define enableML 9// Left motor enable pin (High is on)
#define enableMR 12 // Right motor enable pin (High is on)
// Encoder pins
#define encoderLB 2 // Left motor encoder blue pin
#define encoderLG 6 // Left motor encoder green pin
#define encoderRB 3 // Right motor encoder blue pin
#define encoderRG 7 // Right motor encoder green pin
// IR pins
#define FIR A1 // Front IR sensor on ADC 0
#define LIR A2 // Left IR sensor on ADC 1
#define RIR A0 // Right IR sensor on ADC 2
#define IROn 4 // IR emitter pin
// LED pins
#define red 7
#define yellow 6
#define green 5

// Constants
#define squareWidth 520
#define degrees90 177

// Orientation constants
#define north 0
#define east  1
#define south 2
#define west  3

// Global variables
// Setup
byte state = 0;

// Movement
volatile int encoderLBCount = 0; // Contains encoder value for left motor blue wired encoder
volatile int encoderLGCount = 0; // Contains encoder value for left motor green wired encoder
volatile int encoderRBCount = 0; // Contains encoder value for right motor blue wired encoder
volatile int encoderRGCount = 0; // Contains encoder value for right motor green wired encoder
volatile boolean comparisonFlag = 1; // flag used to check comparisons not interrupted

// DFA
byte stackPointer = 0;
byte stack[36][2] = {0}; // Stack for previously visited nodes
boolean maze[8][8] = {0}; // visited matrix
byte edgeMatrix[8][8][4] = {0}; // Matrix to hold wall values

byte x = 0; // x maze coordinate
byte y = 0; // y maze coordinate
byte orientation = 0; // Current orientation of the robot, 0 = north, 1 = east, 2 = south, 3 = west

void setup()
{
  // Setup
  pinMode(enableMR, OUTPUT);
  pinMode(enableML, OUTPUT);
  digitalWrite(enableML, LOW); // These go low so early to try to stop robot running away on startup
  digitalWrite(enableMR, LOW);
  pinMode(mL, OUTPUT);
  pinMode(mR, OUTPUT);
  pinMode(13, OUTPUT); // Pin powering IR sensor recievers
  pinMode(IROn, OUTPUT); // Pin powering IR sensor Emitters
  digitalWrite(13, HIGH);
  digitalWrite(IROn, LOW);

  pinMode(encoderLB, INPUT);
  pinMode(encoderLG, INPUT);
  pinMode(encoderRB, INPUT);
  pinMode(encoderRG, INPUT);

  pinMode(red, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(green, OUTPUT);

  interrupts();

  attachInterrupt(digitalPinToInterrupt(encoderLB), encoderLBCounter, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderLG), encoderLGCounter, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRB), encoderRBCounter, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRG), encoderRGCounter, RISING);

  // Code
  Serial.begin(9600);
  delay(1000);
  
}

void loop()
{
  
  switch (state)
  {
    case 0:
      {
        digitalWrite(red, HIGH);
        if (leftSensor() > 150)
        {
          state = 1;
        }
        break;
      }
    case 1:
      {
        if (leftSensor() < 100)
        {
          digitalWrite(red, LOW);
          state = 2;
        }
        break;
      }
    case 2:
      {
        digitalWrite(yellow, HIGH);
        if (rightSensor() > 130)
        {
          delay(1000);
          DFA();
          digitalWrite(yellow, LOW);
          state = 4;
        }
        else if (leftSensor() > 130)
        {
          state = 3;
        }
        break;
      }
    case 3:
      {
        if (leftSensor() < 100)
        {
          digitalWrite(yellow, LOW);
          state = 4;
        }
        break;
      }
    case 4:
      {
        digitalWrite(green, HIGH);
        if (rightSensor() > 130)
        {
          // inset A* here
        }
        else if (leftSensor() > 130)
        {
          state = 5;
        }
        break;
      }
    case 5:
      {
        if (leftSensor() < 100)
        {
          digitalWrite(green, LOW);
          state = 0;
        }
        break;
      }
  }
}

//DFA
void DFA()
{
  int findX = 0;
  int findY = 0;
  boolean flag = 0;
  
  while (!flag)
  {
    Serial.println();
    if (maze[x][y] == 0) // if maze node is unvisited
    {
      sensorRead();
      maze[x][y] = 1; // set maze node to visited
    }
    Serial.println();
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(" y: ");
    Serial.println(y);
    Serial.print("visited? ");
    Serial.println(maze[x][y]);
    Serial.print("north wall? ");
    Serial.println(edgeMatrix[x][y][north]);
    Serial.print("east wall? ");
    Serial.println(edgeMatrix[x][y][east]);
    Serial.print("south wall? ");
    Serial.println(edgeMatrix[x][y][south]);
    Serial.print("west wall? ");
    Serial.println(edgeMatrix[x][y][west]);
    
    if ((edgeMatrix[x][y][north] == 1) && (maze[x + 1][y] == 0)) // if north is a path and unvisited move there
    {
      turn(north);
      orientation = north;
      forward(1);
      x = x + 1;
      stackPointer++;
      stack[stackPointer][0] = x;
      stack[stackPointer][1] = y;
    }
    else if ((edgeMatrix[x][y][east] == 1) && (maze[x][y + 1] == 0)) // if east is a path and unvisited move there
    {
      turn(east);
      orientation = east;
      forward(1);
      y = y + 1;
      stackPointer++;
      stack[stackPointer][0] = x;
      stack[stackPointer][1] = y;
    }
    else if ((edgeMatrix[x][y][south] == 1) && (maze[x - 1][y] == 0)) // if south is a path and unvisited move there
    {
      turn(south);
      orientation = south;
      forward(1);
      x = x - 1;
      stackPointer++;
      stack[stackPointer][0] = x;
      stack[stackPointer][1] = y;
    }
    else if ((edgeMatrix[x][y][west] == 1) && (maze[x][y - 1] == 0)) // if west is a path and unvisited move there
    {
      turn(west);
      orientation = west;
      forward(1);
      y = y - 1;
      stackPointer++;
      stack[stackPointer][0] = x;
      stack[stackPointer][1] = y;
    }
    else // If there is no direction unsearched go back to previous node, revise orientation setup to use stacked coords
    {
      stackPointer--;
      findX = x - stack[stackPointer][0];
      findY = y - stack[stackPointer][1];
      
      Serial.print("x(-1): ");
      Serial.print(stack[stackPointer][0]);
      Serial.print(" y(-1): ");
      Serial.println(stack[stackPointer][1]);

      if (findX == -1)
      {
        turn(north);
        orientation = north;
        forward(1);
        x = stack[stackPointer][0];
        y = stack[stackPointer][1];
      }
      else if (findX == 1)
      {
        turn(south);
        orientation = south;
        forward(1);
        x = stack[stackPointer][0];
        y = stack[stackPointer][1];
      }
      else if (findY == -1)
      {
        turn(east);
        orientation = east;
        forward(1);
        x = stack[stackPointer][0];
        y = stack[stackPointer][1];
      }
      else if (findY == 1)
      {
        turn(west);
        orientation = west;
        forward(1);
        x = stack[stackPointer][0];
        y = stack[stackPointer][1];
      }
    }
  }
}

void addEdge(int bearingToChange) // adds the edges
{
  Serial.print("bearingToChange: ");
  Serial.println(bearingToChange);
  edgeMatrix[x][y][bearingToChange] = 1;
  if (bearingToChange == north)
  {
    edgeMatrix[x + 1][y][south] = 1;
  }
  else if (bearingToChange == east)
  {
    edgeMatrix[x][y + 1][west] = 1;
  }
  else if (bearingToChange == south)
  {
    edgeMatrix[x - 1][y][north] = 1;
  }
  else if (bearingToChange == west)
  {
    edgeMatrix[x][y - 1][east] = 1;
  }
}

void sensorRead()
{
  int bearingToChange;
  
  if (frontSensor() < 16)
  {
    Serial.print("adding front edge: ");
    Serial.println(orientation);
    addEdge(orientation);
  }
    
  if (leftSensor() < 16)
  {
    Serial.println("Adding left edge");
    bearingToChange = orientation - 1;
    if (bearingToChange == -1)
    {
      bearingToChange = 3;
    } 
    addEdge(bearingToChange);
  }
  if (rightSensor() < 16)
  {
    Serial.println("Adding right edge");
    bearingToChange = orientation + 1;
    if (bearingToChange == 4)
    {
      bearingToChange = 0;
    }
    Serial.print("bearing to change: ");
    Serial.println(bearingToChange);
    addEdge(bearingToChange);
  }
}

int frontSensor()
{
  digitalWrite(IROn, LOW);
  delay(50);
  int x = analogRead(FIR);
  delay(50);
  digitalWrite(IROn, HIGH);
  delay(50);
  int y = analogRead(FIR);
  digitalWrite(IROn, LOW);
  Serial.print("FIR");
  Serial.println(y - x);
  return (y - x);
}

int leftSensor()
{
  digitalWrite(IROn, LOW);
  delay(50);
  int x = analogRead(LIR);
  delay(50);
  digitalWrite(IROn, HIGH);
  delay(50);
  int y = analogRead(LIR);
  digitalWrite(IROn, LOW);
  Serial.print("LIR");
  Serial.println(y - x);
  return (y - x);
}

int rightSensor()
{
  digitalWrite(IROn, LOW);
  delay(50);
  int x = analogRead(RIR);
  delay(50);
  digitalWrite(IROn, HIGH);
  delay(50);
  int y = analogRead(RIR);
  digitalWrite(IROn, LOW);
  Serial.print("RIR");
  Serial.println(y - x);
  return (y - x);
}

//Movement
void turn( byte bearing) // bearing = 0(north),1(east),2(south),3(west)
{
  int turns = orientation - bearing;
  Serial.print("turns prefix: ");
  Serial.println(turns);
  if (turns == 0)
  {
    return;
  }
  else if (turns == 3)
  {
    turns = -1;
  }
  else if (turns == -3)
  {
    turns = 1;
  }

  Serial.print("turns postfix: ");
  Serial.println(turns);
  if (turns > 0)
  {
    antiClockwise90(turns);
  }
  if (turns < 0)
  {
    clockwise90(abs(turns));
  }
  return;
}


void forward( int squares)
{
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
      comparisonFlag = 1;
      if ((encoderLBCount >= squareWidth) && (flag1 == 1))
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableML, LOW);
          flag1 = 0;
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= squareWidth) && (flag2 == 1))
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableMR, LOW);
          flag2 = 0;
        }
      }
      comparisonFlag = 1;
      if ((encoderLBCount >= (squareWidth - 25)) && (flag1 == 1))
      {
        if (comparisonFlag == 1)
        {
          analogWrite(mL, 155);
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= (squareWidth - 25)) && (flag2 == 1))
      {
        if (comparisonFlag == 1)
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
      comparisonFlag = 1;
      if ((encoderLBCount >= degrees90) && (flag1 == 1))
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableML, LOW);
          flag1 = 0;
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= degrees90) && ( flag2 == 1))
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableMR, LOW);
          flag2 = 0;
        }
      }
      comparisonFlag = 1;
      if ((encoderLBCount >= (degrees90 - 25)) && (flag1 == 1))
      {
        if (comparisonFlag == 1)
        {
          analogWrite(mL, 155);
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= (degrees90 - 25)) && (flag2 == 1))
      {
        if (comparisonFlag == 1)
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
      comparisonFlag = 1;
      if ((encoderLBCount >= degrees90) && (flag1 == 1))
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableML, LOW);
          flag1 = 0;
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= degrees90) && (flag2 == 1))
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableMR, LOW);
          flag2 = 0;
        }
      }
      comparisonFlag = 1;
      if ((encoderLBCount >= (degrees90 - 25)) && (flag1 == 1))
      {
        if (comparisonFlag == 1)
        {
          analogWrite(mL, 99);
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= (degrees90 - 25)) && (flag2 == 1))
      {
        if (comparisonFlag == 1)
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
  comparisonFlag = 0;
}

void encoderLGCounter( void )
{
  encoderLGCount++;
  comparisonFlag = 0;
}

void encoderRBCounter( void )
{
  encoderRBCount++;
  comparisonFlag = 0;
}

void encoderRGCounter( void )
{
  encoderRGCount++;
  comparisonFlag = 0;
}
