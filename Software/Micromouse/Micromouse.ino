// Pin Definitions
// Motors defined whilst looking from rear of robot.
#define mL 10 // Left motor drive
#define mR 11 // Right motor drive
#define enableML 9// Left motor enable pin (High is on)
#define enableMR 12 // Right motor enable pin (High is on)

#define encoderLB 2 // Left motor encoder blue pin
#define encoderLG 6 // Left motor encoder green pin
#define encoderRB 3 // Right motor encoder blue pin
#define encoderRG 7 // Right motor encoder green pin

#define FIR A0 // Front IR sensor on ADC 0
#define LIR A1 // Left IR sensor on ADC 1
#define RIR A2 // Right IR sensor on ADC 2

// Constants
#define squareWidth 510
#define degrees90 177

// Orientation constants
#define north 0
#define east  1
#define south 2
#define west  3

// Global variables
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

byte x = 1; // x maze coordinate
byte y = 1; // y maze coordinate
byte orientation = 0; // Current orientation of the robot, 0 = north, 1 = east, 2 = south, 3 = west

void setup()
{
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

}

//DFA
void DFA()
{
  byte findX = 0;
  byte findY = 0;
  
  if(maze[x][y] == 0) // if maze node is unvisited
  {
    sensorRead();
    maze[x][y] = 1; // set maze node to visited
  }
  
  if(edgeMatrix[x][y][north] == 1 && maze[x+1][y] == 0) // if north is a path and unvisited move there
  {
    orientation = north;
    // Turn north
    // Move forward(1)
    x = x + 1;
    stackPointer = stackPointer++;
    stack[stackPointer][0] = x;
    stack[stackPointer][1] = y;
  }
  else if(edgeMatrix[x][y][east] == 1 && maze[x][y+1] == 0) // if east is a path and unvisited move there
  {
    orientation = east;
    //turn east
    // move forward(1)
    y = y + 1;  
    stackPointer = stackPointer++;
    stack[stackPointer][0] = x;
    stack[stackPointer][1] = y;
  }
  else if(edgeMatrix[x][y][south] == 1 && maze[x-1][y] == 0) // if south is a path and unvisited move there
  {
    orientation = south;
    //turn south
    // move forward(1)  
    x = x - 1;
    stackPointer = stackPointer++;
    stack[stackPointer][0] = x;
    stack[stackPointer][1] = y;
  }
  else if(edgeMatrix[x][y][west] == 1 && maze[x][y-1] == 0) // if west is a path and unvisited move there
  {
    orientation = west;
    //turn west
    // move forward(1)
    y = y - 1;
    stackPointer = stackPointer++;
    stack[stackPointer][0] = x;
    stack[stackPointer][1] = y;
  }
  else // If there is no direction unsearched go back to previous node, revise orientation setup to use stacked coords
  {
    stackPointer--;
    findX = x - stack[stackPointer][0];
    findY = y - stack[stackPointer][1];

    if(findX = -1)
      {
        turn(north);
        orientation = north;
        forward(1);
      }
    else if(findX = 1)
      {
        turn(south);
        orientation = south;
        forward(1);
      }
    else if(findY = -1)
      {
        turn(east);
        orientation = east;
        forward(1);
      }
    else if(findY = 1)
      {
        turn(west);
        orientation = west;
        forward(1);
      }  
  }
  
}

void addEdge(byte bearingToChange) // adds the edges 
{
  edgeMatrix[x][y][bearingToChange] = 1;
  if(bearingToChange == north)
  {
    edgeMatrix[x+1][y][south] = 1;
  }
  else if(bearingToChange == east)
  {
    edgeMatrix[x][y+1][west] = 1;
  }
  else if(bearingToChange == south)
  {
    edgeMatrix[x-1][y][north] = 1;
  }
  else if(bearingToChange == west)
  {
    edgeMatrix[x][y-1][east] = 1;
  }
}

void sensorRead()
{
  byte bearingToChange;
  if(frontSensor() == 1)
    addEdge(orientation);
  if(leftSensor() == 1)
  {
    bearingToChange = orientation - 1;
    if(bearingToChange = -1)
      bearingToChange = 3;
    addEdge(bearingToChange);
  }
  if(rightSensor() == 1)
  {
    bearingToChange = orientation + 1;
    if(bearingToChange = 4)
      bearingToChange = 0;
    addEdge(bearingToChange);
  }
}

boolean frontSensor()
{
  return 1;
}

boolean leftSensor()
{
  return 1;
}

boolean rightSensor()
{
  return 1;
}

//Movement
void turn( byte bearing) // bearing = 0(north),1(east),2(south),3(west)
{
  byte turns = orientation = bearing;
  if(turns = 0)
  {
    return;
  }
  else if(turns == 3)
  {
    turns = -1;
  }
  else if(turns == -3)
  {
    turns = 1;
  }
  
  if(turns > 0)
  {
    antiClockwise90(turns); 
  }
  else if(turns < 0)
  {
    clockwise90(turns);
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
