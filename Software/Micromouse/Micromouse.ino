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

// Orientation constants
#define north 0
#define east  1
#define south 2
#define west  3

// Global variables
// Movement
volatile int encoderLBCount = 0;
volatile int encoderLGCount = 0;
volatile int encoderRBCount = 0;
volatile int encoderRGCount = 0;
volatile boolean comparasonFlag = 1;

// DFA
byte stackPointer = 0;
byte stack[36][2] = {0};
boolean maze[8][8] = {0};
byte edgeMatrix[8][8][4] = {0};

byte x = 1;
byte y = 1;
byte orientation = 0; // 0 = north, 1 = east, 2 = south, 3 = west

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
        orientation = north;
        //turn(orientation);
        //forward(1);
      }
    else if(findX = 1)
      {
        orientation = south;
        //turn(orientation);
        //forward(1);
      }
    else if(findY = -1)
      {
        orientation = east;
        //turn(orientation);
        //forward(1);
      }
    else if(findY = 1)
      {
        orientation = west;
        //turn(orientation);
        //forward(1);
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