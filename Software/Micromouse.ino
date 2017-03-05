#define north 0
#define east  1
#define south 2
#define west  3

byte stackPointer = 0;
byte stack[36][2] = {0};
boolean maze[8][8] = {0};
byte edgeMatrix[8][8][4] = {0};

byte x = 1;
byte y = 1;
byte orientation = 0; // 0 = north, 1 = east, 2 = south, 3 = west

void setup()
{
  
}

void loop()
{
  
}

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
