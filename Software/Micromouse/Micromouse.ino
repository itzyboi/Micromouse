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
#define degrees90 178

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

// A*
byte shortestPath[36][2] = {0}; // Shortest path array, comtains X and Y coords of the path

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
}

void loop()
{
  switch (state)
  {
    case 0: // Do nothing state, stops it accidentaly beginning a search
      {
        digitalWrite(red, HIGH);
        if (leftSensor() > 130) // Waits for you to brush an object infront of a sensor
        {
          state = 1; // Set state to intermediate state
        }
        break;
      }
    case 1: // Intermediate state, to stop the modes cycling at clock speed
      {
        if (leftSensor() < 100) // Waits for object to be moved away
        {
          digitalWrite(red, LOW);
          state = 2; // Enter search mode
        }
        break;
      }
    case 2: // Search mode state
      {
        digitalWrite(yellow, HIGH);
        if (rightSensor() > 130) // If object is put infront of right sensor, preform a search
        {
          delay(1000); // Wait a second for object to be moved away
          DFA(); // Search
          turn(north); // After search has been preformed, turn so robot is facing square exit
          digitalWrite(yellow, LOW);
          state = 4; // Enter A* state
        }
        else if (leftSensor() > 130) // Else wait for an object to be placed infront of the left sensor to change mode
        {
          state = 3;
        }
        break;
      }
    case 3: // Intermediate mode to stop mode cycling at clock speed
      {
        if (leftSensor() < 100)
        {
          digitalWrite(yellow, LOW);
          state = 4;
        }
        break;
      }
    case 4: // A* (PAthfinding state) state
      {
        digitalWrite(green, HIGH);
        if (rightSensor() > 130) //Perform A* if object placed infront of right sensor
        {
          delay(1000);
          AStar();
        }
        else if (leftSensor() > 130) // Go back to do nothing state if object placed infront of left sensors
        {
          state = 5;
        }
        break;
      }
    case 5: // Intermediate state to stop state cycling at clock speed
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
    /*
    If the square has never been visited before the sensors must be read to check for walls.
    This only needs to happen the first time as readings are assumed correct
    */
    if (maze[x][y] == 0) // Check to see if node is unvisited
    {
      sensorRead(); // Sense walls
      maze[x][y] = 1; // set node to visited
    }
    Serial.println(); // Print over serial for debugging
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
    /*
    Once maze data has been collected for the current square, the robot must decide what square to move to next.
    It will always try to go north first, if it is unable to (Either there is a wall in the way or the node has already been visited)
    It will then try to go east, then south then west.
    As it travels it saves the squares it has been to in a stack, so if it hits a dead end it can back track it's steps 
    until it can travel into a new square.IT repeats this process untill it returns to the start square (0,0)
    */
    if ((edgeMatrix[x][y][north] == 1) && (maze[x + 1][y] == 0)) // IF there is a path north and the north square is unvisited
    {
      turn(north); // move there
      orientation = north;
      forward(1);
      x = x + 1; // update coordinates
      stackPointer++; // Save the coordinates to the stack
      stack[stackPointer][0] = x;
      stack[stackPointer][1] = y;
    }
    else if ((edgeMatrix[x][y][east] == 1) && (maze[x][y + 1] == 0)) // Else try to move east
    {
      turn(east);
      orientation = east;
      forward(1);
      y = y + 1;
      stackPointer++;
      stack[stackPointer][0] = x;
      stack[stackPointer][1] = y;
    }
    else if ((edgeMatrix[x][y][south] == 1) && (maze[x - 1][y] == 0)) // Else try to move south
    {
      turn(south);
      orientation = south;
      forward(1);
      x = x - 1;
      stackPointer++;
      stack[stackPointer][0] = x;
      stack[stackPointer][1] = y;
    }
    else if ((edgeMatrix[x][y][west] == 1) && (maze[x][y - 1] == 0)) // Else try to move west
    {
      turn(west);
      orientation = west;
      forward(1);
      y = y - 1;
      stackPointer++;
      stack[stackPointer][0] = x;
      stack[stackPointer][1] = y;
    }
    else // Other wise pop the stack and travel back a square
    {
      stackPointer--;
      // Only one coordinate can change every square moved, therefore to find the direction
      // to travel the coordinated to move to are subtracted from the current coordinates.
      // This then gives the direction to travel as follows.
      // X1 - X2 = -1 --> move north
      // x1 - x2 = 1  --> move south
      // y1 - y2 = 1  --> move west
      // y1 - y2 = -1 --> move east
      findX = x - stack[stackPointer][0]; // calculate X change
      findY = y - stack[stackPointer][1]; // Calculate Y change
      
      Serial.print("x(-1): "); // Print debugging infomation
      Serial.print(stack[stackPointer][0]);
      Serial.print(" y(-1): ");
      Serial.println(stack[stackPointer][1]);

      if (findX == -1) // move north
      {
        turn(north);
        orientation = north;
        forward(1);
        x = stack[stackPointer][0];
        y = stack[stackPointer][1];
      }
      else if (findX == 1) // move south
      {
        turn(south);
        orientation = south;
        forward(1);
        x = stack[stackPointer][0];
        y = stack[stackPointer][1];
      }
      else if (findY == -1) // move east
      {
        turn(east);
        orientation = east;
        forward(1);
        x = stack[stackPointer][0];
        y = stack[stackPointer][1];
      }
      else if (findY == 1) // move west
      {
        turn(west);
        orientation = west;
        forward(1);
        x = stack[stackPointer][0];
        y = stack[stackPointer][1];
      }
    }
    if((x == 0) && (y == 0)) // Check to see if the robot has returned to the beginning. If it has exit search
    {
      flag = 1;
    }
  }
}

void addEdge(int bearingToChange) 
{
  /*
  Adds a path between two squares. Takes an orientation as input and removes the wall in that direction
  */
  Serial.print("bearingToChange: "); // print debugging info
  Serial.println(bearingToChange);
  
  edgeMatrix[x][y][bearingToChange] = 1; // Removes the wall in the orientation given for the current square
  // As each square has a bit for each wall, the wall must also be removed in the adjacent square.
  if (bearingToChange == north) // If removing north wall, remove southern wall in the northern square
  {
    edgeMatrix[x + 1][y][south] = 1;
  }
  else if (bearingToChange == east) // If removing east wall, remove west wall in the eastern square
  {
    edgeMatrix[x][y + 1][west] = 1;
  }
  else if (bearingToChange == south) // If removing south wall, remove north wall in southern square
  {
    edgeMatrix[x - 1][y][north] = 1;
  }
  else if (bearingToChange == west) // If removing west wall, remove east wall in western square
  {
    edgeMatrix[x][y - 1][east] = 1;
  }
}

void sensorRead()
{
  /*
  Tells sensors to take a reading, it then checks to see fi the reading is below the threshold ADC value found in testing.
  The threshold value is the value a sensor reads if a wall is 6cm away (The maximum distance the robot can be from a wall.
  If the reading is less than the threshold it removes the wall by calling addEdge() and gives it the direction to change.
  */
  int bearingToChange;

  if (frontSensor() < 16)
  {
    Serial.print("adding front edge: ");
    Serial.println(orientation);
    addEdge(orientation); // The front sensor is always facing the direction the robot is facing
  }
  /*
  To calculate the direction the left and right sensors are facing 1 is added or subtracted from the forward direction. 
  If this is outside of the range of the defined directions at the top it wraps the value round before calling addEdge()
  */
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
  /*
  Takes a raw reading of the reflected light.
  It does this by turning the IR LED off and taking a background reading, It then turns the LED back on and takes 
  another reading.
  It then returns the background reading subtracted from the reflected reading.
  */
  digitalWrite(IROn, LOW); // Turn LED off (it should already be off) 
  delay(50);
  int x = analogRead(FIR); // Take a background reading
  delay(50);
  digitalWrite(IROn, HIGH); // Turn LED back on
  delay(50);
  int y = analogRead(FIR); // Take reflected light reading
  digitalWrite(IROn, LOW); // Turn LED back off
  Serial.print("FIR");
  Serial.println(y - x); 
  return (y - x); // Return the difference
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
  int difference = 0; // difference between left and right motor counts
  byte speedL = 195; // Base speed of left motor
  byte speedR = 191; // Base speed of right motor
  boolean flag1 = 1; // flags used to tell mototrs to stop.
  boolean flag2 = 1;

  for (i = 0; i < squares; i++)
  {
    analogWrite(mL, speedL);
    analogWrite(mR, speedR);
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
      speedL = 195;
      speedR = 191;
      // Stop condidtion
      comparisonFlag = 1;
      if ((encoderLBCount >= (squareWidth + 4)) && (flag1))
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableML, LOW);
          flag1 = 0;
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= (squareWidth)) && (flag2))
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableMR, LOW);
          flag2 = 0;
        }
      }
      // Slow down when coming to stop condition as to not overshoot
      comparisonFlag = 1;
      if ((encoderLBCount >= (squareWidth - 25)) && (flag1))
      {
        if (comparisonFlag == 1)
        {
          speedL = speedL - 30;
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= (squareWidth - 25)) && (flag2))
      {
        if (comparisonFlag == 1)
        {
          speedR = speedR - 30;
        }
      }
      // Motor control: keep difference between encoder counts to a minimum.
      difference = encoderRBCount - encoderLBCount;
      comparisonFlag = 1;
      if((difference > 0) &&(flag2))
      {
        if(comparisonFlag)
        {
          speedR = speedR - (5 * difference);
          if(speedR < 0)
          {
            speedR = 0;
          }
        }
      }
      comparisonFlag = 1;
      if((difference < 0) && (flag2))
      {
        if(comparisonFlag)
        {
          speedR = speedR - (5 * difference);
          if(speedR > 255)
          {
            speedR = 255;
          }
        }
      }
      analogWrite(mL, speedL);
      analogWrite(mR, speedR);
    }
  }
  return;
}

void clockwise90( int turns)
{
  byte speedL = 169;
  byte speedR = 95;
  boolean flag1 = 1;
  boolean flag2 = 1;
  byte i = 0;

  for (i = 0; i < turns; i++)
  {
    analogWrite(mL, 165);
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
      speedL = 165;
      speedR = 95;
      comparisonFlag = 1;

      // Stop if goal met.
      if ((encoderLBCount >= (degrees90)) && (flag1))
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableML, LOW);
          flag1 = 0;
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= (degrees90)) && ( flag2))
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableMR, LOW);
          flag2 = 0;
        }
      }
      // If coming close to goal, slow down to not overshoot
      comparisonFlag = 1;
      if ((encoderLBCount >= (degrees90 - 25)) && (flag1))
      {
        if (comparisonFlag)
        {
          speedL = speedL - 5;
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= (degrees90 - 25)) && (flag2))
      {
        if (comparisonFlag)
        {
          speedR = speedR + 5;
        }
      }
      // Keep count difference to a minimum.
      analogWrite(mL, speedL);
      analogWrite(mR, speedR);
    }
  }
  return;
}

void antiClockwise90( int turns)
{
  byte speedL = 95;
  byte speedR = 163;
  boolean flag1 = 1;
  boolean flag2 = 1;
  byte i = 0;

  for (i = 0; i < turns; i++)
  {
    analogWrite(mL, speedL);
    analogWrite(mR, speedR);
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
      speedL = 95;
      speedR = 163;
      comparisonFlag = 1;

      // Stop if goal met.
      if ((encoderLBCount >= (degrees90)) && (flag1))
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableML, LOW);
          flag1 = 0;
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= (degrees90)) && ( flag2))
      {
        if (comparisonFlag == 1)
        {
          digitalWrite(enableMR, LOW);
          flag2 = 0;
        }
      }
      // If coming close to goal, slow down to not overshoot
      comparisonFlag = 1;
      if ((encoderLBCount >= (degrees90 - 25)) && (flag1))
      {
        if (comparisonFlag)
        {
          speedL = speedL + 5;
        }
      }
      comparisonFlag = 1;
      if ((encoderRBCount >= (degrees90 - 25)) && (flag2))
      {
        if (comparisonFlag)
        {
          speedR = speedR - 5;
        }
      }
      // Keep count difference to a minimum.
      analogWrite(mL, speedL);
      analogWrite(mR, speedR);
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

//A*
void AStar( void)
{
  byte i = 0;
  byte j = 0;

  boolean found = 0;
  boolean openList[6][6] = {0};
  boolean closedList[6][6] = {0};
  byte costs[6][6][6] = {0}; // 0 = F cost, 1 = G cost, 2 = H cost, 3 = Parent X coord, 4 = Parent Y coord, 5 = # of parents

  byte bestX = 255;
  byte bestY = 255;
  byte tempX = 255;
  byte tempY = 255;

  for (i = 0; i < 6; i++)
  {
    for (j = 0; j < 6; j++)
    {
      costs[i][j][0] = 255;
    }
  }
  Serial.println("Starting A*");
  //Set initial condidtions - the search ends when the goal square enters the openList so it's initialised to a high F cost to be used in the first interation of the loop
  openList[0][0] = 1;
  costs[0][0][0] = 0;
  costs[5][5][0] = 255;
  Serial.println("Initial conditions done");
  while (!found)
  {
    bestX = 5;
    bestY = 5;
    // Search open list for lowest F cost, then select it and put it in closed list
    for (i = 0; i < 6; i++)
    {
      for (j = 0; j < 6; j++)
      {
        if (openList[i][j] == 1)
        {
          if (costs[i][j][0] < costs[bestX][bestY][0])
          {
            bestX = i;
            bestY = j;
          }
        }
      }
    }
    Serial.print("Found lowest F cost: ");
    Serial.println(costs[bestX][bestY][0]);
    Serial.print("Square: x: ");
    Serial.print(bestX);
    Serial.print(" y: ");
    Serial.println(bestY);
    closedList[bestX][bestY] = 1; 
    openList[bestX][bestY] = 0;
    // Calculate adjacent squares cost (if it is a valid move or not already closed), if cost is lower than it's current cost update cost and make selected square it's parent
    if ((edgeMatrix[bestX][bestY][north] == 1) && ( closedList[bestX + 1][bestY] == 0))
    {
      costs[bestX + 1][bestY][1] = costs[bestX][bestY][1] + 1; // Calculate G cost
      costs[bestX + 1][bestY][2] = (5 - (bestX + 1)) + (5 - bestY); // Estimate H cost
      costs[bestX + 1][bestY][0] = costs[bestX + 1][bestY][1] + costs[bestX + 1][bestY][2]; // Calculate F cost
      costs[bestX + 1][bestY][3] = bestX; // Save parent x-coord
      costs[bestX + 1][bestY][4] = bestY; // Save parent y-coord
      costs[bestX + 1][bestY][5] = costs[bestX][bestY][5] + 1; // Update # of parent squares
      openList[bestX + 1][bestY] = 1; // Add square to open list
    }

    if ((edgeMatrix[bestX][bestY][south] == 1) && ( closedList[bestX - 1][bestY] == 0))
    {
      costs[bestX - 1][bestY][1] = costs[bestX][bestY][1] + 1; // Calculate G cost
      costs[bestX - 1][bestY][2] = (5 - (bestX - 1)) + (5 - bestY); // Estimate H cost
      costs[bestX - 1][bestY][0] = costs[bestX - 1][bestY][1] + costs[bestX - 1][bestY][2]; // Calculate F cost
      costs[bestX - 1][bestY][3] = bestX; // Save parent x-coord
      costs[bestX - 1][bestY][4] = bestY; // Save parent y-coord
      costs[bestX - 1][bestY][5] = costs[bestX][bestY][5] + 1; // Update # of parent squares
      openList[bestX - 1][bestY] = 1; // Add square to open list
    }

    if ((edgeMatrix[bestX][bestY][east] == 1) && ( closedList[bestX][bestY + 1] == 0))
    {
      costs[bestX][bestY + 1][1] = costs[bestX][bestY][1] + 1; // Calculate G cost
      costs[bestX][bestY + 1][2] = (5 - bestX) + (5 - (bestY + 1)); // Estimate H cost
      costs[bestX][bestY + 1][0] = costs[bestX][bestY + 1][1] + costs[bestX][bestY][2]; // Calculate F cost
      costs[bestX][bestY + 1][3] = bestX; // Save parent x-coord
      costs[bestX][bestY + 1][4] = bestY; // Save parent y-coord
      costs[bestX][bestY + 1][5] = costs[bestX][bestY][5] + 1; // Update # of parent squares
      openList[bestX][bestY + 1] = 1; // Add square to open list
    }

    if ((edgeMatrix[bestX][bestY][west] == 1) && ( closedList[bestX][bestY - 1] == 0))
    {
      costs[bestX][bestY - 1][1] = costs[bestX][bestY][1] + 1; // Calculate G cost
      costs[bestX][bestY - 1][2] = (5 - bestX) + (5 - (bestY - 1)); // Estimate H cost
      costs[bestX][bestY - 1][0] = costs[bestX][bestY - 1][1] + costs[bestX][bestY][2]; // Calculate F cost
      costs[bestX][bestY - 1][3] = bestX; // Save parent x-coord
      costs[bestX][bestY - 1][4] = bestY; // Save parent y-coord
      costs[bestX][bestY - 1][5] = costs[bestX][bestY][5] + 1; // Update # of parent squares
      openList[bestX][bestY - 1] = 1; // Add square to open list
    }
    Serial.println("Scored adjacents");
    // Check if goal is in open list and set loop condition false, goal has been found
    if (openList[5][5] == 1)
    {
      found = 1;
    }
  }
  Serial.println("Calculated Costs, finding fastest route");
  // Calculate shortest path by following the parent squares backwards from goal
  bestX = 5;
  bestY = 5;
  Serial.print("# of parents goal has: ");
  Serial.println(costs[0][0][5]);
  for (i = costs[5][5][5]; i > 0; i--)
  {
    shortestPath[i][0] = bestX;
    shortestPath[i][1] = bestY;
    tempX = costs[bestX][bestY][3];
    tempY = costs[bestX][bestY][4];
    bestX = tempX;
    bestY = tempY;
  }
  Serial.println("Found fastest route, printing...");
  for(i = 0; i < 36; i++)
  {
    Serial.print("X: ");
    Serial.print(shortestPath[i][0]);
    Serial.print(" Y: ");
    Serial.println(shortestPath[i][1]);
  }

  // Drive to goal
  int findX = 0;
  int findY = 0;
  x = 0;
  y = 0;
  i = 0;
  orientation = north;

  while (1)
  {
    if(x == 5)
    {
      if(y == 5)
      {
        break;
      }
    }
    
    findX = x - shortestPath[i][0];
    findY = y - shortestPath[i][1];

    Serial.print("findX: ");
    Serial.println(findX);
    Serial.print("findY: ");
    Serial.println(findY);
    
    if (findX == -1)
    {
      turn(north);
      orientation = north;
      forward(1);
      x = shortestPath[i][0];
      y = shortestPath[i][1];
    }
    else if (findX == 1)
    {
      turn(south);
      orientation = south;
      forward(1);
      x = shortestPath[i][0];
      y = shortestPath[i][1];
    }
    else if (findY == -1)
    {
      turn(east);
      orientation = east;
      forward(1);
      x = shortestPath[i][0];
      y = shortestPath[i][1];
    }
    else if (findY == 1)
    {
      turn(west);
      orientation = west;
      forward(1);
      x = shortestPath[i][0];
      y = shortestPath[i][1];
    }
    else;
    
    i++;
  }

}

