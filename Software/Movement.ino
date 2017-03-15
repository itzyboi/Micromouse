// Pin Definitions
// Motors defined whilst looking from rear of robot.
#define mL 10 // Left motor drive
#define mR 11 // Right motor drive
#define enableML 9 // Left motor enable pin (High is on)
#define enableMR 12 // Right motor enable pin (High is on)

#define encoderLB 5 // Left motor encoder blue
#define encoderLG 6 // Left motor encoder green
#define encoderRB 7 // Right motor encoder blue
#define encoderRG 8 // Right motor encoder green

#define thresholdLB 41 // threshold values for software schmitt triggers
#define thresholdLG 30 // ADC's are 10 bit with 5V reference
#define thresholdRB 30
#define thresholdRG 41

// Constants
#define squareWidth 666
#define degrees90 127


// Global variables
int encoderLBCount = 0;
int encoderLGCount = 0;
int encoderRBCount = 0;
int encoderRGCount = 0;

void setup()
{
Serial.begin(9600);
Serial.println("Program initialised");

pinMode(encoderLB, INPUT);
pinMode(encoderLG, INPUT);
pinMode(encoderRB, INPUT);
pinMode(encoderRG, INPUT);

attachInterrupt(digitalPinToInterrupt(encoderLB), encoderLBCounter, HIGH);
attachInterrupt(digitalPinToInterrupt(encoderLG), encoderLGCounter, HIGH);
attachInterrupt(digitalPinToInterrupt(encoderRB), encoderRBCounter, HIGH);
attachInterrupt(digitalPinToInterrupt(encoderRG), encoderRGCounter, HIGH);

pinMode(mL, OUTPUT);
pinMode(mR, OUTPUT);
pinMode(enableMR, OUTPUT);
pinMode(enableML, OUTPUT);
digitalWrite(enableML, LOW);
digitalWrite(enableMR, LOW);
analogReference(DEFAULT);
Serial.println("Setup done");
}

void loop() 
{

delay(2000);
//forward(1);
//delay(2000);
clockwise90(1);
//delay(2000);
//antiClockwise90(1);

}

void forward( int squares)
{
  Serial.println("Forward begin");
  // Travels forward x squares, x being int squares.
  
  int distance = 0;
  boolean flag  = 0;
  boolean trigLB = 0;
  boolean trigLG = 0;
  boolean trigRB = 0;
  boolean trigRG = 0;
  int readingLB = 0;
  int readingLG = 0;
  int readingRB = 0;
  int readingRG = 0;
  
  encoderLBCount = 0;
  encoderLGCount = 0;
  encoderRBCount = 0;
  encoderRGCount = 0;

  distance = squareWidth * squares;
  
  analogWrite(mL, 191);
  analogWrite(mR, 191);
  digitalWrite(enableML, HIGH);
  digitalWrite(enableMR, HIGH);
  Serial.println("Forward set");
  while(!flag)
  { 
    readingLB = analogRead(encoderLB);
    readingLG = analogRead(encoderLG);
    readingRB = analogRead(encoderRB);
    readingRG = analogRead(encoderRG);
    if((readingLB > thresholdLB) && trigLB == 0)
    {
      encoderLBCount++;
      trigLB = 1;
    }
    else if(readingLB < thresholdLB)
    {
      trigLB = 0;
    }
    
    if((readingLG > thresholdLG) && trigLG == 0)
    {
      encoderLGCount++;
      trigLG = 1;
    }
    else if(readingLG < thresholdLG)
    {
      trigLG = 0;
    }
    
    if((readingRB > thresholdRB) && trigRB == 0)
    {
      encoderRBCount++;
      trigRG = 1;
    }
    else if(readingRB < thresholdRB)
    {
      trigRB = 0;
    }
    
    if((readingRG > thresholdRG) && trigRG == 0)
    {
      encoderRGCount++;
      trigRG = 1;
    }
    else if(readingRG < thresholdRG)
    {
      trigRG = 0;
    }
    
    if(encoderLBCount > distance ||  encoderLGCount > distance || encoderRBCount > distance || encoderRGCount > distance)
    {
      flag = 1;
      //Serial.println("Flag set");
    }
  }
  // Braking maneuver
  digitalWrite(enableML, LOW);
  digitalWrite(enableMR, LOW);
  //Serial.println("Motors off");
  //Serial.println("Braking start");
  //analogWrite(mL, 0);
  //analogWrite(mR, 0);
  //Serial.println("Motors on");
  //digitalWrite(enableML, HIGH);
  //digitalWrite(enableMR, HIGH);  
  //delay(1);
  //digitalWrite(enableML, LOW);
  //digitalWrite(enableMR, LOW);
  //Serial.println("Motors off");
  Serial.println("Braking end and exit forward");
  return;
}

void clockwise90( int turns)
{ 
  boolean flag = 0;
  boolean trigLB = 0;
  boolean trigLG = 0;
  boolean trigRB = 0;
  boolean trigRG = 0;
  int readingLB = 0;
  int readingLG = 0;
  int readingRB = 0;
  int readingRG = 0;
  
  encoderLBCount = 0;
  encoderLGCount = 0;
  encoderRBCount = 0;
  encoderRGCount = 0;

  int degreestoturn = degrees90 * turns;

  
  
  analogWrite(mL, 159);
  analogWrite(mR, 95);
  digitalWrite(enableML, HIGH);
  digitalWrite(enableMR, HIGH);

  
  while(encoderLBCount < degreestoturn || encoderLGCount < degreestoturn || encoderRBCount < degreestoturn || encoderRGCount < degreestoturn)
  {
    //average = (encoderL1Count + encoderL2Count + encoderR1Count + encoderR2Count) / 4;
  }
  
  /*while(!flag)
  {  
    readingLB = analogRead(encoderLB);
    if((readingLB > thresholdLB) && trigLB == 0)
    {
      encoderLBCount++;
      trigLB = 1;
    }
    else if(readingLB < (thresholdLB))
    {
      trigLB = 0;
    }

    readingLG = analogRead(encoderLG);
    if((readingLG > thresholdLG) && trigLG == 0)
    {
      encoderLGCount++;
      trigLG = 1;
    }
    else if(readingLG < (thresholdLG))
    {
      trigLG = 0;
    }
    
    readingRB = analogRead(encoderRB);
    if((readingRB > thresholdRB) && trigRB == 0)
    {
      encoderRBCount++;
      trigRG = 1;
    }
    else if(readingRB < (thresholdRB))
    {
      trigRB = 0;
    }

    readingRG = analogRead(encoderRG);
    if((readingRG > thresholdRG) && trigRG == 0)
    {
      encoderRGCount++;
      trigRG = 1;
    }
    else if(readingRG < (thresholdRG))
    {
      trigRG = 0;
    }
    
    if(encoderLBCount > degreestoturn || encoderLGCount > degreestoturn || encoderRBCount > degreestoturn || encoderRGCount > degreestoturn)
    {
      flag = 1;
    }
  }
  */
  // Braking maneuver
  digitalWrite(enableML, LOW);
  digitalWrite(enableMR, LOW);
  //analogWrite(mL, 64);
  //analogWrite(mR, 191);
  //digitalWrite(enableML, HIGH);
  //digitalWrite(enableMR, HIGH);  
  //delay(1);
  //digitalWrite(enableML, LOW);
  //digitalWrite(enableMR, LOW);
  return;
}

void antiClockwise90( int turns)
{ 
  boolean flag = 0;
  boolean trigLB = 0;
  boolean trigLG = 0;
  boolean trigRB = 0;
  boolean trigRG = 0;
  int readingLB = 0;
  int readingLG = 0;
  int readingRB = 0;
  int readingRG = 0;
  
  encoderLBCount = 0;
  encoderLGCount = 0;
  encoderRBCount = 0;
  encoderRGCount = 0;

  int degreestoturn = degrees90 * turns;

  Serial.println("Anti-clockwise90 start");

  analogWrite(mL, 64);
  analogWrite(mR, 191);
  digitalWrite(enableML, HIGH);
  digitalWrite(enableMR, HIGH);
  
  while(!flag)
  {
    readingLB = analogRead(encoderLB);
    readingLG = analogRead(encoderLG);
    readingRB = analogRead(encoderRB);
    readingRG = analogRead(encoderRG);
    if((readingLB > thresholdLB) && trigLB == 0)
    {
      encoderLBCount++;
      trigLB = 1;
    }
    else if(readingLB < thresholdLB)
    {
      trigLB = 0;
    }
    
    if((readingLG > thresholdLG) && trigLG == 0)
    {
      encoderLGCount++;
      trigLG = 1;
    }
    else if(readingLG < thresholdLG)
    {
      trigLG = 0;
    }
    
    if((readingRB > thresholdRB) && trigRB == 0)
    {
      encoderRBCount++;
      trigRG = 1;
    }
    else if(readingRB < thresholdRB)
    {
      trigRB = 0;
    }
    
    if((readingRG > thresholdRG) && trigRG == 0)
    {
      encoderRGCount++;
      trigRG = 1;
    }
    else if(readingRG < thresholdRG)
    {
      trigRG = 0;
    }
    
    if(encoderLBCount > degreestoturn || encoderLGCount > degreestoturn || encoderRBCount > degreestoturn || encoderRGCount > degreestoturn)
    {
      flag = 1;
    }
  }
  // Braking maneuver
  digitalWrite(enableML, LOW);
  digitalWrite(enableMR, LOW);
  //analogWrite(mL, 255);
  //analogWrite(mR, 0);
  //digitalWrite(enableML, HIGH);
  //digitalWrite(enableMR, HIGH);  
  //delay(1);
  //digitalWrite(enableML, LOW);
  //digitalWrite(enableMR, LOW);
  Serial.println("anticlockwise end");
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
