// Pin Definitions
// Motors defined whilst looking from rear of robot.
#define mL 10 // Left motor drive
#define mR 11 // Right motor drive
#define enableML 9 // Left motor enable pin (High is on)
#define enableMR 12 // Right motor enable pin (High is on)

#define encoderLB 2 // Left motor encoder blue
#define encoderLG 6 // Left motor encoder green
#define encoderRB 3 // Right motor encoder blue
#define encoderRG 7 // Right motor encoder green

// Constants
#define squareWidth 500
#define degrees90C 202
#define degrees90A 188 


// Global variables
volatile int encoderLBCount = 0;
volatile int encoderLGCount = 0;
volatile int encoderRBCount = 0;
volatile int encoderRGCount = 0;

void setup()
{
Serial.begin(9600);
interrupts();
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

attachInterrupt(digitalPinToInterrupt(encoderLB), encoderLBCounter, RISING);
attachInterrupt(digitalPinToInterrupt(encoderLG), encoderLGCounter, RISING);
attachInterrupt(digitalPinToInterrupt(encoderRB), encoderRBCounter, RISING);
attachInterrupt(digitalPinToInterrupt(encoderRG), encoderRGCounter, RISING);

//Serial.println("Setup done");
delay(1000);
forward(2);
delay(250);
antiClockwise90(2);
delay(250);
forward(1);
delay(250);
antiClockwise90(1);
delay(250);
forward(1);
delay(250);
clockwise90(1);
delay(250);
forward(1);
delay(250);
clockwise90(1);
delay(250);
forward(1);
delay(250);
antiClockwise90(2);
delay(250);
forward(1);
delay(250);
clockwise90(1);
delay(250);
forward(1);
delay(250);
}

void loop() 
{


}

void forward( int squares)
{
  //Serial.println("Forward begin");
  // Travels forward x squares, x being int squares.
  volatile boolean flag = 0;
  encoderLBCount = 0;
  encoderLGCount = 0;
  encoderRBCount = 0;
  encoderRGCount = 0;

  int distance = squareWidth * squares;
  
  analogWrite(mL, 193);
  analogWrite(mR, 191);
  digitalWrite(enableML, HIGH);
  digitalWrite(enableMR, HIGH);
  //Serial.println(distance);

  while(!flag)
  {
    flag = (encoderLBCount > distance) || (encoderRBCount > distance);
    //Serial.println();
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
  //Serial.println("Braking end and exit forward");
  return;
}

void clockwise90( int turns)
{ 
  volatile boolean flag = 0;
  encoderLBCount = 0;
  encoderLGCount = 0;
  encoderRBCount = 0;
  encoderRGCount = 0;

  int degreestoturn = degrees90C * turns;
  //Serial.println("clockwise start");
  
  
  analogWrite(mL, 161);
  analogWrite(mR, 95);
  digitalWrite(enableML, HIGH);
  digitalWrite(enableMR, HIGH);
  //Serial.println(degreestoturn);
  
  while(!flag)
  {
    flag = (encoderLBCount > degreestoturn || encoderRBCount > degreestoturn);
    //Serial.println(encoderRBCount);
  }

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
  volatile boolean flag = 0;
  encoderLBCount = 0;
  encoderLGCount = 0;
  encoderRBCount = 0;
  encoderRGCount = 0;

  int degreestoturn = degrees90A * turns;
  //Serial.println("clockwise start");
  
  
  analogWrite(mL, 95);
  analogWrite(mR, 161);
  digitalWrite(enableML, HIGH);
  digitalWrite(enableMR, HIGH);
  //Serial.println(degreestoturn);
  
  while(!flag)
  {
    flag = (encoderLBCount > degreestoturn || encoderRBCount > degreestoturn);
    //Serial.println(encoderRBCount);
  }

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
