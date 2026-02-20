// Libraries + Config + Variables + Pins
#include <Stepper.h>
#include <SoftwareSerial.h>


const int limitSwitchPin = 4;
const int buttonPin = 9;
const int DCIN1 = A0;
const int DCIN2 = A1;
SoftwareSerial Serial1(10, 11);


const int stepsPerRevolution = 200;
Stepper myStepper(stepsPerRevolution, A2, A3, A4, A5);

int samplesCollected = 0;
int heightVal = 0;
int motorTime = 0;
int startButton = 0;
int readiness = 0;
int rot_degrees;
float dist;                
int strength;          
int check;            
int i;
int uart[9];                
const int HEADER = 0x59;      


// Setup
void setup()
{
Serial.begin(9600);  
Serial1.begin(115200);
myStepper.setSpeed(300);
pinMode(limitSwitchPin, INPUT);
pinMode(buttonPin, INPUT);
pinMode(DCIN1, OUTPUT);
pinMode(DCIN2, OUTPUT);
}


// Main Loop
void loop()
{
// check if full
if (samplesCollected >= 8)
  {
  Serial.println("Capacity full");
  return 0;
  }


// measure height
dist = measureHeight();
readiness = distCheck(dist, readiness);
startButton = digitalRead(buttonPin);

// collect sample
if (startButton == 1 && readiness == 1)
{
  // Time Calculation
  motorTime = getMotorTime(dist);


  // Drop Sampler
  runMotorDown(motorTime);
  delay(motorTime); // Hold for 60 seconds


  // Height Recheck
  readiness = 0;
  while (readiness == 0)
  {
    dist = measureHeight();
    readiness = distCheck(dist, readiness);
    startButton = digitalRead(buttonPin);
  }

  motorTime = getMotorTime(dist);


  // Retrieve Sampler
  runMotorUp(motorTime);


  // Sample Logging
  samplesCollected++;
  Serial.print("Samples Collected: ");
  Serial.println(samplesCollected);


  // Sample check and instructions
  if (samplesCollected < 8)
    {
      rot_degrees = (100*stepsPerRevolution)/8;
      }
  else
    {
      rot_degrees = (100*stepsPerRevolution)/16 ;
      }
 
  rotateStepper(rot_degrees);
   
  if (samplesCollected == 8)
    {
    Serial.println("Capacity full");
    return 0;
    }
  else
    {
    Serial.println("Fly to next location");
    }
  }
}


// Functions
float measureHeight()
{
  if (Serial1.available())
  {
    if (Serial1.read() == HEADER)
    {
      uart[0] = HEADER;
      if (Serial1.read() == HEADER)
      {
        uart[1] = HEADER;
        for (i = 2; i < 9; i++)
        {
          uart[i] = Serial1.read();
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        if (uart[8] == (check & 0xff))
        {
          dist = uart[2] + uart[3] * 256;
          Serial.print("dist = ");
          Serial.print(dist);
          Serial.print('\n');
        }
      }
    }
  }
  return dist;
}


int getMotorTime(float dist)
{
return 5000;
}


void runMotorDown(int motorTime)
{
digitalWrite(DCIN1, HIGH);
digitalWrite(DCIN2, LOW);
delay(motorTime);
digitalWrite(DCIN2, HIGH);
}


void runMotorUp(int motorTime)
{
digitalWrite(DCIN1, HIGH);
digitalWrite(DCIN2, HIGH);
 
while (digitalRead(limitSwitchPin) == 0)
{
  digitalWrite(DCIN1, LOW);
  }
digitalWrite(DCIN1, HIGH);
}


void rotateStepper(float rot_degrees)
{
 myStepper.step(rot_degrees);
}


int distCheck(float dist, int readiness)
{
  if (dist < 50)
  {
  Serial.println("Go Up");
  readiness = 0;
  }
  else if (dist > 150)
  {
  Serial.println("Go Down");
  readiness = 0;
  }
  else
  {
  Serial.println("Ready to Collect");
  readiness = 1;
  }
  return readiness;
}
