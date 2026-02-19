// Libraries + Config + Variables + Pins

#include <Stepper.h>
#include <SoftwareSerial.h> 

const int limitSwitchPin = 2;
buttonPin = 1;
const int DCIN1 = D2;
const int DCIN2 = D3;
SoftwareSerial Serial1(D10, D11);

const int stepsPerRevolution = 200;
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

myStepper.setSpeed(480);

pinMode(buttonPin, INPUT);
pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);

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

distCheck(dist, readiness);

startButton = digitalRead(buttonPin);

// collect sample

if (startButton == 1 && readiness == 1)
{
	// Time Calculation

  motorTime = getMotorTime(dist);

  // Drop Sampler

  runMotorDown(motorTime);
  delay(60000); // Hold for 60 seconds

  // Height Recheck

  readiness = 0;

  while (readiness == 0)
  {
  dist = measureHeight();
  distCheck(dist, readiness);
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
switch (height) 
  {
  case 8: return 20000;
  case 9: return 30000;
  case 10: return 40000;
  default: return 0;
 	}
}

void runMotorDown(int motorTime)
{
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
delay(motorTime);
digitalWrite(IN2, HIGH);
}

void runMotorUp(int motorTime)
{
digitalWrite(IN1, HIGH);
digitalWrite(IN2, HIGH);

while (digitalRead(limitSwitchPin) == HIGH)
{
	digitalWrite(IN2, LOW);
	}
digitalWrite(IN2, HIGH); 
}

void rotateStepper(float rot_degrees) 
{
 myStepper.step(stepsPerRevolution);
  Serial.println("Stepper rotated 45 degrees");
}

void distCheck(float dist, int readiness)
{
  if (dist < 8) 
  {
  Serial.println("Go Up"); 
  readiness = 0;
  } 
  else if (dist > 10) 
  {
  Serial.println("Go Down"); 
  readiness = 0;
  } 
  else 
  {
  Serial.println("Ready to Collect"); 
  readiness = 1;
  }
}
