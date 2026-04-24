#include <Stepper.h>

// Same stepper hardware setup as your original code
Stepper myStepper(200, A2, A3, A4, A5);

// ─── Tunable Test Values ─────────────────────────────────────
long TEST_STEPS = 2500;   // change this value to test different amounts
int STEPPER_SPEED_RPM = 300;

// ─── Helper ──────────────────────────────────────────────────
void stopStepper() {
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);
}

void runStepperTest() {
  Serial.print(F("Running stepper test with steps = "));
  Serial.println(TEST_STEPS);

  myStepper.step(TEST_STEPS);
  stopStepper();

  Serial.println(F("Stepper move complete."));
  Serial.println();
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  myStepper.setSpeed(STEPPER_SPEED_RPM);

  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);

  stopStepper();

  Serial.println(F("Stepper calibration test ready."));
  Serial.print(F("Current TEST_STEPS = "));
  Serial.println(TEST_STEPS);
  Serial.println(F("Send 'g' in Serial Monitor to run the test."));
  Serial.println(F("Send 'p' to print the current value."));
  Serial.println(F("Send '+' to add 50 steps."));
  Serial.println(F("Send '-' to subtract 50 steps."));
  Serial.println(F("Send 'r' to reverse the sign of TEST_STEPS."));
  Serial.println();
}

// ─── Loop ────────────────────────────────────────────────────
void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'g' || cmd == 'G') {
      runStepperTest();
    }
    else if (cmd == 'p' || cmd == 'P') {
      Serial.print(F("Current TEST_STEPS = "));
      Serial.println(TEST_STEPS);
    }
    else if (cmd == '+') {
      TEST_STEPS += 50;
      Serial.print(F("TEST_STEPS updated to "));
      Serial.println(TEST_STEPS);
    }
    else if (cmd == '-') {
      TEST_STEPS -= 50;
      Serial.print(F("TEST_STEPS updated to "));
      Serial.println(TEST_STEPS);
    }
    else if (cmd == 'r' || cmd == 'R') {
      TEST_STEPS = -TEST_STEPS;
      Serial.print(F("TEST_STEPS updated to "));
      Serial.println(TEST_STEPS);
    }
  }
}