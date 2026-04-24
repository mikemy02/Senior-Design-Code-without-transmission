#include <Stepper.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

// ─── Tunable Parameters ────────────────────────────────────────────────────
const int TIME_SPENT_BOTTOM = 20;   // seconds idle at bottom after lowering

// These are now only safety timeouts.
// Ramp behavior is based on Hall counts, not seconds.
const unsigned long LOWER_TIME_MS = 20000;
const unsigned long RAISE_TIME_MS = 20000;

const int MOTOR_MIN_SPEED = 100;           // PWM 0–255: starting/ending speed
const int MOTOR_MAX_SPEED_DOWN = 190;     // lowering max PWM
const int MOTOR_MAX_SPEED_UP   = 210;     // raising max PWM

const float RAMP_FRACTION = 0.25;         // 25% ramp up, 50% full, 25% ramp down

// Lowering parameters in cm
const float SPOOL_CIRCUMFERENCE_CM = 5.65;
const float EXTRA_LOWER_OFFSET_CM  = 0;
const int   HALL_COUNTS_PER_REV    = 1;

// Stepper/carousel values
const int STANDARD_SLOT_STEPS = 2500;       // 45 degree turn
const int FINAL_HALF_SLOT_STEPS = 1250;     // 22.5 degree turn on 8th cycle

// Limit switch polarity
// Current Nathan/Sunday behavior assumes:
// LOW  = not home yet
// HIGH = home reached
const int LIMIT_NOT_PRESSED = LOW;
const int LIMIT_PRESSED     = HIGH;

// Motor direction setting
// Set this to true based on your request:
// lowering uses the inverse PWM value because it moves opposite direction.
const bool LOWERING_USES_INVERSE_PWM = false;

// ─── Pins ──────────────────────────────────────────────────────────────────
const int limitSwitchPin = 8;
const int DCIN1 = 5;
const int DCIN2 = 6;
const int hallPin = 7;

SoftwareSerial Serial1(9, 10); // Lidar
Stepper myStepper(200, A2, A3, A4, A5);

#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#define MAIN_ADDR 1
#define SECONDARY_ADDR 2

// ─── Lidar Variables ───────────────────────────────────────────────────────
float dist = 0;
int strength = 0;
int check = 0;
int i = 0;
int uart[9];
const int HEADER = 0x59;

// ─── LoRa Objects ──────────────────────────────────────────────────────────
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, MAIN_ADDR);

// ─── States ────────────────────────────────────────────────────────────────
enum State {
  IDLE,
  LOWERING,
  WAIT_AT_BOTTOM,
  CHECK_DEPTH,
  RAISING,
  ROTATING,
  FULL
};

State currentState = IDLE;

// ─── Timers ────────────────────────────────────────────────────────────────
unsigned long stateTimer = 0;
unsigned long lastTelemetry = 0;
unsigned long bottomTimer = 0;

// ─── Sampling / Action Tracking ────────────────────────────────────────────
int samples = 0;
uint32_t actionCounter = 0;
uint8_t controllerAddr = 0;

// ─── Hall Sensor Counting ──────────────────────────────────────────────────
unsigned long magnetCount = 0;
int lastHallState = HIGH;

float startDepthCm = 0.0;
float targetLowerDistanceCm = 0.0;
unsigned long targetLowerCounts = 0;
unsigned long targetRaiseCounts = 0;

// ─── Speed Profile Based on Motor Rotations / Hall Counts ──────────────────
int getMotorSpeedFromCounts(unsigned long currentCount,
                            unsigned long targetCount,
                            int maxSpeed) {
  if (targetCount == 0) targetCount = 1;

  float progress = (float)currentCount / (float)targetCount;
  progress = constrain(progress, 0.0f, 1.0f);

  float speedNorm;

  if (progress < RAMP_FRACTION) {
    speedNorm = progress / RAMP_FRACTION;                       // ramp up
  } else if (progress < (1.0f - RAMP_FRACTION)) {
    speedNorm = 1.0f;                                           // full speed
  } else {
    speedNorm = (1.0f - progress) / RAMP_FRACTION;              // ramp down
  }

  int pwm = (int)(MOTOR_MIN_SPEED + speedNorm * (maxSpeed - MOTOR_MIN_SPEED));
  return constrain(pwm, 0, 255);
}

// ─── Motor Helpers ─────────────────────────────────────────────────────────
void motorStop() {
  analogWrite(DCIN1, 0);
  analogWrite(DCIN2, 0);
}

// Lowering direction.
// Uses inverse PWM on one side because lowering is the opposite direction.
void motorLower(int pwmSpeed) {
  pwmSpeed = constrain(pwmSpeed, 0, 255);

  if (LOWERING_USES_INVERSE_PWM) {
    analogWrite(DCIN1, 255 - pwmSpeed);
    analogWrite(DCIN2, 255);
  } else {
    analogWrite(DCIN1, pwmSpeed);
    analogWrite(DCIN2, 0);
  }
}

// Raising direction.
// Opposite of lowering.
void motorRaise(int pwmSpeed) {
  pwmSpeed = constrain(pwmSpeed, 0, 255);

  if (LOWERING_USES_INVERSE_PWM) {
    analogWrite(DCIN1, pwmSpeed);
    analogWrite(DCIN2, 0);
  } else {
    analogWrite(DCIN1, 255 - pwmSpeed);
    analogWrite(DCIN2, 255);
  }
}

// ─── Hall Counter Helper ───────────────────────────────────────────────────
void updateMagnetCount(bool enableCounting) {
  int currentHallState = digitalRead(hallPin);

  if (!enableCounting) {
    lastHallState = currentHallState;
    return;
  }

  if (lastHallState == HIGH && currentHallState == LOW) {
    magnetCount++;
    Serial.print(F("MagnetCount: "));
    Serial.println(magnetCount);
  }

  lastHallState = currentHallState;
}

// ─── Lowering Count Calculation ────────────────────────────────────────────
unsigned long computeTargetLowerCounts(float measuredDepthCm) {
  targetLowerDistanceCm = measuredDepthCm + EXTRA_LOWER_OFFSET_CM;

  float revsNeeded = targetLowerDistanceCm / SPOOL_CIRCUMFERENCE_CM;
  float countsNeeded = revsNeeded * HALL_COUNTS_PER_REV;

  unsigned long result = (unsigned long)(countsNeeded);

  // Round up so we do not under-lower
  if ((float)result < countsNeeded) {
    result++;
  }

  if (result == 0) result = 1;

  Serial.print(F("Lower count calc | depth: "));
  Serial.print(measuredDepthCm);
  Serial.print(F(" cm | lower distance: "));
  Serial.print(targetLowerDistanceCm);
  Serial.print(F(" cm | revs: "));
  Serial.print(revsNeeded, 2);
  Serial.print(F(" | counts: "));
  Serial.println(result);

  return result;
}

// helper print function to stop flooding serial monitor with excessive prints
void printMotorStatus(long currentCount, long targetCount, int pwm, bool homeReached, bool raising) {
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 200) {
    if (raising){
      Serial.print(F("RAISING | count: "));
    } else {
      Serial.print(F("LOWERING | count: "));
    }
    Serial.print(currentCount);
    Serial.print(F(" / "));
    Serial.print(targetCount);
    Serial.print(F(" | pwm: "));
    Serial.print(pwm);
    Serial.print(F(" | limit: "));
    Serial.println(homeReached ? F("PRESSED") : F("NOT PRESSED"));
    lastPrint = millis();
  }
}

// ─── Setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);

  myStepper.setSpeed(300);

  pinMode(limitSwitchPin, INPUT);
  pinMode(DCIN1, OUTPUT);
  pinMode(DCIN2, OUTPUT);

  pinMode(hallPin, INPUT_PULLUP);
  lastHallState = digitalRead(hallPin);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println(F("Initializing LoRa..."));

  if (!manager.init()) {
    Serial.println(F("LoRa manager init failed"));
    while (1);
  }

  rf95.setFrequency(915.0);
  rf95.setTxPower(23, false);

  motorStop();

  Serial.println(F("System Ready"));
  Serial.print(F("Initial hall state: "));
  Serial.println(lastHallState);
  Serial.print(F("Initial limit switch: "));
  Serial.println(digitalRead(limitSwitchPin));
}

// ─── Main Loop ─────────────────────────────────────────────────────────────
void loop() {
  dist = measureHeight();

  if (millis() - lastTelemetry > 500) {
    sendDepth(dist);
    lastTelemetry = millis();
  }

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;

  if (manager.recvfromAckTimeout(buf, &len, 10, &from)) {
    Serial.print(F("LoRa RX | from: "));
    Serial.print(from);
    Serial.print(F(" | len: "));
    Serial.println(len);

    if (len >= 1 && buf[0] == 'T' && currentState == IDLE) {
      Serial.println(F("Trigger received while IDLE"));
      Serial.print(F("Current depth: "));
      Serial.println(dist);

      if (dist >= 50 && dist <= 180) {
        actionCounter++;
        controllerAddr = from;

        magnetCount = 0;
        lastHallState = digitalRead(hallPin);

        startDepthCm = dist;
        // compute number of rotations needed
        targetLowerCounts = computeTargetLowerCounts(startDepthCm);

        Serial.println(F("Starting cycle"));
        Serial.print(F("Action count: "));
        Serial.println(actionCounter);
        Serial.print(F("Controller address: "));
        Serial.println(controllerAddr);
        Serial.print(F("Measured depth cm: "));
        Serial.println(startDepthCm);
        Serial.print(F("Target lower distance cm: "));
        Serial.println(targetLowerDistanceCm);
        Serial.print(F("Target lower Hall counts: "));
        Serial.println(targetLowerCounts);

        currentState = LOWERING;
        stateTimer = millis();

        Serial.println(F("STATE: IDLE -> LOWERING"));
      } else {
        Serial.println(F("Trigger ignored: depth out of range"));
        Serial.print(F("Depth: "));
        Serial.println(dist);
      }
    } else if (len >= 1 && buf[0] == 'T') {
      Serial.print(F("Trigger ignored. Current state: "));
      Serial.println(currentState);
    }
  }

  bool countEnabled = (currentState == LOWERING || currentState == RAISING);
  updateMagnetCount(countEnabled);

  runSequence();
}

// ─── FSM ───────────────────────────────────────────────────────────────────
void runSequence() {
  switch (currentState) {
    case IDLE:
      break;

    case LOWERING: {
      int currentPWM = getMotorSpeedFromCounts(
        magnetCount,
        targetLowerCounts,
        MOTOR_MAX_SPEED_DOWN
      );

      printMotorStatus(magnetCount, targetLowerCounts, currentPWM, false, false);
      motorLower(currentPWM);

      if (magnetCount >= targetLowerCounts) {
        motorStop();

        targetRaiseCounts = magnetCount;   // Raise approximately same number of rotations
        magnetCount = 0;
        lastHallState = digitalRead(hallPin);

        currentState = WAIT_AT_BOTTOM;
        bottomTimer = millis();

        Serial.println(F("Lowering target reached"));
        Serial.print(F("Raise target Hall counts: "));
        Serial.println(targetRaiseCounts);
        Serial.println(F("STATE: LOWERING -> WAIT_AT_BOTTOM"));
      } 
      else if (millis() - stateTimer > LOWER_TIME_MS) {
        motorStop();

        targetRaiseCounts = magnetCount;
        if (targetRaiseCounts == 0) targetRaiseCounts = targetLowerCounts;

        magnetCount = 0;
        lastHallState = digitalRead(hallPin);

        currentState = WAIT_AT_BOTTOM;
        bottomTimer = millis();

        Serial.println(F("Lowering safety timeout reached"));
        Serial.print(F("Raise target Hall counts: "));
        Serial.println(targetRaiseCounts);
        Serial.println(F("STATE: LOWERING -> WAIT_AT_BOTTOM"));
      }

      break;
    }

    case WAIT_AT_BOTTOM:
      if (millis() - bottomTimer >= (unsigned long)TIME_SPENT_BOTTOM * 1000UL) {
        currentState = CHECK_DEPTH;
        Serial.println(F("Done waiting at bottom"));
        Serial.println(F("STATE: WAIT_AT_BOTTOM -> CHECK_DEPTH"));
      }
      break;

    case CHECK_DEPTH:
      // If you only want to raise when the measured depth is valid, keep this.
      // Otherwise, you can remove the if statement and always enter RAISING.
      if (dist >= 50 && dist <= 180) {
        magnetCount = 0;
        lastHallState = digitalRead(hallPin);

        currentState = RAISING;
        stateTimer = millis();

        Serial.println(F("Entered RAISING"));
        Serial.print(F("Depth before raising: "));
        Serial.println(dist);
        Serial.print(F("Raise target counts: "));
        Serial.println(targetRaiseCounts);
        Serial.println(F("STATE: CHECK_DEPTH -> RAISING"));
      } else {
        Serial.println(F("CHECK_DEPTH: depth out of range"));
        Serial.print(F("Depth: "));
        Serial.println(dist);
      }
      break;

    case RAISING: {
      int currentPWM = getMotorSpeedFromCounts(
        magnetCount,
        targetRaiseCounts,
        MOTOR_MAX_SPEED_UP
      );

      bool homeReached = (digitalRead(limitSwitchPin) == LIMIT_PRESSED);

      printMotorStatus(magnetCount, targetRaiseCounts, currentPWM, homeReached, true);

      if (!homeReached) {
        motorRaise(currentPWM);
      } else {
        motorStop();
        Serial.println(F("Home position reached by limit switch"));
        currentState = ROTATING;
        Serial.println(F("STATE: RAISING -> ROTATING"));
      }

      if (millis() - stateTimer > RAISE_TIME_MS) {
        motorStop();
        Serial.println(F("Raising safety timeout reached"));
        currentState = ROTATING;
        Serial.println(F("STATE: RAISING -> ROTATING"));
      }

      break;
    }

    case ROTATING: {
      int stepsToMove;

      if (samples == 7) {
        stepsToMove = FINAL_HALF_SLOT_STEPS;
        Serial.println(F("Eighth cycle: half-slot rotation"));
      } else {
        stepsToMove = STANDARD_SLOT_STEPS;
        Serial.println(F("Standard cycle: full-slot rotation"));
      }

      Serial.print(F("ROTATING | samples before: "));
      Serial.println(samples);
      Serial.print(F("Stepper steps: "));
      Serial.println(stepsToMove);

      myStepper.step(stepsToMove);
      stopStepper();

      samples++;

      Serial.print(F("Samples after: "));
      Serial.println(samples);

      sendSampleIncrementIfChanged();

      if (samples >= 8) {
        currentState = FULL;
        Serial.println(F("Storage is now FULL"));
        Serial.println(F("STATE: ROTATING -> FULL"));
      } else {
        currentState = IDLE;
        Serial.println(F("STATE: ROTATING -> IDLE"));
      }

      break;
    }

    case FULL: {
      static bool notifiedFull = false;

      if (!notifiedFull) {
        Serial.println(F("Storage Full - Notifying Controller"));
        sendSampleIncrementIfChanged();
        notifiedFull = true;
      }

      break;
    }
  }
}

// ─── Helpers ───────────────────────────────────────────────────────────────
void stopStepper() {
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);
  Serial.println(F("Stepper stopped"));
}

void sendSampleIncrementIfChanged() {
  static int lastSentSamples = -1;

  // Only send when samples increases
  if (samples <= lastSentSamples) return;

  uint16_t samplesToSend = (uint16_t)samples;
  uint8_t pkt[1 + sizeof(uint32_t) + sizeof(uint16_t)];

  pkt[0] = 'K';
  memcpy(&pkt[1], &actionCounter, sizeof(uint32_t));
  memcpy(&pkt[1 + sizeof(uint32_t)], &samplesToSend, sizeof(uint16_t));

  if (manager.sendtoWait(pkt, sizeof(pkt), controllerAddr)) {
    Serial.print(F("Sent sample increment | action/samples: "));
    Serial.print(actionCounter);
    Serial.print(F(" / "));
    Serial.println(samples);

    lastSentSamples = samples;
  } else {
    Serial.println(F("Failed to send sample increment"));
  }
}

void sendDepth(float d) {
  uint8_t pkt[1 + sizeof(float)];

  pkt[0] = 'D';
  memcpy(&pkt[1], &d, sizeof(float));

  static uint8_t seq = 0;
  manager.setHeaderId(++seq);
  manager.sendto(pkt, sizeof(pkt), SECONDARY_ADDR);
}

float measureHeight() {
  if (Serial1.available()) {
    if (Serial1.read() == HEADER) {
      uart[0] = HEADER;

      if (Serial1.read() == HEADER) {
        uart[1] = HEADER;

        for (i = 2; i < 9; i++) {
          uart[i] = Serial1.read();
        }

        check = uart[0] + uart[1] + uart[2] + uart[3] +
                uart[4] + uart[5] + uart[6] + uart[7];

        if (uart[8] == (check & 0xff)) {
          dist = uart[2] + uart[3] * 256;

          Serial.print(F("dist = "));
          Serial.println(dist);
        }
      }
    }
  }

  return dist;
}
