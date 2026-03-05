#include <ESP32Servo.h>

// ── Color Sensor ──────────────────────────────────────
#define S0  32
#define S1   2
#define S2  15
#define S3  22
#define OUT 39

// ── Motor Driver ──────────────────────────────────────
#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 23
#define ENA 33
#define ENB 25

// ── IR Sensors ────────────────────────────────────────
#define IR_LEFT   17
#define IR_MID    16
#define IR_RIGHT   4

#define JUNCTION_DEBOUNCE_MS 60

// ── Servo Pins ────────────────────────────────────────
#define BASE_PIN    13
#define ARM_PIN     19
#define GRIPPER_PIN 21

// ── Servo Angle Positions ─────────────────────────────
#define HOME_BASE        137
#define HOME_ARM          39
#define HOME_GRIPPER     144
#define SEARCH_BASE      137
#define SEARCH_ARM       113
#define SEARCH_GRIPPER   143
#define HARVEST_BASE     137
#define HARVEST_ARM      113
#define HARVEST_GRIPPER   87
#define CONTAINER_BASE   135
#define CONTAINER_ARM      0
#define CONTAINER_GRIPPER 87
#define DROP_GRIPPER     114

// ── Color Calibration ─────────────────────────────────
const int RED_R   = 23,  RED_G   = 119, RED_B   = 81;
const int GREEN_R = 77,  GREEN_G =  59, GREEN_B = 78;
#define UNKNOWN_THRESHOLD 150

// ── Servo move speed ──────────────────────────────────
#define SERVO_STEP_DELAY 12

// ── Line follow correction delay (ms) ────────────────
// How long to correct before checking sensors again
// Lower = more responsive, Higher = smoother
#define CORRECT_TIME 80

Servo baseServo, armServo, gripperServo;

int currentBase    = HOME_BASE;
int currentArm     = HOME_ARM;
int currentGripper = HOME_GRIPPER;

unsigned long unknownStartTime    = 0;
bool          unknownTimerStarted = false;

// ══════════════════════════════════════════════════════
//  MOTOR FUNCTIONS
//  ENA/ENB = HIGH always — no ledcAttach
//  All LEDC channels free for ESP32Servo
// ══════════════════════════════════════════════════════

void setupMotors() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
}

// Both motors forward — equal speed
void goStraight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

// Drift left — left motor stops, right motor runs
// Brings bot back so only mid sees black
void correctLeft() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);   // left stop
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // right forward
  delay(CORRECT_TIME);
}

// Drift right — right motor stops, left motor runs
// Brings bot back so only mid sees black
void correctRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // left forward
  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);   // right stop
  delay(CORRECT_TIME);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  digitalWrite(ENA, LOW); digitalWrite(ENB, LOW);
}

// ══════════════════════════════════════════════════════
//  IR — JUNCTION DETECTION
// ══════════════════════════════════════════════════════

bool allIRDetectBlack() {
  bool l = digitalRead(IR_LEFT)  == HIGH;
  bool m = digitalRead(IR_MID)   == HIGH;
  bool r = digitalRead(IR_RIGHT) == HIGH;

  if (l && m && r) {
    unsigned long start = millis();
    while (millis() - start < JUNCTION_DEBOUNCE_MS) {
      if (digitalRead(IR_LEFT)  != HIGH ||
          digitalRead(IR_MID)   != HIGH ||
          digitalRead(IR_RIGHT) != HIGH) {
        return false;
      }
    }
    return true;
  }
  return false;
}

// ══════════════════════════════════════════════════════
//  SERVO HELPERS
// ══════════════════════════════════════════════════════

void moveServo(Servo &servo, int &current, int target) {
  if (current == target) return;
  int step = (target > current) ? 1 : -1;
  while (current != target) {
    current += step;
    servo.write(current);
    delay(SERVO_STEP_DELAY);
  }
}

void moveAll(int baseTarget, int armTarget, int gripperTarget) {
  while (currentBase    != baseTarget ||
         currentArm     != armTarget  ||
         currentGripper != gripperTarget) {
    if (currentBase != baseTarget) {
      currentBase += (baseTarget > currentBase) ? 1 : -1;
      baseServo.write(currentBase);
    }
    if (currentArm != armTarget) {
      currentArm += (armTarget > currentArm) ? 1 : -1;
      armServo.write(currentArm);
    }
    if (currentGripper != gripperTarget) {
      currentGripper += (gripperTarget > currentGripper) ? 1 : -1;
      gripperServo.write(currentGripper);
    }
    delay(SERVO_STEP_DELAY);
  }
}

// ══════════════════════════════════════════════════════
//  COLOR SENSOR
// ══════════════════════════════════════════════════════

void setupColorSensor() {
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

int readChannel(int s2val, int s3val) {
  digitalWrite(S2, s2val);
  digitalWrite(S3, s3val);
  delay(10);
  return pulseIn(OUT, LOW, 100000);
}

void readRGB(int &r, int &g, int &b) {
  r = readChannel(LOW,  LOW);
  g = readChannel(HIGH, HIGH);
  b = readChannel(LOW,  HIGH);
}

void readRGBSmoothed(int &r, int &g, int &b, int samples = 5) {
  long sumR = 0, sumG = 0, sumB = 0;
  for (int i = 0; i < samples; i++) {
    int tr, tg, tb;
    readRGB(tr, tg, tb);
    sumR += tr; sumG += tg; sumB += tb;
    delay(10);
  }
  r = sumR / samples;
  g = sumG / samples;
  b = sumB / samples;
}

String detectColor() {
  int r, g, b;
  readRGBSmoothed(r, g, b, 5);
  long distRed   = abs(r - RED_R)   + abs(g - RED_G)   + abs(b - RED_B);
  long distGreen = abs(r - GREEN_R) + abs(g - GREEN_G) + abs(b - GREEN_B);
  Serial.print("Color -> R:"); Serial.print(r);
  Serial.print(" G:"); Serial.print(g);
  Serial.print(" B:"); Serial.print(b);
  Serial.print(" | dRed:"); Serial.print(distRed);
  Serial.print(" dGreen:"); Serial.println(distGreen);
  if (min(distRed, distGreen) > UNKNOWN_THRESHOLD) return "UNKNOWN";
  return (distRed < distGreen) ? "RED" : "GREEN";
}

// ══════════════════════════════════════════════════════
//  ARM SEQUENCE
// ══════════════════════════════════════════════════════

void runArmSequence() {
  enum ArmState {
    GO_HOME, GO_SEARCH, SCANNING,
    HARVESTING, GO_CONTAINER, DROPPING, ARM_DONE
  };

  ArmState      armState          = GO_HOME;
  bool          armUnknownStarted = false;
  unsigned long armUnknownTime    = 0;

  while (armState != ARM_DONE) {
    switch (armState) {

      case GO_HOME:
        Serial.println("[STATE] Going to HOME");
        moveAll(HOME_BASE, HOME_ARM, HOME_GRIPPER);
        armUnknownStarted = false;
        delay(500);
        armState = GO_SEARCH;
        break;

      case GO_SEARCH:
        Serial.println("[STATE] Moving to SEARCH position");
        moveAll(SEARCH_BASE, SEARCH_ARM, SEARCH_GRIPPER);
        delay(500);
        armState = SCANNING;
        break;

      case SCANNING: {
        Serial.println("[STATE] Scanning...");
        String color = detectColor();

        if (color == "RED") {
          Serial.println(">>> RED — harvesting");
          armUnknownStarted = false;
          armState = HARVESTING;

        } else if (color == "GREEN") {
          Serial.println(">>> GREEN — skipping");
          armUnknownStarted = false;
          moveAll(HOME_BASE, HOME_ARM, HOME_GRIPPER);
          delay(500);
          armState = ARM_DONE;

        } else {
          if (!armUnknownStarted) {
            armUnknownStarted = true;
            armUnknownTime    = millis();
            Serial.println(">>> UNKNOWN — searching 4.5s...");
          }
          unsigned long elapsed = millis() - armUnknownTime;
          Serial.print("  searching... "); Serial.print(elapsed / 1000); Serial.println("s");

          if (elapsed >= 4500) {
            Serial.println(">>> Timeout — skipping");
            armUnknownStarted = false;
            moveAll(HOME_BASE, HOME_ARM, HOME_GRIPPER);
            delay(500);
            armState = ARM_DONE;
          }
        }
        break;
      }

      case HARVESTING:
        Serial.println("[STATE] HARVESTING - closing gripper");
        moveServo(gripperServo, currentGripper, HARVEST_GRIPPER);
        delay(600);
        armState = GO_CONTAINER;
        break;

      case GO_CONTAINER:
        Serial.println("[STATE] Moving to CONTAINER position");
        moveServo(armServo,  currentArm,  CONTAINER_ARM);
        delay(300);
        moveServo(baseServo, currentBase, CONTAINER_BASE);
        delay(300);
        armState = DROPPING;
        break;

      case DROPPING:
        Serial.println("[STATE] DROPPING block");
        moveServo(gripperServo, currentGripper, DROP_GRIPPER);
        delay(800);
        Serial.println(">>> Block dropped! Returning home...");
        moveAll(HOME_BASE, HOME_ARM, HOME_GRIPPER);
        delay(500);
        armState = ARM_DONE;
        break;

      case ARM_DONE:
        break;
    }
  }
}

// ══════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);

  setupMotors();
  setupColorSensor();

  pinMode(IR_LEFT,  INPUT);
  pinMode(IR_MID,   INPUT);
  pinMode(IR_RIGHT, INPUT);

  baseServo.attach(BASE_PIN);
  armServo.attach(ARM_PIN);
  gripperServo.attach(GRIPPER_PIN);

  baseServo.write(HOME_BASE);
  armServo.write(HOME_ARM);
  gripperServo.write(HOME_GRIPPER);
  delay(1000);

  Serial.println("System ready — following line!");
  goStraight();
}

// ══════════════════════════════════════════════════════
//  MAIN LOOP
// ══════════════════════════════════════════════════════

void loop() {
  bool L = digitalRead(IR_LEFT)  == HIGH;
  bool M = digitalRead(IR_MID)   == HIGH;
  bool R = digitalRead(IR_RIGHT) == HIGH;

  // ── ALL 3 BLACK = junction ────────────────────────
  if (L && M && R) {
    Serial.println("[JUNCTION] Stopping...");
    stopMotors();
    delay(200);
    runArmSequence();
    Serial.println("[DRIVE] Resuming...");
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);
    goStraight();
    delay(800);
  }

  // ── ONLY MID BLACK = on line → straight ──────────
  else if (!L && M && !R) {
    goStraight();
  }

  // ── LEFT + MID BLACK = drifting left → correct ───
  else if (L && M && !R) {
    Serial.println("DRIFT LEFT → correcting");
    correctLeft();
    goStraight();
  }

  // ── MID + RIGHT BLACK = drifting right → correct ─
  else if (!L && M && R) {
    Serial.println("DRIFT RIGHT → correcting");
    correctRight();
    goStraight();
  }

  // ── ANYTHING ELSE → keep straight ────────────────
  else {
    goStraight();
  }
}