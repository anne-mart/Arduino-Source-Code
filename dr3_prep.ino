
#include <Wire.h>
#include <VL53L1X.h>
#include <ezButton.h>

ezButton limitSwitch(10);
ezButton limitSwitchVertical(12);
VL53L1X lidar;

#define DIR 4
#define PUL 5
#define ENCODER_A 2
#define ENCODER_B 3
#define SENSOR_PIN A0
#define VACUUM_PUMP 13
#define DIR_END 6
#define PUL_END 7
#define DIR_VERTICAL 8
#define PUL_VERTICAL 9
#define SENSOR_PIN A0
#define AVG_WINDOW_SIZE 5
#define PI 3.1415926535897932384626433832795
#define SERIAL_BAUD 115200
#define MAX_SAMPLES 5
#define MAX_THETAS_PER_SAMPLE 5

const int microStepEndTool = 4;
const float anglePerStepEndTool = 1.8;
const float stepsPerRevolutionEndTool = microStepEndTool * (360.0 / anglePerStepEndTool);
const float gearRatioEndTool = 20;
const long stepsPerOutputRevolutionEndTool = stepsPerRevolutionEndTool * gearRatioEndTool;
const int speedRPMEndTool = 240;
const float delayBetweenStepsEndTool = (60.0 / (speedRPMEndTool * stepsPerRevolutionEndTool)) * 1000000;
volatile long encoderCount = 0;
volatile bool indexDetected = false;
const float gearRatio = 50.16;
const float countsPerMotorRev = 4000.0;
const float countsPerOutputRev = countsPerMotorRev * gearRatio;
const float turn90Const = (countsPerOutputRev * 90) / 360.0;

float xs[MAX_SAMPLES][MAX_THETAS_PER_SAMPLE];
float ys[MAX_SAMPLES][MAX_THETAS_PER_SAMPLE];
float thetas[MAX_SAMPLES][MAX_THETAS_PER_SAMPLE];
int thetaCounts[MAX_SAMPLES];
String sampleNames[MAX_SAMPLES];
int sampleCount = 0;

int i = 0;
String inputString = "";
bool newData = false;
bool endReceived = false;
const String END_MARKER = "<END>";
bool lidarPresent = false;

void setup() {
  pinMode(SENSOR_PIN, INPUT);
  
  limitSwitch.setDebounceTime(50);
  limitSwitchVertical.setDebounceTime(50);

  Serial.begin(SERIAL_BAUD);
  delay(50);

  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(VACUUM_PUMP, OUTPUT);
  pinMode(DIR_END, OUTPUT);
  pinMode(PUL_END, OUTPUT);
  pinMode(DIR_VERTICAL, OUTPUT);
  pinMode(PUL_VERTICAL, OUTPUT);
  digitalWrite(DIR_END, HIGH);
  pinMode(SENSOR_PIN, INPUT);

  // encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), handleEncoderB, CHANGE);

  // --- VL53L1X init (robust: no infinite hang) ---
  Wire.begin();
  lidar.setTimeout(200); // short timeout

  if (lidar.init()) {
    lidarPresent = true;
    lidar.setDistanceMode(VL53L1X::Long);
    lidar.setMeasurementTimingBudget(50000);  // 50 ms sample budget
    lidar.startContinuous(50);                // continuous with 50ms between measurements
  } else {
    lidarPresent = false;
  }
  delay(1000);
}

// MAIN CONTROL LOOP ============================================================================
void loop() {
  readSerial();

  if (newData) {
    if (inputString.startsWith("MOVE_UP")) {
       digitalWrite(DIR_VERTICAL, HIGH);
       delay(500);
        moveUp(12000);
        inputString = "";
        newData = false;
        return;
    }
    
    if (endReceived) {
      delay(500);
      homeArm();
      delay(500);

      for (int s = 0; s < sampleCount; s++) {
        digitalWrite(DIR_VERTICAL, LOW);
        delay(500);
        moveDownUntilDistance(65);
        delay(5000);

        digitalWrite(VACUUM_PUMP, HIGH);
        delay(10000);
        
        digitalWrite(DIR_VERTICAL, HIGH);
        delay(500);
        moveUp(100000);
        delay(500);

        long counts90 = (countsPerOutputRev * 90.0) / 360.0;
        encoderCount = 0;
        digitalWrite(DIR, LOW);
        delay(2000);
        while (abs(encoderCount) < counts90) {
          digitalWrite(PUL, HIGH);
          delayMicroseconds(1000);
          digitalWrite(PUL, LOW);
          delayMicroseconds(1000);
        }
        delay(2000);

// === 2. PROCESS EACH THETA ==========================================
        for (int t = 0; t < thetaCounts[s]; t++) {
          float x = xs[s][t];
          float y = ys[s][t];
          float theta = thetas[s][t];  // Already calculated relative to (3,3) by Python
        
          x = x - 3;
          y = y - 3;
          
          // Convert θ to target step count
          float distance = sqrt((x*x) + (y*y));
          float tryAgain = (distance * 360)/(25 * PI);
          float targetSteps = (countsPerOutputRev * tryAgain) / 360.0;

          encoderCount = 0;
          digitalWrite(DIR, LOW);
          delay(2000);
          delayMicroseconds(1000);
          while (abs(encoderCount) < targetSteps) {
            digitalWrite(PUL, HIGH);
            delayMicroseconds(1000);
            digitalWrite(PUL, LOW);
            delayMicroseconds(1000);
          }
          delay(1000);

          // → Counter-rotate the sample by the same angle so selected point stays at (3,3)
          moveMotorByAngle(theta);

          // move down so that sample sits on top of NMR mouse
          
          
          // we want to wait for TTL pulse here
          while (digitalRead(A0) != HIGH) {
            // do nothing -- waiting for TTL pulse
          }

          moveMotorByAngleReverse(theta);
          delay(3000);

          // ASM TODO :: see if this should go outside of the t loop
          // → Return arm -theta back to 90°
          encoderCount = 0;
          digitalWrite(DIR, HIGH);
          delay(2000);
          while (abs(encoderCount) < targetSteps) {
            digitalWrite(PUL, HIGH);
            delayMicroseconds(1000);
            digitalWrite(PUL, LOW);
            delayMicroseconds(1000);
          }
          delay(2000); // now back at 90°
        }

        // GO BACK 45 AND DISPOSE OF SAMPLE =============================================
        encoderCount = 0;
        digitalWrite(DIR, HIGH);
        delay(2000);
        while (abs(encoderCount) < counts90/2) {
          digitalWrite(PUL, HIGH);
          delayMicroseconds(1000);
          digitalWrite(PUL, LOW);
          delayMicroseconds(1000);
        }
        delay(2000);
        
        // drop sample
        digitalWrite(VACUUM_PUMP, LOW);
        delay(500);
        
        homeArm();
        delay(2000);
      }

      sampleCount = 0;
      endReceived = false;
    } else {
      parseMessage(inputString);
    }

    inputString = "";
    newData = false;
  }
}

/*** Serial / parsing helpers ***/
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    inputString += c;

    if (inputString.endsWith(END_MARKER)) {
      inputString.remove(inputString.length() - END_MARKER.length());
      endReceived = true;
      if (inputString.length() > 0) newData = true;
      return;
    }
    if (c == '\n') {
      newData = true;
      return;
    }
  }
}

void parseMessage(String msg) {
  int firstComma = msg.indexOf(',');
  if (firstComma == -1) return;
  int sampleIndex = msg.substring(0, firstComma).toInt();
  msg.remove(0, firstComma + 1);

  int secondComma = msg.indexOf(',');
  if (secondComma == -1) return;
  String sampleName = msg.substring(0, secondComma);
  msg.remove(0, secondComma + 1);

  if (sampleIndex - 1 < MAX_SAMPLES) {
    sampleNames[sampleIndex - 1] = sampleName;
  }

  int thetaIndex = 0;
  while (msg.length() > 0 && thetaIndex < MAX_THETAS_PER_SAMPLE) {
    int semiPos = msg.indexOf(';');
    String coordSet;
    if (semiPos != -1) {
      coordSet = msg.substring(0, semiPos);
      msg.remove(0, semiPos + 1);
    } else {
      coordSet = msg;
      msg = "";
    }

    int c1 = coordSet.indexOf(',');
    int c2 = coordSet.indexOf(',', c1 + 1);
    if (c1 != -1 && c2 != -1) {
      float x = coordSet.substring(0, c1).toFloat();
      float y = coordSet.substring(c1 + 1, c2).toFloat();
      float theta = coordSet.substring(c2 + 1).toFloat();
      xs[sampleIndex - 1][thetaIndex] = x;
      ys[sampleIndex - 1][thetaIndex] = y;
      thetas[sampleIndex - 1][thetaIndex++] = theta;
    }
  }

  thetaCounts[sampleIndex - 1] = thetaIndex;
  if (sampleIndex > sampleCount) sampleCount = sampleIndex;
}

/*** Encoder ISRs ***/
void handleEncoderA() {
  bool A = digitalRead(ENCODER_A);
  bool B = digitalRead(ENCODER_B);
  if (A == B) encoderCount++;
  else encoderCount--;
}

void handleEncoderB() {
  bool A = digitalRead(ENCODER_A);
  bool B = digitalRead(ENCODER_B);
  if (A != B) encoderCount++;
  else encoderCount--;
}

void handleIndex() {
  indexDetected = true;
}

/*** Motor helpers (unchanged except minor fixes) ***/
void turnArmDegrees(long targetCounts) {
  while (abs(encoderCount) < targetCounts) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(1000);
    digitalWrite(PUL, LOW);
    delayMicroseconds(1000);
  }
  delay(3000);
}

void moveMotorByAngle(float theta) {
  int dir = HIGH;
  delay(2000);
  if (theta > 180) dir = LOW;
  theta = 180 - theta;
  digitalWrite(DIR_END, dir);
  delay(2000);
  long stepsToMove = (long)((abs(theta) / 360.0) * stepsPerOutputRevolutionEndTool);
  for (long i = 0; i < stepsToMove; i++) {
    digitalWrite(PUL_END, HIGH);
    delayMicroseconds(1);
    digitalWrite(PUL_END, LOW);
    delay(1);
  }
}

void moveMotorByAngleReverse(float theta) {
  int dir = LOW;
  if (theta > 180) dir = HIGH;
  theta = 180 - theta;
  digitalWrite(DIR_END, dir);
  delay(2000);
  long stepsToMove = (long)((abs(theta) / 360.0) * stepsPerOutputRevolutionEndTool);
  for (long i = 0; i < stepsToMove; i++) {
    digitalWrite(PUL_END, HIGH);
    delayMicroseconds(1);
    digitalWrite(PUL_END, LOW);
    delay(1);
  }
}

void moveUp(long steps) {
  //digitalWrite(DIR_VERTICAL, LOW);
  delayMicroseconds(1);
  for (long i = 0 ; i < steps ; i++) {
    digitalWrite(PUL_VERTICAL, HIGH);
    delayMicroseconds(180);
    digitalWrite(PUL_VERTICAL, LOW);
    delayMicroseconds(180);
  }
}
void moveDown(long steps) {
  digitalWrite(DIR_VERTICAL, HIGH);
  delayMicroseconds(1);
  for (long i = 0 ; i < steps ; i++) {
    digitalWrite(PUL_VERTICAL, HIGH);
    delayMicroseconds(180);
    digitalWrite(PUL_VERTICAL, LOW);
    delayMicroseconds(180);
  }
}

//void moveDownUntilDistance(int targetMM) {
//  const int tolerance = 10;
//  const int maxBad = 5;
//  int badCount = 0;
//  unsigned long lastStepTime = 0;
//  const unsigned long stepDelayMicroseconds = 400;
//
//  // Variables for Averaging
//  int readings[AVG_WINDOW_SIZE];
//  int readingIndex = 0;
//  int goodReadingCount = 0;
//
//  if (!lidarPresent) {
//    Serial.println("LiDAR not present: fallback to moveDown(2000) steps.");
//    moveDown(2000);
//    return;
//  }
//
//  // Pre-fill the array with a safe, high value
//  for (int i = 0; i < AVG_WINDOW_SIZE; i++) {
//    readings[i] = 4000;
//  }
//
//  while (true) {
//    // *** Non-blocking I2C read attempt ***
//    if (lidar.dataReady()) {
//      int distance = lidar.read();
//
//      if (distance > 0 && distance < 4000) {
//        // --- Successful Reading ---
//        badCount = 0;
//        
//        // 1. Add new reading to the buffer
//        readings[readingIndex] = distance;
//        readingIndex = (readingIndex + 1) % AVG_WINDOW_SIZE; // Roll index
//
//        // 2. Increment good reading count until it reaches window size
//        if (goodReadingCount < AVG_WINDOW_SIZE) {
//            goodReadingCount++;
//        }
//        
//        // 3. Calculate Average distance
//        long sum = 0;
//        for (int i = 0; i < goodReadingCount; i++) {
//          sum += readings[i];
//        }
//        // Use the actual count of good readings, not the array size
//        int averageDistance = sum / goodReadingCount; 
//
//        // 4. Check if the average has reached the target
//        if (goodReadingCount == AVG_WINDOW_SIZE && averageDistance <= targetMM + tolerance) {
//          break; // Exit the loop and stop the motor
//        }
//
//      } else {
//        // --- Bad Reading ---
//        badCount++;
//        if (badCount > maxBad) {
//          break;
//        }
//      }
//    }
//
//    // *** Motor Stepping (time-based, non-blocking step) ***
//    if (micros() - lastStepTime >= stepDelayMicroseconds) {
//      // Toggle PUL pin for one step cycle (200us HIGH, 200us LOW total = 400us)
//      digitalWrite(PUL_VERTICAL, HIGH);
//      delayMicroseconds(200);
//      digitalWrite(PUL_VERTICAL, LOW);
//      delayMicroseconds(200);
//      lastStepTime = micros();
//    }
//  }
//}
void moveDownUntilDistance(int targetMM) {
  const int tolerance = 10;
  const int maxBad = 5;
  int badCount = 0;
  unsigned long lastStepTime = 0;
  const unsigned long stepDelayMicroseconds = 400;

  // Variables for Averaging
  int readings[AVG_WINDOW_SIZE];
  int readingIndex = 0;
  int goodReadingCount = 0;

  if (!lidarPresent) {
    Serial.println("LiDAR not present: fallback to moveDown(2000) steps.");
    moveDown(2000);
    return;
  }

  // Pre-fill the array with a safe, high value
  for (int i = 0; i < AVG_WINDOW_SIZE; i++) {
    readings[i] = 4000;
  }

  // Set direction to move down
  digitalWrite(DIR_VERTICAL, LOW);
  
  while (true) {
    // 1. Check Limit Switch (MUST be first, highest priority stop condition)
    limitSwitchVertical.loop(); // Update the button state
    if (limitSwitchVertical.isPressed()) {
      Serial.println("Vertical Limit Switch Pressed: Stopping vertical movement.");
      break; // Exit the loop and stop the motor
    }

    // 2. LiDAR Distance Check (Non-blocking I2C read attempt)
    if (lidar.dataReady()) {
      int distance = lidar.read();

      if (distance > 0 && distance < 4000) {
        // --- Successful Reading ---
        badCount = 0;
        
        // a. Add new reading to the buffer
        readings[readingIndex] = distance;
        readingIndex = (readingIndex + 1) % AVG_WINDOW_SIZE; // Roll index

        // b. Increment good reading count until it reaches window size
        if (goodReadingCount < AVG_WINDOW_SIZE) {
            goodReadingCount++;
        }
        
        // c. Calculate Average distance
        long sum = 0;
        for (int i = 0; i < goodReadingCount; i++) {
          sum += readings[i];
        }
        // Use the actual count of good readings, not the array size
        int averageDistance = sum / goodReadingCount;  

        // d. Check if the average has reached the target
        if (goodReadingCount == AVG_WINDOW_SIZE && averageDistance <= targetMM + tolerance) {
          Serial.print("Target Distance Reached (Avg: ");
          Serial.print(averageDistance);
          Serial.println("mm): Stopping vertical movement.");
          break; // Exit the loop and stop the motor
        }

      } else {
        // --- Bad Reading ---
        badCount++;
        if (badCount > maxBad) {
          Serial.println("Max bad LiDAR readings reached: Stopping vertical movement.");
          break; // Exit the loop and stop the motor
        }
      }
    }

    // 3. Motor Stepping (time-based, non-blocking step)
    if (micros() - lastStepTime >= stepDelayMicroseconds) {
      // Toggle PUL pin for one step cycle (200us HIGH, 200us LOW total = 400us)
      digitalWrite(PUL_VERTICAL, HIGH);
      delayMicroseconds(120);
      digitalWrite(PUL_VERTICAL, LOW);
      delayMicroseconds(120);
      lastStepTime = micros();
    }
  }
}
void testVerticalLimit() {
  digitalWrite(DIR_VERTICAL, HIGH);
  delayMicroseconds(1000);
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    digitalWrite(PUL_VERTICAL, HIGH);
    delayMicroseconds(400);
    digitalWrite(PUL_VERTICAL, LOW);
    delayMicroseconds(400);
  }
  delay(1000);

  digitalWrite(DIR_VERTICAL, LOW);
  delay(20);

  while (true) {
    limitSwitchVertical.loop();
    if (limitSwitchVertical.isPressed()) break;
    digitalWrite(PUL_VERTICAL, HIGH);
    delayMicroseconds(400);
    digitalWrite(PUL_VERTICAL, LOW);
    delayMicroseconds(400);
  }
}

void homeArm() {
  delay(1000);
  // first move away from switch (in event already hit it)
   digitalWrite(DIR, LOW);   // move away from switch
   delay(1000);
    for (int i = 0; i < 400; i++) {
      limitSwitch.loop();
      digitalWrite(PUL, HIGH);
      delayMicroseconds(800);
      digitalWrite(PUL, LOW);
      delayMicroseconds(800);
    }
    
  digitalWrite(DIR, HIGH);
  delay(1000);
  limitSwitch.loop();
  delay(1000);
  while (true) {
    limitSwitch.loop();
    if (limitSwitch.isPressed()) break;
    digitalWrite(PUL, HIGH);
    delayMicroseconds(1000);
    digitalWrite(PUL, LOW);
    delayMicroseconds(1000);
  }
  encoderCount = 0;
  delay(300);
  for (int i = 0; i < 10; i++) limitSwitch.loop();
}

int getFilteredDistance() {
  if (!lidarPresent) return -1;  // no lidar available

  const uint8_t N = 5;           // number of samples
  int readings[N];

  // collect the samples
  for (uint8_t i = 0; i < N; i++) {
    readings[i] = lidar.read();
    delay(5); // very small delay between reads
  }

  // sort 5 items (insertion sort because N is tiny)
  for (uint8_t i = 1; i < N; i++) {
    int key = readings[i];
    int j = i - 1;
    while (j >= 0 && readings[j] > key) {
      readings[j + 1] = readings[j];
      j--;
    }
    readings[j + 1] = key;
  }

  // return the MEDIAN of 5 sorted samples
  return readings[N / 2];
}
