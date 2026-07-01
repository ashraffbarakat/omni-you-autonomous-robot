/*
=========================================================
OMNIBOT MEGA VERSION (FULL MISSION SEQUENCE)

- Continuous Yaw Tracking (Fixes CW Rotation Bug Forever)
- Step 1: Forward 63 cm 
- Step 2: Rotate 90° CCW (Precise CCW Lock)
- Step 3 & 31: Forward 118 cm → Place Red Box 
- Step 32: Backward 22 cm
- Step 4: Rotate 185° CCW (Precise CCW Lock to exactly 272 deg) 
- Step 5 & 51: Left 220 cm → Place Green Box
- Step 6 (NEW): Right 37 cm (Clear the Green Box)
- Step 7: FORWARD 145 cm 
- Step 9 & 91: Diagonal Forward-Right 110 cm → Place Blue Box
=========================================================
*/

#include <Wire.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

unsigned long stepStartTime = 0;   
unsigned long settleTime = 0;      
unsigned long telemetryTimer = 0;

// ===== AUTO COMMANDS =====
#define CMD_RED   "AR"
#define CMD_GREEN "AG"
#define CMD_BLUE  "AB"

// ===== PI COMMUNICATION FLAGS =====
bool waitingForPi = false;
bool redDone   = false;
bool greenDone = false;
bool blueDone  = false;

// ===== FRONT LEFT =====
#define FL_IN1 53
#define FL_IN2 51
#define FL_EN  6

// ===== FRONT RIGHT =====
#define FR_IN1 10
#define FR_IN2 11
#define FR_EN  4

// ===== BACK LEFT =====
#define BL_IN1 50
#define BL_IN2 52
#define BL_EN  7

// ===== BACK RIGHT =====
#define BR_IN1 12
#define BR_IN2 13
#define BR_EN  5

// ===== ENCODER PINS =====
#define FL_ENCA 19
#define FL_ENCB 35
#define FR_ENCA 2
#define FR_ENCB 23
#define BL_ENCA 18
#define BL_ENCB 31
#define BR_ENCA 3
#define BR_ENCB 27

// === System Limits & Tuning ===
const float PPR = 308.0;
const int MIN_PWM = 60;
const int interval = 50;
const double maxTargetRPM = 120.0;
const double maxAutoRPM = 80.0;

// === Odometry Constants ===
const float WHEEL_DIAMETER = 9.7; // cm
const float CM_PER_TICK = (PI * WHEEL_DIAMETER) / PPR;
const float SLIP_FACTOR_X = 0.85;
const float SLIP_FACTOR_Y = 0.85;

// === Autonomous Sequence Variables ===
int autoSequenceStep = 0;
double currentPosX = 0.0, targetPosX = 0.0;
double currentPosY = 0.0, targetPosY = 0.0;
double Kp_pos = 4.0;

// === Motor PID Tuning ===
double Kp = 3.0, Ki = 5.0, Kd = 0.0;

// === Yaw PID Tuning ===
double Kp_yaw = 3.5, Ki_yaw = 0.1, Kd_yaw = 1.5;

MPU6050 mpu;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// === CONTINUOUS YAW VARIABLES ===
double yawOffset = 0.0;
double previousYaw = 0.0; 
double currentYaw = 0.0;  
double yawSetpoint = 0.0;
double yawInput = 0.0;
double yawOutput = 0.0;

PID yawPID(&yawInput, &yawOutput, &yawSetpoint, Kp_yaw, Ki_yaw, Kd_yaw, DIRECT);

double setX = 0, setY = 0, setW = 0;
double targetFL, targetFR, targetBL, targetBR;
double inputFL, inputFR, inputBL, inputBR;
double setpointFL, setpointFR, setpointBR, setpointBL;
double pwmFL, pwmFR, pwmBL, pwmBR;
double actualRPM_FL, actualRPM_FR, actualRPM_BL, actualRPM_BR;

volatile long ticksFL = 0, ticksFR = 0, ticksBL = 0, ticksBR = 0;
long prevFL = 0, prevFR = 0, prevBL = 0, prevBR = 0;

PID pidFL(&inputFL, &pwmFL, &setpointFL, Kp, Ki, Kd, DIRECT);
PID pidFR(&inputFR, &pwmFR, &setpointFR, Kp, Ki, Kd, DIRECT);
PID pidBL(&inputBL, &pwmBL, &setpointBL, Kp, Ki, Kd, DIRECT);
PID pidBR(&inputBR, &pwmBR, &setpointBR, Kp, Ki, Kd, DIRECT);

unsigned long previousMillis = 0;

// === Encoder ISRs ===
void countFL() { if (digitalRead(FL_ENCB) == LOW) ticksFL++; else ticksFL--; }
void countFR() { if (digitalRead(FR_ENCB) == LOW) ticksFR--; else ticksFR++; }
void countBL() { if (digitalRead(BL_ENCB) == LOW) ticksBL++; else ticksBL--; }
void countBR() { if (digitalRead(BR_ENCB) == LOW) ticksBR--; else ticksBR++; }

// === Motor Drive Function ===
void driveMotor(int in1, int in2, int en, double target, double &pwm, PID &motorPID) {
  if (abs(target) < 15) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(en, 0);
    motorPID.SetMode(MANUAL);
    pwm = 0;
    motorPID.SetMode(AUTOMATIC);
  } else {
    int finalPWM = 0;
    if (pwm > 0) {
      finalPWM = map((int)pwm, 0, 255, MIN_PWM, 255);
    }
    
    if (target > 0) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    analogWrite(en, finalPWM);
  }
}

void setup() {
  Serial.begin(115200);  
  Serial2.begin(9600);   

  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT); pinMode(FL_EN, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT); pinMode(FR_EN, OUTPUT);
  pinMode(BL_IN1, OUTPUT); pinMode(BL_IN2, OUTPUT); pinMode(BL_EN, OUTPUT);
  pinMode(BR_IN1, OUTPUT); pinMode(BR_IN2, OUTPUT); pinMode(BR_EN, OUTPUT);

  pinMode(FL_ENCA, INPUT_PULLUP); pinMode(FL_ENCB, INPUT_PULLUP);
  pinMode(FR_ENCA, INPUT_PULLUP); pinMode(FR_ENCB, INPUT_PULLUP);
  pinMode(BL_ENCA, INPUT_PULLUP); pinMode(BL_ENCB, INPUT_PULLUP);
  pinMode(BR_ENCA, INPUT_PULLUP); pinMode(BR_ENCB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FL_ENCA), countFL, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_ENCA), countFR, RISING);
  attachInterrupt(digitalPinToInterrupt(BL_ENCA), countBL, RISING);
  attachInterrupt(digitalPinToInterrupt(BR_ENCA), countBR, RISING);

  pidFL.SetMode(AUTOMATIC); pidFL.SetOutputLimits(0, 255);
  pidFR.SetMode(AUTOMATIC); pidFR.SetOutputLimits(0, 255);
  pidBL.SetMode(AUTOMATIC); pidBL.SetOutputLimits(0, 255);
  pidBR.SetMode(AUTOMATIC); pidBR.SetOutputLimits(0, 255);
  
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(-35, 35);

  pidFL.SetSampleTime(interval);
  pidFR.SetSampleTime(interval);
  pidBL.SetSampleTime(interval);
  pidBR.SetSampleTime(interval);
  yawPID.SetSampleTime(interval);

  Wire.begin();
  Wire.setClock(400000);
  
  mpu.initialize();
  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
    Serial.println("MPU6050 READY. Please wait for calibration...");
    
    delay(5000);
    
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      yawOffset = ypr[0] * 180.0 / M_PI;
      previousYaw = 0.0;
      currentYaw = 0.0;
    }
  }
  Serial.println("SYSTEM READY");

}

void loop() {
  /* ================= RASPBERRY PI RESPONSES ================= */
  if (Serial.available()) {
    char res = Serial.read();
    if(res == 'D') {
      Serial.println("TASK_DONE");
      if(autoSequenceStep == 31) redDone = true;
      else if(autoSequenceStep == 51) greenDone = true;
      else if(autoSequenceStep == 91) blueDone = true;
      waitingForPi = false;
    }
    else if(res == 'N') {
      Serial.println("TASK_FAILED");
    }
  }

  /* ================= BLUETOOTH COMMANDS ================= */
  if (Serial2.available()) {
    char cmd = Serial2.read();

    if (autoSequenceStep == 0) {
      if (cmd == 'F')      { setX = maxTargetRPM;  setY = 0; setW = 0; }
      else if (cmd == 'B') { setX = -maxTargetRPM; setY = 0; setW = 0; }
      else if (cmd == 'L') { setX = 0; setY = -maxTargetRPM; setW = 0; }
      else if (cmd == 'R') { setX = 0; setY = maxTargetRPM;  setW = 0; }
      else if (cmd == 'H') { setX = 0; setY = 0; setW = -maxTargetRPM; }
      else if (cmd == 'J') { setX = 0; setY = 0; setW = maxTargetRPM;  }
      else if (cmd == 'S') { setX = 0; setY = 0; setW = 0; }
      else if (cmd == 'Z') {
        yawSetpoint = currentYaw; 
        Serial.println("YAW LOCKED");
      }
      else if (cmd == 'V') {
        setX = 0; setY = 0; setW = 0; 
        Serial.println("V");          
      }
      else if (cmd == 'W') {
        setX = 0; setY = 0; setW = 0; 
        Serial.println("W");
      }
    }

    
   // Start Full Mission
    if (cmd == 'A') {
      autoSequenceStep = 1;
      stepStartTime = millis();
      currentPosX = 0;
      targetPosX = 63.0; // Step 1: Forward 63 cm
      setX = 0; setY = 0; setW = 0;
      yawSetpoint = currentYaw;
      Serial.print("START MISSION | Locked Yaw = ");
      Serial.println(yawSetpoint);
    }
    if (cmd == 'X') {
      autoSequenceStep = 0;
      setX = 0; setY = 0; setW = 0;
      Serial.println("EMERGENCY STOP!");
    }
  }

  /* ================= MAIN CONTROL LOOP ================= */
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

   // 1. MPU6050 - CONTINUOUS YAW TRACKING
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
      double rawYaw = (ypr[0] * 180.0 / M_PI) - yawOffset;
      double deltaY = rawYaw - previousYaw;
      
      if (deltaY > 180.0) deltaY -= 360.0;
      else if (deltaY < -180.0) deltaY += 360.0;
      
      currentYaw += deltaY; 
      previousYaw = rawYaw;
    }

    // 2. Encoders
    noInterrupts();
    long cFL = ticksFL; long cFR = ticksFR; long cBL = ticksBL; long cBR = ticksBR;
    interrupts();

    long dFL = cFL - prevFL; long dFR = cFR - prevFR; long dBL = cBL - prevBL; long dBR = cBR - prevBR;
    prevFL = cFL; prevFR = cFR; prevBL = cBL; prevBR = cBR;

    // 3. RPM Calculation & DYNAMIC SPOOFING
    actualRPM_FL = abs((dFL * 60000.0) / (PPR * interval));
    actualRPM_FR = abs((dFR * 60000.0) / (PPR * interval));
    actualRPM_BL = abs((dBL * 60000.0) / (PPR * interval));
    
    if (abs(setX) > abs(setY)) {
      actualRPM_BR = actualRPM_FR; 
    } 
    else if (abs(setY) > abs(setX)) {
      actualRPM_BR = actualRPM_BL; 
    } 
    else {
      actualRPM_BR = actualRPM_FL; 
    }

    // --- Odometry ---
    float distFL = dFL * CM_PER_TICK;
    float distFR = dFR * CM_PER_TICK;
    
    float deltaX = ((distFL + distFR) / 2.0) * SLIP_FACTOR_X;
    float deltaY = ((distFR - distFL) / 2.0) * SLIP_FACTOR_Y;

    
// ==========================================
// Full Mission State Machine
// ==========================================

// Step 1: Move Forward 63 cm
if (autoSequenceStep == 1) {
  currentPosX += deltaX;
  double errorX = targetPosX - currentPosX;

  setX = errorX * Kp_pos;
  if (setX > maxAutoRPM) setX = maxAutoRPM;
  if (setX < -maxAutoRPM) setX = -maxAutoRPM;
  setY = 0; setW = 0;

  if (abs(errorX) < 3.0 || millis() - stepStartTime > 6000) {
    setX = 0;
    autoSequenceStep = 2;
    stepStartTime = millis();
    settleTime = 0; 
    
    yawSetpoint += 90.0; 
    Serial.println("Step 1 Done. Rotating 87 deg CCW.");
  }
}

// Step 2: Rotate 90 deg CCW (With Active Braking)
else if (autoSequenceStep == 2) {
  setX = 0; setY = 0; setW = 0;

  double yError = abs(yawSetpoint - currentYaw);

  bool isSettled = false;
  if (yError <= 2.0) {
    if (settleTime == 0) settleTime = millis();
    if (millis() - settleTime > 600) isSettled = true; 
  } else {
    settleTime = 0; 
  }

  if (isSettled || millis() - stepStartTime > 5000) {
    autoSequenceStep = 3;
    stepStartTime = millis();
    currentPosX = 0;
    targetPosX = 123.0; 
    Serial.println("Step 2 Done. Moving Forward 118cm.");
  }
}

// Step 3: Move Forward 123 cm
else if (autoSequenceStep == 3) {
  currentPosX += deltaX;
  double errorX = targetPosX - currentPosX;

  setX = errorX * Kp_pos;
  if (setX > maxAutoRPM) setX = maxAutoRPM;
  if (setX < -maxAutoRPM) setX = -maxAutoRPM;
  setY = 0; setW = 0; 

  if (abs(errorX) < 3.0 || millis() - stepStartTime > 6000) {
    setX = 0;
    autoSequenceStep = 31;
    redDone = false;
    waitingForPi = true;
    Serial.println(CMD_RED);
    Serial.println("RED WAIT");
  }
}

// Step 31: Place Red Box
else if (autoSequenceStep == 31) {
  setX = 0; setY = 0; setW = 0;
 if (redDone) {
 
    autoSequenceStep = 32; 
    stepStartTime = millis();
    currentPosX = 0;
    targetPosX = -22.0; // التحديث لـ 25 سم لورا بناءً على الكومنت بتاعك
    Serial.println("Red Placed. Moving Backward 25cm to clear the box.");
 }
}

// Step 32: Move Backward 22 cm
else if (autoSequenceStep == 32) {
  currentPosX += deltaX; 
  double errorX = targetPosX - currentPosX;

  setX = errorX * Kp_pos;
  if (setX > maxAutoRPM) setX = maxAutoRPM;
  if (setX < -maxAutoRPM) setX = -maxAutoRPM;
  setY = 0; setW = 0; 

  if (abs(errorX) < 2.0 || millis() - stepStartTime > 4000) {
    setX = 0;
    autoSequenceStep = 4;
    stepStartTime = millis();
    settleTime = 0;
    
    yawSetpoint += 180.0;
    Serial.println("Cleared Box. Rotating 185 deg CCW.");
  }
}

// Step 4: Rotate 180 deg CCW
else if (autoSequenceStep == 4) {
  setX = 0; setY = 0; setW = 0;

  double yError = abs(yawSetpoint - currentYaw);

  bool isSettled = false;
  if (yError <= 2.0) {
    if (settleTime == 0) settleTime = millis();
    if (millis() - settleTime > 800) isSettled = true; 
  } else {
    settleTime = 0;
  }

  if (isSettled || millis() - stepStartTime > 8000) {
    autoSequenceStep = 5;
    stepStartTime = millis();
    currentPosY = 0;
    targetPosY = 220.0; // 220 سم بناءً على الكومنت
    Serial.println("Step 4 Done. Moving Left 164cm (Without IMU)");
  }
}

// Step 5: Move Left 220 cm
else if (autoSequenceStep == 5) {
  currentPosY += deltaY;
  double errorY = targetPosY - currentPosY;

  setY = errorY * Kp_pos;
  if (setY > maxAutoRPM) setY = maxAutoRPM;
  if (setY < -maxAutoRPM) setY = -maxAutoRPM;
  setX = 0; setW = 0; 

  if (abs(errorY) < 3.0 || millis() - stepStartTime > 6000) {
    setY = 0;
    autoSequenceStep = 51; 
    greenDone = false;
    waitingForPi = true;
    Serial.println(CMD_GREEN);
    Serial.println("GREEN WAIT");
  }
}

// Step 51: Place Green Box
else if (autoSequenceStep == 51) {
  setX = 0; setY = 0; setW = 0;

  if (greenDone) {
    autoSequenceStep = 6; // الخطوة الجديدة: الحركة يمين 25 سم
    stepStartTime = millis();
    currentPosY = 0;
    targetPosY = -50.0; // السالب في اتجاه الـ Y معناه يمين
    Serial.println("Green Placed. Moving Right 25cm.");
  }
}

// Step 6 (NEW): Move Right 50 cm
else if (autoSequenceStep == 6) {
  currentPosY += deltaY; 
  double errorY = targetPosY - currentPosY; 

  setY = errorY * Kp_pos;
  if (setY > maxAutoRPM) setY = maxAutoRPM;
  if (setY < -maxAutoRPM) setY = -maxAutoRPM;
  setX = 0; setW = 0; 

  if (abs(errorY) < 2.0 || millis() - stepStartTime > 4000) {
    setY = 0;
    autoSequenceStep = 7;
    stepStartTime = millis();
    currentPosX = 0;
    targetPosX = 145.0; 
    Serial.println("Step 6 Done. Moving Forward 148cm.");
  }
}

// Step 7: Move Forward 145 cm
else if (autoSequenceStep == 7) {
  currentPosX += deltaX;
  double errorX = targetPosX - currentPosX;

  setX = errorX * Kp_pos;
  if (setX > maxAutoRPM) setX = maxAutoRPM;
  if (setX < -maxAutoRPM) setX = -maxAutoRPM;
  setY = 0; setW = 0; 

  if (abs(errorX) < 3.0 || millis() - stepStartTime > 7000) {
    setX = 0;
    autoSequenceStep = 9;
    stepStartTime = millis();
    currentPosX = 0;
    currentPosY = 0;
    Serial.println("Step 7 Done. Moving Diagonal 120cm.");
  }
}

// Step 9: Diagonal Forward-Right 110 cm
else if (autoSequenceStep == 9) {
  // التصحيح الفيزيائي: عجلة FL هي النشطة في حركة قدام-يمين
  float distFL = dFL * CM_PER_TICK;
  float deltaDiag = distFL * 0.85; 
  
  currentPosX += deltaDiag; 
  
  double errorDist = 110.0 - currentPosX; // التحديث لـ 120 سم

  double speedOut = errorDist * Kp_pos;
  if (speedOut > maxAutoRPM) speedOut = maxAutoRPM;
  if (speedOut < -maxAutoRPM) speedOut = -maxAutoRPM;

  // توجيه صحيح لقدام-يمين (+X للأمام و -Y لليمين)
  setX = speedOut * 0.707;
  setY = -speedOut * 0.707;
  setW = 0; 

  if (abs(errorDist) < 3.0 || millis() - stepStartTime > 6000) {
    setX = 0;
    setY = 0;
    autoSequenceStep = 91;
    blueDone = false;
    waitingForPi = true;
    Serial.println(CMD_BLUE);
    Serial.println("BLUE WAIT");
  }
}

// Step 91: Place Blue Box & Finish
else if (autoSequenceStep == 91) {
  setX = 0; setY = 0; setW = 0;
if (blueDone) {
    autoSequenceStep = 0;
    Serial.println("====== MISSION ACCOMPLISHED ======");
}
}

    // =========================================================
    // 4. Yaw Control (IMU ACTIVE ONLY FOR ROTATION)
    // =========================================================
    if (autoSequenceStep == 2 || autoSequenceStep == 4) {
      yawInput = currentYaw; 
      
      double yErrAbs = abs(yawSetpoint - currentYaw);
      if (yErrAbs <= 1.0) {
        yawOutput = 0;
      } else {
        yawPID.Compute();
      }
    }
    // فصل الـ IMU تماماً في أي خطوة مشي
    else if (autoSequenceStep > 0) {
      yawOutput = 0;
    }
    else {
      if (abs(setW) > 1) {
         yawSetpoint = currentYaw;
         yawOutput = 0;
      } else {
         yawOutput = 0; 
      }
    }

    // 5. Mecanum Kinematics
    double totalW = setW + yawOutput;
    
    targetFL = setX - setY - totalW;
    targetFR = setX + setY + totalW;
    targetBL = setX + setY - totalW;
    targetBR = setX - setY + totalW;

    // 6. Motor PID
    inputFL = actualRPM_FL;   setpointFL = abs(targetFL);   pidFL.Compute();
    inputFR = actualRPM_FR;   setpointFR = abs(targetFR);   pidFR.Compute();
    inputBL = actualRPM_BL;   setpointBL = abs(targetBL);   pidBL.Compute();
    inputBR = actualRPM_BR;   setpointBR = abs(targetBR);   pidBR.Compute();

    // 7. Motor Drive
    driveMotor(FL_IN1, FL_IN2, FL_EN, targetFL, pwmFL, pidFL);
    driveMotor(FR_IN1, FR_IN2, FR_EN, targetFR, pwmFR, pidFR);
    driveMotor(BL_IN1, BL_IN2, BL_EN, targetBL, pwmBL, pidBL);
    driveMotor(BR_IN1, BR_IN2, BR_EN, targetBR, pwmBR, pidBR);
  }

if (millis() - telemetryTimer >= 100) {
  telemetryTimer = millis();
  Serial.print("T,");
  Serial.print(autoSequenceStep);
  Serial.print(",");
  Serial.print((int)actualRPM_FL);
  Serial.print(",");
  Serial.print((int)actualRPM_FR);
  Serial.print(",");
  Serial.print((int)actualRPM_BL);
  Serial.print(",");
  Serial.print((int)actualRPM_BR);
  Serial.print(",");
  Serial.print((int)currentPosX);
  Serial.print(",");
  Serial.print((int)currentPosY);
  Serial.print(",");
  Serial.println((int)currentYaw);
}

}