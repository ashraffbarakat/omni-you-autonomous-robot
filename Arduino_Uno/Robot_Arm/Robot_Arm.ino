#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150
#define SERVOMAX 575

#define LED_RED   8
#define LED_GREEN 9
#define LED_BLUE  10

// =====================
// Servo Channels
// =====================

#define BASE_CH      5
#define SHOULDER_CH 12
#define ELBOW_CH     1
#define GRIPPER_CH   7

// =====================

String buffer = "";

bool waiting = false;

// =====================
// Current Angles
// =====================

int elbowAngle = 120;
int shoulderAngle = 55;
int baseAngle = 0;
int gripperAngle = 45;

// =====================

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void moveServoSmooth(int channel, int fromAngle, int toAngle, int stepDelay) {
  if (fromAngle < toAngle) {
    for (int angle = fromAngle; angle <= toAngle; angle++) {
      pwm.setPWM(channel, 0, angleToPulse(angle));
      delay(stepDelay);
    }
  } else {
    for (int angle = fromAngle; angle >= toAngle; angle--) {
      pwm.setPWM(channel, 0, angleToPulse(angle));
      delay(stepDelay);
    }
  }
}

// =====================
// LED
// =====================

void lightLED(String color)
{
  digitalWrite(LED_RED,LOW);
  digitalWrite(LED_GREEN,LOW);
  digitalWrite(LED_BLUE,LOW);

  if(color=="Red")
    digitalWrite(LED_RED,HIGH);

  else if(color=="Green")
    digitalWrite(LED_GREEN,HIGH);

  else if(color=="Blue")
    digitalWrite(LED_BLUE,HIGH);

  delay(5000);

  digitalWrite(LED_RED,LOW);
  digitalWrite(LED_GREEN,LOW);
  digitalWrite(LED_BLUE,LOW);
}

// =====================
// PICK FUNCTIONS
// =====================

void pickRed()
{
   moveServoSmooth(7, gripperAngle, 100, 10);
  gripperAngle = 100;
  delay(500);
  moveServoSmooth(5, baseAngle, 165, 15); // rotate
  baseAngle = 165;
  delay(500);
  moveServoSmooth(12, shoulderAngle, 80, 10);           //shoulder
  shoulderAngle = 80;
  delay(500);
  moveServoSmooth(1, elbowAngle, 95, 15);
  elbowAngle = 95;
  delay(500);
  moveServoSmooth(7, gripperAngle,45, 10);
  gripperAngle = 45;

  // Return Home
  delay(1000);

  moveServoSmooth(1, elbowAngle, 120, 10); // elbow
  elbowAngle = 120;
  delay(500);
moveServoSmooth(SHOULDER_CH, shoulderAngle, 55, 10); //shoulder
  shoulderAngle = 55;
  delay(500);

  moveServoSmooth(5, baseAngle, 0, 15);
  baseAngle = 0;

   
  Serial.println("D");
}

void pickGreen()
{
  // ======================== PICK ========================
  moveServoSmooth(7, gripperAngle, 100, 10);
  gripperAngle = 100;
  delay(500);
  moveServoSmooth(5, baseAngle, 135, 15); // rotate
  baseAngle = 135;
  delay(500);
  moveServoSmooth(12, shoulderAngle, 77, 10);           //shoulder
  shoulderAngle = 77;
  delay(500);
  moveServoSmooth(1, elbowAngle, 90, 15);
  elbowAngle = 90;
  delay(500);
  moveServoSmooth(7, gripperAngle,45, 10);
  gripperAngle = 45;

  // Return Home
  delay(1000);

  moveServoSmooth(1, elbowAngle, 120, 10); // elbow
  elbowAngle = 120;
  delay(500);
moveServoSmooth(SHOULDER_CH, shoulderAngle, 55, 10); //shoulder
  shoulderAngle = 55;
  delay(500);

  moveServoSmooth(5, baseAngle, 0, 15);
  baseAngle = 0;

  delay(2000);



  Serial.println("D");
}

void pickBlue()
{
   moveServoSmooth(7, gripperAngle, 100, 10);
  gripperAngle = 100;
  delay(500);
  moveServoSmooth(5, baseAngle, 100, 15); // rotate
  baseAngle = 100;
  delay(500);
   moveServoSmooth(12, shoulderAngle, 70, 10);           //shoulder
  shoulderAngle = 70;
  delay(500);
  moveServoSmooth(1, elbowAngle, 70, 15);
  elbowAngle = 70;
  delay(500);
  moveServoSmooth(7, gripperAngle,45, 10);
  gripperAngle = 45;

  // Return Home
  delay(1000);

  moveServoSmooth(1, elbowAngle, 120, 10); // elbow
  elbowAngle = 120;
  delay(500);
moveServoSmooth(SHOULDER_CH, shoulderAngle, 55, 10); //shoulder
  shoulderAngle = 55;
  delay(500);

  moveServoSmooth(5, baseAngle, 0, 15);
  baseAngle = 0;


  Serial.println("D");
}

// =====================
// PLACE FUNCTIONS
// =====================

void placeRed()
{
moveServoSmooth(5, baseAngle, 160, 15); // rotate
  baseAngle = 160;
  delay(500);
   moveServoSmooth(12, shoulderAngle, 105, 10);           //shoulder
  shoulderAngle = 105;
  delay(500);
  moveServoSmooth(1, elbowAngle, 103, 15);
  elbowAngle = 103;
  delay(500);
  moveServoSmooth(7, gripperAngle, 100, 10);
  gripperAngle = 100;
  delay(500);

  //home position
  moveServoSmooth(SHOULDER_CH, shoulderAngle, 55, 10); //shoulder
  shoulderAngle = 55; 
  delay(500);
  moveServoSmooth(1, elbowAngle, 120, 10); // elbow
  elbowAngle = 120;
   moveServoSmooth(5, baseAngle, 0, 15);
  baseAngle = 0;
  moveServoSmooth(7, gripperAngle,45, 10);
  gripperAngle = 45;
  
  Serial.println("D");
}

void placeGreen()
{
     moveServoSmooth(5, baseAngle, 135, 15); // rotate
  baseAngle = 135;
  delay(500);
  moveServoSmooth(12, shoulderAngle, 95, 10);           //shoulder
  shoulderAngle = 95;
  delay(500);
  moveServoSmooth(1, elbowAngle, 90, 15);
  elbowAngle = 90;
  delay(500);
  moveServoSmooth(7, gripperAngle, 100, 10);
  gripperAngle = 100;
  delay(500);
  // place box to position 
  moveServoSmooth(SHOULDER_CH, shoulderAngle, 55, 10); //shoulder
  shoulderAngle = 55;
  moveServoSmooth(1, elbowAngle, 120, 10); // elbow
  elbowAngle = 120;
  delay(500);
 moveServoSmooth(SHOULDER_CH, shoulderAngle, 55, 10); //shoulder
  shoulderAngle = 55;
  delay(500);
  moveServoSmooth(5, baseAngle, 0, 15);
  baseAngle = 0;
  moveServoSmooth(7, gripperAngle,45, 10);
  gripperAngle = 45;

  Serial.println("D");
}

void placeBlue()
{
   moveServoSmooth(5, baseAngle, 100, 15); // rotate
  baseAngle = 100;
  delay(500);
   moveServoSmooth(1, elbowAngle, 82, 15);
  elbowAngle = 82;
  delay(500);
  moveServoSmooth(12, shoulderAngle, 90, 10);           //shoulder
  shoulderAngle = 90;
 
  delay(500);
  moveServoSmooth(7, gripperAngle, 100, 10);
  gripperAngle = 100;
  delay(500);
  // place box to position 
  moveServoSmooth(SHOULDER_CH, shoulderAngle, 55, 10); //shoulder
  shoulderAngle = 55;
  moveServoSmooth(1, elbowAngle, 120, 10); // elbow
  elbowAngle = 120;
  delay(500);
 moveServoSmooth(SHOULDER_CH, shoulderAngle, 55, 10); //shoulder
  shoulderAngle = 55;
  delay(500);
  moveServoSmooth(5, baseAngle, 0, 15);
  baseAngle = 0;
  moveServoSmooth(7, gripperAngle,45, 10);
  gripperAngle = 45;

  Serial.println("D");
}

// =====================
// Commands
// =====================

void handle(String cmd)
{
  cmd.trim();

  // ========= LEDs =========

  if(cmd=="LRed")
  {
    lightLED("Red");
    return;
  }

  if(cmd=="LGreen")
  {
    lightLED("Green");
    return;
  }

  if(cmd=="LBlue")
  {
    lightLED("Blue");
    return;
  }

  // ========= Manual =========

  if(cmd=="W")
  {
    waiting = true;
    return;
  }

  if(waiting)
  {
    if(cmd=="Red")
      pickRed();

    else if(cmd=="Green")
      pickGreen();

    else if(cmd=="Blue")
      pickBlue();

    waiting = false;

    return;
  }

  // ========= Auto =========

  if(cmd=="AR")
  {
    placeRed();
    return;
  }

  if(cmd=="AG")
  {
    placeGreen();
    return;
  }

  if(cmd=="AB")
  {
    placeBlue();
    return;
  }
}



void setup()
{
  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(50);

  pinMode(LED_RED,OUTPUT);
  pinMode(LED_GREEN,OUTPUT);
  pinMode(LED_BLUE,OUTPUT);


 moveServoSmooth(1, elbowAngle, 120, 10); // elbow
  elbowAngle = 120;
  delay(500);
 moveServoSmooth(12, shoulderAngle, 40, 10); //shoulder
  shoulderAngle = 40;
  delay(500);
  moveServoSmooth(5, baseAngle, 0, 10); // base
  baseAngle = 0;
  delay(500);
  moveServoSmooth(7, gripperAngle,45, 10);
  gripperAngle = 45;

}

// =====================
// Loop
// =====================

void loop()
{
  while(Serial.available())
  {
    char c = Serial.read();

    if(c=='\n' || c=='\r')
    {
      if(buffer.length())
      {
        handle(buffer);

        buffer = "";
      }
    }
    else
    {
      buffer += c;
    }
  }
}