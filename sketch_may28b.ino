#include <HCPCA9685.h>

//using servo driver library HCPCA9685


#include <Servo.h>

#define I2CAdd 0x40
#include <Stepper.h>
#define STEPS 200

#define motorInterfaceType 1

HCPCA9685 ServoController(I2CAdd);
unsigned int pos;

//Pin Connectors on PCA9685 Servo Driver

//Shoulder pins Left:2 Right: 12
//Shoulder Neutral Positions: 2 pin 262, 12 pin 175
//Elbow Neutral Position: 294 Pin: 9
//Wrist Joint: Pin: 15 Neutral Position: 255
//Clamp Pin: 0

//Conversion from position to angle

//2 Pos per Degree of Angle


const int dirPin = 4;
const int stepPin = 5;
const int stepsPerRevolution = 200;

int leftPos = 262;
int rightPos = 175;
int ePos = 294;
int hPos = 255;
int stepDelay = 9500;

double s = 18; //shoulder limb length
double e = 12; //elbow limb length
double h = 19.5; //hand limb length
void setup() {
  ServoController.Init(SERVO_MODE);
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  /* Wake the device up */
  ServoController.Sleep(false);

  Serial.begin(4800);  // Initialise default communication rate of the Bluetooth module


  delay(12000);
  // Declare pins as Outputs
  PickUpAndDrop(24, -24, 10);
}
void loop() {
}

//Counterclockwise rotation of base with given angle
void baseRotateCounterClock(double ang) {
  //setting motor rotation to HIGH corresponding to CCW
  digitalWrite(dirPin, HIGH);
  // Spin motor
  double steps = (ang / 360) * stepsPerRevolution * 4;
  Serial.print(steps);
  for (int x = 0; x < steps; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }
  delay(2000);
}

//Clockwise rotation of base with given angle
void baseRotateClock(double ang) {
  //Sets motor direction pin to LOW corresponding to clockwise
  digitalWrite(dirPin, LOW);
  // Spin motor
  double steps = (ang / 360) * stepsPerRevolution * 4;
  for (int x = 0; x < steps; x++) {
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
  }
  delay(2000);  // Wait a second
}

//Opens Clamp
void OpenGrip() {
  ServoController.Servo(0, 300);
}
//Closes Clamp
void CloseGrip() {
  ServoController.Servo(0,170);
}
//Moves Shoulder a given angle
void MoveShoulder(int angle) {
  //262 is the neutral position of the left shoulder motor
  //The left and right shoulder motors move in opposite directions to move the arm a certain angle.
  for (pos = 262 - leftPos; pos < abs(angle) * 2; pos++) {
    if (angle > 0) {
      leftPos--;
      rightPos++;
    } else {
      leftPos++;
      rightPos--;
    }
    //updates the corresponding pins in the servo driver to the right position
    ServoController.Servo(2, leftPos);
    ServoController.Servo(12, rightPos);
    delay(20);
  }
  delay(1000);
}

//Moves Elbow joint a given angle
void MoveElbow(double angle) {
  for (pos = ePos - 294; pos < abs(angle) * 2; pos++) {
    //updates elbow position incrementally
    if (angle >= 0) {
      ePos++;
    } else {
      ePos--;
    }
    //Updates corresponding pin for servo motor at pin 8 on servo driver
    ServoController.Servo(8, ePos);
    delay(10);
  }
  delay(1000);
}

//Moves Hand joint a given angle
void MoveWrist(double angle) {
  for (pos = 255 - hPos; pos < abs(angle) * 2; pos++) {
    if (angle >= 0) {
      hPos--;
    } else {
      hPos++;
    }
    //Updates servo motor at pin 15 (hand joint);
    ServoController.Servo(15, hPos);
    delay(20);
  }
  delay(1000);
}


//variable to indicate whether the hand joint points downards or upwards, since it alters calculation of the two bar endpoint
bool down = false;

//angles from verticals for shoulder, elbow, and hand respectively.
double theta1;
double theta2;
double theta3;

//Test angles for tracing back endpoint for TwoBar linkage (shoulder and elbow joints)
double angle = -2000;


//Moves Arm in polar coordinates a distance r from the base and a distance z from the ground (ignores base)
void MoveArm(double r, double z) {
  //Logic for the following methods explained in project portfolio with diagrams
  for (int ang = 0; ang < 90; ang++) {
    if (sqrt(pow(r - h * cos(ang * 3.14159 / 180), 2) + pow(z - h * sin(ang * 3.14159 / 180), 2)) < s + e && (z - h * sin(ang * 3.14159 / 180)) >= 0) {
      down = false;
      angle = ang;
      TwoBar(r - h * cos(ang * 3.14159 / 180), z - h * sin(ang * 3.14159 / 180));
      if (theta1 < -90 || theta1 > 90 || theta2 < -90 || theta2 > 75 || theta3 < -90 || theta3 > 90) {
        angle = -2000;
        continue;
      } else {
        break;
      }
    } else if (sqrt(pow(r - h * cos(ang * 3.14159 / 180), 2) + pow(z + h * sin(ang * 3.14159 / 180), 2)) < s + e) {
      down = true;
      angle = ang;
      TwoBar(r - h * cos(ang * 3.14159 / 180), z + h * sin(ang * 3.14159 / 180));
      if (theta1 < -90 || theta1 > 90 || theta2 < -90 || theta2 > 75 || theta3 < -90 || theta3 > 90) {
        angle = -2000;
        continue;
      } else {
        break;
      }
    }
  }
  if (angle == -2000) {
    Serial.print("Not possible");
    return;
  }
  Serial.println(theta1);
  Serial.println(theta2);
  Serial.println(theta3);
  Serial.print("Angle: ");
  Serial.println(angle);

  MoveShoulder(theta1);
  delay(500);
  MoveElbow(theta2);
  delay(500);
  MoveWrist(theta3);
  delay(500);
}

//Calculates theta1, theta2, and theta3 given a r and z endpoint for the two bar.
void TwoBar(double r, double z) {
  theta2 = acos((pow(r, 2) + pow(z, 2) - pow(s, 2) - pow(e, 2)) / (2 * s * e));
  double beta = atan((e * sin(theta2)) / (s + e * cos(theta2))) * (180 / 3.14159);
  double gamma = atan(z / r) * (180 / 3.15159);
  theta2 = theta2 * (180 / 3.14159);
  if (theta2 >= 0) {
    theta1 = 90 - gamma - beta;
  } else {
    theta1 = 90 - gamma + beta;
  }
  if (down == false) {
    theta3 = (90 - angle) - theta1 - theta2;
  } else {
    theta3 = (90 + angle) - theta1 - theta2;
  }
  Serial.println(r);
  Serial.println(z);
}

//Important because after firstNeutral the computer knows the position of each motor
bool firstNeutral = true;

//Moves each motor to its neutral position(robot goes straight up)
void MoveNeutral() {
  if(firstNeutral == true) {
    ServoController.Servo(2, 262);
    ServoController.Servo(12, 175);
  } else {
    for (int pos = leftPos; pos < 262; pos++) {
      leftPos++;
      rightPos--;
      ServoController.Servo(2, leftPos);
      ServoController.Servo(12, rightPos);
    }
  }
  
  delay(700);
  ServoController.Servo(8, 294);
  delay(700);
  ServoController.Servo(15, 255);
  delay(700);
}

//Combines previous methods to pick up an appropriate object given its x, y, and z position
void PickUpAndDrop(double x, double y, double z) {
  //Starts off the robot in neutral position
  MoveNeutral();
  delay(1000);
  //Opens grip so that it can grab the object
  OpenGrip();
  delay(1000);
  
  //finds the angle to move the base and in which direction
  double baseAng = atan(y/x) * 180/3.14159;
  Serial.print(baseAng);
  //moves the base
  if (x >= 0 && y >= 0) {
    baseRotateCounterClock(baseAng);
  } else if (x <= 0 && y >= 0) {
    baseRotateCounterClock(180 + baseAng);
  } else if (x <= 0 && y <= 0) {
    baseRotateClock(180 - baseAng);
  } else {
    baseRotateClock(-1*baseAng);
  }

  //Converts x, y, and z coordinates to r and z coordinates that can be inputted to move arm

  //Moves arm to correct position
  MoveArm(pow(pow(x,2)+pow(y,2),0.5), z);
  delay(1000);

  //Clamps the object
  CloseGrip();
  delay(1000);

  //Goes back to the neutral position
  MoveNeutral();
  delay(1000);

  //Moves robot to position of the place to drop the object in the bin
  if (x >= 0 && y >= 0) {
    baseRotateClock(baseAng);
  } else if (x <= 0 && y >= 0) {
    baseRotateClock(180 + baseAng);
  } else if (x <= 0 && y <= 0) {
    baseRotateCounterClock(180 - baseAng);
  } else {
    baseRotateCounterClock(-1*baseAng);
  }
  delay(1000);
  MoveArm(30,5);
  delay(1000);

  //Releases the grip
  OpenGrip();
  delay(1000);

  //Returns to neutral
  MoveNeutral();
  
}
