/*******************************************************************************************
 * fischertechnik simple robot arm 
 * Using FTlegacy library
 * 
 * This robot arm is from the PROFI COMPUTING kit (30390)
 *
 * Copyright (c) 2025 Jeroen Regtien
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 **********************************************************************************************/

/*
  In the Arduino IDE serial monitor a number of commands can be given 
  to test the individual robot arm movements.

  The 'd', 'u', 'f' and 'b' commands can be followed by a number <99

  's' : all motors stop
  'd' : upper arm down
  'f' : lower arm forward      f5 means five steps forward. 
  'b' : lower arm backward
  'l' : rotate left
  'r' : rotate right
  'c' : close gripper
  'o' : open grippe
  'm' : perform a movement of two stapled bricks

*/
#include <FTlegacy.h>

#define ROTATE ft_M1
#define ROTATE_PIN ft_E1
#define ROTATE_END ft_E4
#define UP_DOWN ft_M2
#define UP_DOWN_PIN ft_E2
#define UP_DOWN_END ft_E5
#define GRIPPER ft_M3
#define GRIPPER_PIN ft_E3
#define GRIPPER_END ft_E6

const char* programName = "Simple 30390 Arm";
FTlegacy interface(SER, 1);
FTcontroller controller(D2004);

bool armForward(int steps);
bool armDown(int steps);
bool rotate(int steps);
bool gripperOpen();
bool gripperClose();

FTencoderMotor verticalArm(&interface, UP_DOWN, E_STD, UP_DOWN_PIN);
FTencoderMotor rotateArm(&interface, ROTATE, E_STD, ROTATE_PIN);
FTencoderMotor gripper(&interface, GRIPPER, E_STD, GRIPPER_PIN);

int minPosition[4] = { 0, 0, 0, 0 };
int maxPosition[4] = { 0, 100, 32, 200 };
bool homeState[4] = { false, false, false, false };

const int moveMax = 14;
int movementRot[moveMax]  = {20,  0, 55,  0, -55,  0, 55,  0, -25, 25,  0, -55,  0,  0};
int movementArm[moveMax]  = {23,-20, 27,-25,  25,-20, 20,-20,   0, 30,-25,   0, 25,-25};
int movementGrip[moveMax] = { 1,  1,  0,  0,   1,  1,  0,  0,   0,  1,  1,   1,  0,  0};

void setup() {

  Serial.begin(9600);
  controller.begin(programName);
  interface.begin();
  verticalArm.begin();
  rotateArm.begin();
  resetAllStations();
  delay(2000);
}

String command, numberChar, choice;
int steps;

void loop() {

  interface.getInputs();

    // read strings from the serial monitor
  // a single letter may be followed by a number <99
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    choice = command.substring(0, 1);
    if (command.length() > 1) {
      numberChar = command.substring(1, 3);
      steps = int max(1, numberChar.toInt());
    }

    if (choice == "s") { Serial.println(F("stop")); interface.setAllMotorsSTOP(); }
    if (choice == "d") { Serial.println(F("down")); moveArm(steps);  }
    if (choice == "u") { Serial.println(F("up")); moveArm(-steps); }
    if (choice == "l") { Serial.println(F("left")); rotate(steps); }
    if (choice == "r") { Serial.println(F("right")); rotate(-steps); }
    if (choice == "c") { Serial.println(F("close")); gripperClose(); }
    if (choice == "o") { Serial.println(F("open")); gripperOpen();  }
    if (choice == "i") { Serial.println(F("initialise")); resetAllStations(); }
    if (choice == "m") { Serial.println(F("perform movement")); doMovement(); }
  }
  interface.ftUpdateDisplay();
}

void doMovement() {
  int r, a, g;

  resetAllStations();

  for (int i=0; i<moveMax; i++) {
    r = movementRot[i];
    a = movementArm[i];
    g = movementGrip[i];
    Serial.print("Move to point: ");
    Serial.print(r);
    Serial.print(",");
    Serial.print(a);
    Serial.print(",");
    Serial.println(g);

    rotate(r);
    moveArm(a);
    gripperAction(g);
  }
}

bool moveArm(int steps) {
  int rSteps, curPos;
  
  curPos = verticalArm.getPosition();
  if (steps>=0) {
    rSteps = min (curPos+steps, maxPosition[UP_DOWN]) - curPos;
  } else if (steps<0) {
    rSteps = max (curPos+steps, minPosition[UP_DOWN]) - curPos;
  }
  verticalArm.setSteps(rSteps);
}

bool rotate(int steps) {
  int rSteps, curPos;
  curPos = rotateArm.getPosition();
  if (steps>=0) {
    rSteps = min (curPos+steps, maxPosition[ROTATE]) - curPos;
  } else if (steps<0) {
    rSteps = max (curPos+steps, minPosition[ROTATE]) - curPos;
  }
  rotateArm.setSteps(rSteps);
}

bool resetRotation() {
  homeState[ROTATE] = true;
  return (rotateArm.findHome(ROTATE_END, CCW));
};

bool resetArmUpDown() {
  homeState[UP_DOWN] = true;
  return (verticalArm.findHome(UP_DOWN_END, CCW));
};


//
// reset all stations to avoid interference or clashes
bool resetAllStations() {
  resetArmUpDown();
  resetRotation();
  gripperOpen();
  return (true);
}

void gripperAction(int action) {
  if (action==0) {
    gripperOpen();
  }
  else {
    gripperClose();
  }
}

bool gripperOpen() {
  homeState[GRIPPER] = true;
  bool result = gripper.findHome(GRIPPER_END, CW);
  return (result);
}

bool gripperClose() {
  bool go_on = true;
  int startPos, newPos;
  FTtimer gripperTimer(6000);

  if (homeState[GRIPPER]) {
    startPos = gripper.getPosition();
    while (go_on) {
      startPos = gripper.getPosition();
      if (!gripperTimer.ready()) {
        gripper.moveRelative(-1);
        newPos = gripper.getPosition();
        if (newPos == startPos) {
          go_on = false;
        }
      } else {
        go_on = false;
      }
    }
    homeState[GRIPPER] = false;
  }
}
