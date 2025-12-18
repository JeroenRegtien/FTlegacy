/*******************************************************************************************
 * fischertechnik Computing box 30554 with parallel interface. 
 * Using FTlegacy library
 * 
 * Plotter Model 1: Doing a single step
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
  Interface:
    E1  : rotatie right
    E2  : opto-coupler rotation 
    E3  : nul-stand shoulder (upper arm)
    E4  : opto-coupler schoulder 
    E5  : nul-stand elbow (lower arm)
    E6  : opto-coupler elleboog 
    E7  : gripper position 
    E8  : STOP / RESET (not used in this program)

  In the Arduino IDE serial monitor a number of commands can be given 
  to test the individual robot arm movements.

  's' : all motors stop
  'd' : upper arm down
  'f' : lower arm forward
  'b' : lower arm backward
  'l' : rotate left
  'r' : rotate right
  'c' : close gripper
  'o' : open grippe
  'm1u': motor 1 up
  'm2u': motor 2 up

*/
#include <FTlegacy.h>

#define ROTATE ft_M1
#define ROTATE_PIN ft_E2
#define ROTATE_END ft_E1
#define FORWARD ft_M2
#define FORWARD_PIN ft_E4
#define FORWARD_END ft_E3
#define UP_DOWN ft_M3
#define UP_DOWN_PIN ft_E6
#define UP_DOWN_END ft_E5
#define GRIPPER ft_M4
#define GRIPPER_PIN ft_E8
#define GRIPPER_END ft_E7

const char* programName = "FTencoder 96782";
FTlegacy interface(SER, 1);
FTcontroller controller(D2004);

bool armForward(int steps);
bool armDown(int steps);
bool rotate(int steps);
bool gripperOpen();
bool gripperClose();

FTencoderMotor verticalArm(&interface, UP_DOWN, E_STD, UP_DOWN_PIN);
FTencoderMotor horizontalArm(&interface, FORWARD, E_STD, FORWARD_PIN);
FTencoderMotor rotateArm(&interface, ROTATE, E_STD, ROTATE_PIN);
FTencoderMotor gripper(&interface, GRIPPER, E_STD, GRIPPER_PIN);

int minPosition[4] = { 0, 0, 0, 0 };
int maxPosition[4] = { 200, 200, 200, 200 };
bool homeState[4] = { false, false, false, false };

// data structure to store sequence of events for learning mode
struct Position {
  int angle;
  int length;
  int height;
  int gripper;
};

// demo sequence of events, note these are absolute coordinates
int demoIndex = 10;
Position demo[10] = { { 114, 6, -130, 1 }, { 114, 6, -50, 1 }, { 80, 100, -130, 0 }, { 80, 100, -50, 0 }, 
                  { 0, 0, 0, 0}, { 80, 100, -130, 1 }, { 80, 100, -50, 1}, { 153, 0, -130, 0 }, 
                  {153, 0, -50, 0}, { 0, 0, 0, 0 } };

// Tower of Hanoi specific variables

int positionRotation[3] = { 80, 114, 153};
int positionLength[3] =   { 100, 6, 0};
int positionHeight[3] =   { -136, -108, -82};

int numDiscs = 3;
int stack[3] = {numDiscs, 0, 0};

void setup() {

  Serial.begin(9600);
  controller.begin(programName);
  interface.begin();
  verticalArm.begin();
  rotateArm.begin();
  delay(2000);
  resetAllStations();
}

String command, numberChar, choice;
int steps;

void loop() {

  interface.getInputs();
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    choice = command.substring(0, 1);
    steps = 0;
    if (command.length() > 1) {
      numberChar = command.substring(1, 3);
      steps = int max(1, numberChar.toInt());
    }

    if (choice == "b") {
      if (!interface.getInput(FORWARD_END)) {
        Serial.println("backward");
      }
      armForward(-steps);
    }
    
    if (choice == "c") { Serial.println("close"); gripperClose(); }

    if (choice == "d") { Serial.println("down"); armDown(-steps); }

    if (choice == "f") { Serial.println("forward"); armForward(steps); }

    if (command == "h") { 
      resetAllStations();
      towerHanoiInit();
      towerHanoi(numDiscs, 0, 2, 1); 
    }

    if (command == "i") { resetAllStations(); }

    if (choice == "l") {
      if (!interface.getInput(ROTATE_END)) {
        Serial.println("left");
        rotate(-steps);
      }
    }

    if (choice == "m") { executeDemo(); }

    if (choice == "o") { Serial.println("open"); gripperOpen(); }

    if (choice == "r") { Serial.println("right"); rotate(steps); }


    if (choice == "s") { Serial.println("stop"); interface.setAllMotorsSTOP(); }

    if (choice == "u") {
      if (!interface.getInput(UP_DOWN_END)) {
        Serial.println("up");
        armDown(steps);
      }
    }
  }

  interface.ftUpdateDisplay();
}

bool armDown(int steps) {
  verticalArm.setSteps(steps);
  Serial.println(verticalArm.getPosition());
}

bool armForward(int steps) {
  horizontalArm.setSteps(steps);
  Serial.println(horizontalArm.getPosition());
}

bool rotate(int steps) {
  rotateArm.setSteps(steps);
  Serial.println(rotateArm.getPosition());
}

bool resetRotation() {
  homeState[ROTATE] = true;
  return (rotateArm.findHome(ROTATE_END, CCW));
};

bool resetArmUpDown() {
  homeState[UP_DOWN] = true;
  return (verticalArm.findHome(UP_DOWN_END, CW));
};

bool resetArmForward() {
  homeState[FORWARD] = true;
  return (horizontalArm.findHome(FORWARD_END, CCW));
};

//
// reset all stations to avoid interference or clashes
bool resetAllStations() {

  resetRotation();
  resetArmUpDown();
  resetArmForward();
  gripperOpen();
  // delay(400);
  // gripperClose();
  return (true);
}

bool gripperOpen() {
  homeState[GRIPPER] = true;
  return (gripper.findHome(GRIPPER_END, CW));
}

bool gripperClose() {
  bool go_on = true;
  int startPos, newPos;
  FTtimer gripperTimer(3000);

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
//------------------------------------------------------------------------
// Recursive Tower of Hanoi routine
void towerHanoi (int n, int source, int destination, int store) {

  Serial.print("Move by : ");
  Serial.print(n);
  Serial.print(" from  ");
  Serial.print(source);
  Serial.print(" to  ");
  Serial.println(destination);

  if (n==1) { 
    moveDisc(source, destination);
    return;
  }

  // move n-1 disc from source to store
  towerHanoi(n-1, source, store, destination);
  moveDisc(source, destination);
  towerHanoi(n-1, store, destination, source);
}

void move (int encoder, int pin, int target) {
  Serial.print("Move by : ");
  Serial.print(encoder);
  Serial.print(" to ");
  Serial.println(target);
  if (encoder == ROTATE) {
    rotateArm.moveToPosition(target);
  } else if (encoder == FORWARD) { 
    horizontalArm.moveToPosition(target);
  } else if (encoder == UP_DOWN) { 
    verticalArm.moveToPosition(target);
  }
}

//------------------------------------------------------------------------
// Move disk from source to destination
void moveDisc(int source, int destination) {

  move(ROTATE, ROTATE_PIN, positionRotation[source]);
  move(FORWARD, FORWARD_PIN, positionLength[source]); 
  move(UP_DOWN, UP_DOWN_PIN, positionHeight[stack[source]-1]); 
  gripperClose();
  move(UP_DOWN, UP_DOWN_PIN, -50);
  stack[source]--;

  move(ROTATE, ROTATE_PIN, positionRotation[destination]);
  move(FORWARD, FORWARD_PIN, positionLength[destination]); 
  move(UP_DOWN, UP_DOWN_PIN, positionHeight[stack[destination]]); 
  gripperOpen();
  move(UP_DOWN, UP_DOWN_PIN, -50);
  stack[destination]++;
}

void towerHanoiInit() {
   stack[0] = numDiscs;
   stack[1] = 0;
   stack[2] = 0;
}

void executeDemo() {
int r, l, h, g;

  resetAllStations();

  for (int i=0; i<demoIndex; i++) {
    r = demo[i].angle;
    l = demo[i].length;
    h = demo[i].height;
    g = demo[i].gripper;
    Serial.print("Move to point: ");
    Serial.print(r);
    Serial.print(",");
    Serial.print(l);
    Serial.print(",");
    Serial.println(h);

    rotateArm.moveToPosition(r);
    horizontalArm.moveToPosition(l);
    verticalArm.moveToPosition(h);

    if (g==1) {
       gripperClose();
    } else {
      gripperOpen();
    }
  }
}
