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

void setup() {

  Serial.begin(9600);
  controller.begin(programName);
  interface.begin();
  verticalArm.begin();
  rotateArm.begin();
  delay(2000);
}

String command, numberChar, choice;
int number;

void loop() {

  interface.getInputs();
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    choice = command.substring(0, 1);

    if (choice == "s") {
      Serial.println("stop");
      interface.setAllMotorsSTOP();
    }

    if (choice == "d") {
      Serial.println("down");
      armDown(-10);
    }

    if (choice == "u") {
      if (!interface.getInput(UP_DOWN_END)) {
        Serial.println("up");
        armDown(10);
      }
    }

    if (choice == "l") {
      if (!interface.getInput(ROTATE_END)) {
        Serial.println("left");
        rotate(-30);
      }
    }

    if (choice == "r") {
      Serial.println("right");
      rotate(30);
    }

    if (choice == "f") {
      Serial.println("forward");
      armForward(20);
    }

    if (choice == "b") {
      if (!interface.getInput(FORWARD_END)) {
        Serial.println("backward");
      }
      armForward(-20);
    }

    if (choice == "m") {
      Serial.println("move 50");
      horizontalArm.moveToPosition(50);
    }
    if (choice == "c") {
      Serial.println("close");
      gripperClose();
    }

    if (command == "o") {
      Serial.println("open");
      gripperOpen();
    }

    if (command == "i") {
      resetAllStations();
    }
  }
  interface.ftUpdateDisplay();
}

bool armDown(int steps) {
  verticalArm.setSteps(steps);
}

bool armForward(int steps) {
  horizontalArm.setSteps(steps);
}

bool rotate(int steps) {
  rotateArm.setSteps(steps);
}

bool resetRotation() {
  homeState[ROTATE] = true;
  return (rotateArm.findHome(ROTATE_END, CCW));
  //  return (interface.setMotorUntil(ROTATE, ft_E1, ON, CCW));
};

bool resetArmUpDown() {
  homeState[UP_DOWN] = true;
  return (verticalArm.findHome(UP_DOWN_END, CW));
  // return (interface.setMotorUntil(UP_DOWN, ft_E5, ON, CW));
};

bool resetArmForward() {
  homeState[FORWARD] = true;
  return (horizontalArm.findHome(FORWARD_END, CCW));
  // return (interface.setMotorUntil(FORWARD, ft_E3, ON, CCW));
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
