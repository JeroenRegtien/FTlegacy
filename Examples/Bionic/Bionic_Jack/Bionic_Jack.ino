/*******************************************************************************************
 * fischertechnik Bionic Robots Model JACK
 * further information: https://docs.fischertechnikclub.nl/digital/62964.pdf
 *                    : https://docs.fischertechnikclub.nl/digital/62965UK.pdf
 *
 * Using FTlegacy library
 * 
 * Copyright (c) 2023-2025 Jeroen Regtien
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
  to test the basic robot movements. 

  'f' : forward.     f5 means five steps forward. 
  'b' : backward
  'l' : rotate left
  'r' : rotate right
*/

#include <FTlegacy.h>

// define constants for the interface motors and input switches
#define RIGHT ft_M1         // motor right side looking from above towards the head
#define RIGHT_FRONT ft_E1   // pin numbers deviate from manual
#define RIGHT_MIDDLE ft_E3  // pin numbers deviate from manual
#define RIGHT_BUMPER ft_E5  // pin numbers deviate from manual

#define LEFT ft_M2          // motor left side looking from above towards the head
#define LEFT_FRONT ft_E2    // pin numbers deviate from manual
#define LEFT_MIDDLE ft_E4   // pin numbers deviate from manual
#define LEFT_BUMPER ft_E6   // pin numbers deviate from manual

const char* programName = "Bionic Jack";
FTlegacy interface(SER, 1);
FTcontroller controller(NONE);

//-------------------------------------------------------------
// Setup
void setup() {
  Serial.begin(9600);
  controller.begin(programName);
  interface.begin();
  delay(1000);
}

String command, numberChar, choice;
int steps;

//-------------------------------------------------------------
// Main loop
void loop() {

  interface.getInputs();
  interface.printInputBuffer();

  // read strings from the serial monitor
  // a single letter may be followed by a number <99
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    choice = command.substring(0, 1);
    if (command.length() > 1) {
      numberChar = command.substring(1, 3);
      steps = int max(1, numberChar.toInt());
    }

    if (choice == "l") {      // turn left
      Serial.print("Turn ");
      Serial.print(steps);
      Serial.println(" steps left");
      left(steps);
    }

    if (choice == "r") {     // turn right
      Serial.print("Turn ");
      Serial.print(steps);
      Serial.println(" steps right");
      right(steps);
    }

    if (choice == "f") {     // forward
      Serial.print("Move ");
      Serial.print(steps);
      Serial.println(" steps forward");
      move(steps);
    }

    if (choice == "b") {     // backward
      Serial.print("Move ");
      Serial.print(steps);
      Serial.println(" steps backward");
      move(-steps);
    }
  }
}

//-------------------------------------------------------------
// Function to handle forward and backward movements.
// For backward movements provide negative number of steps
void move(int steps) {

  bool goLeft, goRight, triggerLeft, triggerRight;

  for (int i = 0; i < abs(steps); i++) {
    bool goLeft = true;
    bool goRight = true;
    bool triggerLeft = false;
    bool triggerRight = false;
    if (steps > 0) {
      interface.setMotorCCW(LEFT);
      interface.setMotorCCW(RIGHT);
    } else {
      interface.setMotorCW(LEFT);
      interface.setMotorCW(RIGHT);
    }
    // keep both legs legs moving until a cycle is complete
    while (goLeft || goRight) {
      interface.getInputs();
      if (goLeft) {
        goLeft = cycle(LEFT, LEFT_FRONT, triggerLeft);
      }
      if (goRight) {
        goRight = cycle(RIGHT, RIGHT_MIDDLE, triggerRight);
      }
    }
  }
}

//-------------------------------------------------------------
// complete a cycle on one side of the robot.
// stop when the switch 'pin' is just released
bool cycle(int motor, int pin, bool& trigger) {
  bool go = true;
  if (interface.getInput(pin)) {
    trigger = true;
  }
  if (trigger && !interface.getInput(pin)) {
    interface.setMotorSTOP(motor);
    trigger = false;
    go = false;
  }
  return (go);   // return false when position reached
}

//-------------------------------------------------------------
// Make a number of left turns
// Note different inputswitches as compared to a right turn
void left(int steps) {

  bool goLeft, goRight, triggerLeft, triggerRight;

  for (int i = 0; i < abs(steps); i++) {
    bool goLeft = true;
    bool goRight = true;
    bool triggerLeft = false;
    bool triggerRight = false;
    interface.setMotorCW(LEFT);
    interface.setMotorCCW(RIGHT);

    // keep both legs legs moving until a cycle is complete
    while (goLeft || goRight) {
      interface.getInputs();
      if (goLeft) {
        goLeft = cycle(LEFT, LEFT_FRONT, triggerLeft);
      }
      if (goRight) {
        goRight = cycle(RIGHT, RIGHT_MIDDLE, triggerRight);
      }
    }
  }
}

//-------------------------------------------------------------
// Make a number of right turns
// Note different inputswitches as compared to a left turn
void right(int steps) {

  bool goLeft, goRight, triggerLeft, triggerRight;

  // loop over the number of steps
  for (int i = 0; i < abs(steps); i++) {
    bool goLeft = true;
    bool goRight = true;
    bool triggerLeft = false;
    bool triggerRight = false;
    interface.setMotorCCW(LEFT);
    interface.setMotorCW(RIGHT);

    // keep both legs legs moving until a cycle is complete
    while (goLeft || goRight) {
      interface.getInputs();
      if (goLeft) {
        goLeft = cycle(LEFT, LEFT_MIDDLE, triggerLeft);
      }
      if (goRight) {
        goRight = cycle(RIGHT, RIGHT_FRONT, triggerRight);
      }
    }
  }
}