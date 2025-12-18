/*******************************************************************************************
 * fischertechnik CVK Computing box with parallel interface. 
 * Using FTlegacy library
 * 
 * Model: Lift
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

#include <FTlegacy.h>

const char* programName = "Computing Lift";
FTlegacy interface(PAR, 1);
FTcontroller controller(D2004);

FTtimer pauze(2000);

// define constants for inputs and outputs
const int MOTOR = ft_M4;
const int GREEN = ft_M1;
const int RED = ft_M3;

const int L1CALL = ft_E8;
const int L1THERE = ft_E5;
const int L2CALL = ft_E7;
const int L2THERE = ft_E4;
const int L3CALL = ft_E6;
const int L3THERE = ft_E3;

int floors[3] = { L1THERE, L2THERE, L3THERE };

int currentFloor;
bool first = true;

void setup() {

  Serial.begin(9600);
  controller.begin(programName);
  interface.begin();
}

void loop() {

  int targetFloor = 1;

  interface.getInputs();  // always need getInputs, otherwise timeout
  interface.printInputBuffer();

  if (first) {
    currentFloor = determineFloor();
    if (currentFloor != 1) {  // move to floor 1 (index 0)
      interface.setMotorCCW(RED);
      interface.setMotorSTOP(GREEN); 
      interface.setMotorUntil(MOTOR, floors[0], ON, CCW);
      interface.setMotorSTOP(RED);
      interface.setMotorCCW(GREEN);
    }
    currentFloor = 1;
    first = false;
  }

  targetFloor = waitForInput();
  if (targetFloor > 0 && targetFloor <= 3) {
    moveToFloor(targetFloor);
  }

  interface.ftUpdateDisplay();
  delay(50);
}

//
// Move to the designated floor only if on a different floor
void moveToFloor(int targetFloor) {

  int delta;

  delta = targetFloor - currentFloor;

  if (delta != 0) {
    /*   
      C T D
      1 2 1
      1 3 2
      2 1 -1
      2 3 1
      3 1 -2
      3 2 -1
  */
    interface.setMotorCCW(RED);
    interface.setMotorSTOP(GREEN);

    if (currentFloor == 1) {
      interface.setMotorUntil(MOTOR, floors[targetFloor - 1], ON, CW);
    } else if (currentFloor == 3) {
      interface.setMotorUntil(MOTOR, floors[targetFloor - 1], ON, CCW);
    } else if (currentFloor == 2) {
      if (delta == 1) {
        interface.setMotorUntil(MOTOR, floors[targetFloor - 1], ON, CW);
      } else if (delta == -1) {
        interface.setMotorUntil(MOTOR, floors[targetFloor - 1], ON, CCW);
      }
    }
    currentFloor = targetFloor;

    interface.setMotorSTOP(RED);
    interface.setMotorCCW(GREEN);
  }
}

//
// find out on which floor the lift is. Return zero if not on one of the three floors
int determineFloor() {

  int floor = 0;
  for (int i = 0; i < 3; i++) {
    if (interface.getInput(floors[i])) {
      floor = i + 1;
    }
  }
  return (floor);
}

//
// wait until the lift is called and return the floor 
int waitForInput() {

  int floor = 0;

  if (interface.getInput(L1CALL)) {
    floor = 1;
  } else if (interface.getInput(L2CALL)) {
    floor = 2;
  } else if (interface.getInput(L3CALL)) {
    floor = 3;
  }
  return (floor);
}