/*******************************************************************************************
 * fischertechnik Computing box 30554 with parallel interface. 
 * Using FTlegacy library
 * 
 * Model: Tower of Hanoi
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

#include <FTlegacy.h>

// define Program name
const char* programName = "FTcomputing Tower Hanoi";

// create interface objects. first argument = {PAR, PAREX, SER, SEREX, ROBO},
// second is interface number for the type.
FTlegacy interface (PAR,1);

// second is the display {D1604, D1602, D2004}
FTcontroller controller (NONE);

// indicate where motors are used for
int ROTATE = ft_M1;
int ARM = ft_M2;
int MAGNET = ft_M3;
int BOTTOM = ft_E1;
int TOP = ft_E2;

int position[3] = {90, 120, 150 };   // angle for the piles
int numDiscs = 5;                    // number of discs for the puzzle

//------------------------------------------------------------------------
// Setup routine
void setup() {

  Serial.begin(9600);
  controller.begin(programName);
  interface.begin();

  towerHanoi(numDiscs, 0, 2, 1);   // array index starts at zero!
}

//------------------------------------------------------------------------
// Main loop
void loop() {

  interface.getInputs();  // alway need getInputs, oterwise timeout
  interface.getAnalogInputs();
  // interface.printInputBuffer();
  if (interface.getInput(ft_E7)) {
     towerHanoi(numDiscs, 0, 2, 1);   // array index starts at zero!
  }
}
//------------------------------------------------------------------------
// Recursive Tower of Hanoi routine
void towerHanoi (int n, int source, int destination, int store) {

  interface.getInputs();  // alway need getInputs, oterwise timeout
  interface.getAnalogInputs();
  // interface.printInputBuffer();

  if (n==1) { 
    moveDisc(source, destination);
    return;
  }

  // move n-1 disc from source to store
  towerHanoi(n-1, source, store, destination);
  moveDisc(source, destination);
  towerHanoi(n-1, store, destination, source);
}

//------------------------------------------------------------------------
// Move disk from source to destination
void moveDisc(int source, int destination) {
   moveArm(position[source]);
   armDOWN();
   magnetON();
   armUP();
   moveArm(position[destination]);
   armDOWN();
   magnetOFF();
   armUP();   
}

//------------------------------------------------------------------------
// Switch magnet ON
void magnetON() {
  interface.setMotorCW(MAGNET);
  Serial.println("Magnet On");
}

//------------------------------------------------------------------------
// Switch magnet off, reverse polarity to avoid sticking
void magnetOFF() {
  interface.setMotorSTOP(MAGNET);
  interface.setMotorCCW(MAGNET);
  interface.setMotorSTOP(MAGNET);
  Serial.println("Magnet Off");
}

//------------------------------------------------------------------------
// Move arm up
void armUP(){
  bool go_on = true;
  
  Serial.println("start arm up");
  while (go_on) {
    interface.getInputs();
    go_on = interface.setMotorUntil(ARM, TOP, true, CW);
  }

  Serial.println("Arm Up");
}

//------------------------------------------------------------------------
// Move arm down
void armDOWN(){
  bool go_on = true;

  Serial.println("start arm down");
  while (go_on) {
    interface.getInputs();
    // interface.printInputBuffer();
    go_on = interface.setMotorUntil(ARM, BOTTOM, true, CCW);
  }
  Serial.println("Arm Down");
}

//------------------------------------------------------------------------
// Move arm to target angle
void moveArm(int targetAngle){
  int angle=0;

  Serial.println("move arm");

  int diff = 2;
  while (abs(diff) > 1) {
    interface.getInputs();
    interface.getAnalogInputs();
    // interface.printInputBuffer();
    angle = interface.getAnalogY();
  
    diff = targetAngle - angle;
  
    // Serial.println(diff);
    if (diff > 1) {
      interface.setMotorCCW(ROTATE); 
    }
    else if (-diff > 1) {
      interface.setMotorCW(ROTATE);
    }
    else { 
      interface.setMotorSTOP(ROTATE);// do nothing
      Serial.println("arm in place");
    }
  }
}
