/*******************************************************************************************
 * fischertechnik Computing box 30554 with parallel interface. 
 * Using FTlegacy library
 * 
 * Model: Traffic lights
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

#define DEMO ft_E1 // E1: execute demo sequence
#define EXEC ft_E2 // E2: execute learn sequence
#define CCW ft_E3  // E3: rotate CCW
#define CW ft_E4   // E4: rotate CW
#define UP ft_E5   // E5: arm UP
#define DOWN ft_E6 // E6: arm down
#define TOGGLE ft_E7 // E7: toggle magnet
#define LEARN ft_E8  // E8: learn position

// M1: EX: arm rotating min (right) = 60, max (left) = 240
// M2: EY: arm for/backward potsetting min (back, CW) = 95, max (for, CCW) = 230
#define ROTATE ft_M1
#define ARM    ft_M2
#define MAGNET ft_M3

// define Program name
const char* programName = "Teach IN";

// create interface objects. first argument is one of {PAR, PAREX, SER, SEREX, ROBO}
// second is interface sequance number for the type (1-4 for PAR & SER if MEGA, 1 for UNO)
FTlegacy interface(PAR, 1);

// create board object. first argument display {D1604, D1602, D2004}
FTcontroller controller(D2004);

// define LCD display
int anaX = -1;
int anaY = -1;
bool first = true;
bool onEx = true;
bool onEy = true;
bool onLCD = false;

// data structure to store sequence of events for learning mode
struct Position {
  int angle;
  int height;
  int magnet;
};

#define MAX_POS 64

Position pos[MAX_POS];
// current position index
int numPos = 0;
int onMagnet = 0;

int exBounds[2] = { 60, 240 };
int eyBounds[2] = { 95, 230 };

//! tolarance for position of ex and ey readings
#define TOLERANCE 3

// demo sequence of events
int demoIndex = 9;
Position demo[9] = { { 200, 210, 1 }, { 200, 240, 1 }, { 127, 152, 0 }, { 127, 240, 0 }, { 156, 94, 1 }, 
                     { 156, 240, 1 }, { 124, 152, 0 }, { 124, 240, 0 }, {156, 240, 0} };

int proceed;

void executeLearnSequence();
void executeDemoSequence();

void setup() {
  Serial.begin(9600);

  controller.addInterface(interface);
  controller.begin(programName);
  interface.begin();

  proceed = interface.connectedAnalog();
  moveArm(240);
  rotateArm(160);
}

void loop() {

  // get the inputs
  interface.getInputs();
  interface.printInputBuffer();
  interface.getAnalogInputs();

  anaX = interface.getAnalogX();
  anaY = interface.getAnalogY();

  if (interface.getInput(LEARN)) {
    // learning mode is on
    pos[numPos++] = { anaX, anaY, onMagnet };
  } else if (interface.getInput(DEMO)) {
    // execute the demo sequence
    executeDemoSequence();
  } else if (interface.getInput(EXEC)) {
    // execute the learn sequence
    executeLearnSequence();
  }

  // reset the learning sequence in a clumsy manner by pressing E7, E8 together
  if (interface.getInput2(TOGGLE,LEARN)) {
    numPos=0;
  }

  if (interface.getInput(CCW)) {

    if (interface.getAnalogX() < exBounds[0]) {
      interface.setMotorSTOP(ROTATE);
    } else {
      interface.setMotorCCW(ROTATE);
    }

  } else if (interface.getInput(CW)) {

    if (interface.getAnalogX() > exBounds[1]) {
      interface.setMotorSTOP(ROTATE);
    } else {
      interface.setMotorCW(ROTATE);
    }
  } else {
    interface.setMotorSTOP(ROTATE);
  }

  if (interface.getInput(DOWN)) {

    if (interface.getAnalogY() < eyBounds[0]) {
      interface.setMotorSTOP(ARM);
    } else {
      interface.setMotorCCW(ARM);
    }

  } else if (interface.getInput(UP)) {

    if (interface.getAnalogY() > eyBounds[1]) {
      interface.setMotorSTOP(ARM);
    } else {
      interface.setMotorCW(ARM);
    }
  } else {
    interface.setMotorSTOP(ARM);
  }

  if (interface.getInput(TOGGLE)) {
    // toggle magnet
    if (!onMagnet) {
      magnetON();
    } else {
      magnetOFF();
    }
    onMagnet = !onMagnet;
  }
  interface.ftUpdateDisplay();
}

void magnetON() {
  interface.setMotorCW(MAGNET);
}

void magnetOFF() {
  interface.setMotorCCW(MAGNET);
  interface.setMotorSTOP(MAGNET);
  interface.setMotorCW(MAGNET);
  interface.setMotorCCW(MAGNET);
  interface.setMotorSTOP(MAGNET);
}

void moveArm(int pos) {
  bool stop = false;

  pos = min(pos, eyBounds[1]);
  pos = max(pos, eyBounds[0]);

  while (!stop) {
    interface.getAnalogInputs();
    if (interface.getAnalogY() > pos + TOLERANCE) {
      interface.setMotorCCW(ARM);
    } else if (interface.getAnalogY() < pos - TOLERANCE) {
      interface.setMotorCW(ARM);
    } else {
      interface.setMotorSTOP(ARM);
      stop = true;
    }
  }
}

void rotateArm(int pos) {
  bool stop = false;

  pos = min(pos, exBounds[1]);
  pos = max(pos, exBounds[0]);

  while (!stop) {
    interface.getAnalogInputs();
    if (interface.getAnalogX() > pos + TOLERANCE) {
      interface.setMotorCCW(ROTATE);
    } else if (interface.getAnalogX() < pos - TOLERANCE) {
      interface.setMotorCW(ROTATE);
    } else {
      interface.setMotorSTOP(ROTATE);
      stop = true;
    }
  }
}

// execute the demo sequence
void executeLearnSequence() {
  moveArm(255);
  for (int i = 0; i < numPos; i++) {
    Serial.print(F("learning sequence "));
    Serial.print(i);
    Serial.print(" H ");
    Serial.print(pos[i].height);
    Serial.print(" A ");
    Serial.print(pos[i].angle);
    Serial.print(" M ");
    Serial.println(pos[i].magnet);

    rotateArm(pos[i].angle);
    moveArm(pos[i].height);
    pos[i].magnet ? magnetON() : magnetOFF();
  }
}

void printLearnSequence() {
  for (int i = 0; i < numPos; i++) {
    Serial.print(F("learn sequence "));
    Serial.print(i);
    Serial.print("  ");
    Serial.print(pos[i].height);
    Serial.print("  ");
    Serial.print(pos[i].angle);
    Serial.print("  ");
    Serial.println(pos[i].magnet);
  }
}

// execute the demo sequence
void executeDemoSequence() {
  moveArm(255);
  for (int i = 0; i < demoIndex; i++) {

    Serial.print(F("demo sequence "));
    Serial.print(i);
    Serial.print(" H ");
    Serial.print(demo[i].height);
    Serial.print(" A ");
    Serial.print(demo[i].angle);
    Serial.print(" M ");
    Serial.println(demo[i].magnet);

    rotateArm(demo[i].angle);
    moveArm(demo[i].height);
    demo[i].magnet ? magnetON() : magnetOFF();
  }
}
