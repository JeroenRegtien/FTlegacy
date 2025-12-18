/*******************************************************************************************
 * fischertechnik Computing box 30554 with parallel interface. 
 * Using FTlegacy library
 * 
 * Model: Automated Drill
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

const char* programName = "FTcomputing Auto Drill";
FTlegacy interface (PAR,1);
FTcontroller controller (NONE);

int DRILL = ft_M2;
int ROTATE = ft_M1;
int UP = ft_E1;
int DOWN = ft_E2;
int HALT = ft_E3;

FTtimer drillTimer(5000);

void setup() {

  Serial.begin(9600);
  controller.begin(programName);
  interface.begin();
  
  interface.setAllMotorsSTOP();
}

bool start = false;
bool motorUp = false;
bool motorDown = false;

void loop() {

  interface.getInputs();
  interface.printInputBuffer();

  if (!motorUp) {
     motorUp = !interface.setMotorUntil(DRILL, UP, true, CCW);
     motorDown = false;
  }      

  if (motorUp && interface.getInput(HALT)) {
    start = true;
    interface.setMotorSTOP(ROTATE);
  } else {
    interface.setMotorCCW(ROTATE);
  }
  
  if (start) {

    if (!motorDown) {
      motorDown = !interface.setMotorUntil(DRILL, DOWN, true, CW);
      drillTimer.reset();
    } else {
      if (drillTimer.ready()) {
        start = false;
        motorUp = false;
      }
    }
  }
}
