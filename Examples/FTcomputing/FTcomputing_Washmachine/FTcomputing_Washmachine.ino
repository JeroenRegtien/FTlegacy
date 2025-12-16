/*******************************************************************************************
 * fischertechnik Computing box 30554 with parallel interface. 
 * Using FTlegacy library
 * 
 * Model: CVK8 Washing Machine
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

const char* programName = "CVK8 Wash Machine";
FTlegacy interface(PAR, 1);
FTcontroller controller(D2004);

FTtimer pauze(2000);

// define constants for inputs and outputs
int HEAT = ft_M3;
int ROTATE = ft_M4;
int DOOR = ft_E7;
int TURN = ft_E8;

int readingNTC;     // NTC reading
int readingRoom;    // room NTC reading
bool bWash = true;  // status parameter for washing operation
bool first = true;  // first pass through loop

void setup() {

  Serial.begin(9600);
  controller.begin(programName);
  interface.begin();
}

void loop() {

  interface.getInputs();  // always need getInputs, otherwise timeout
  interface.getAnalogInputs();
  interface.printInputBuffer();

  if (first) {
    // determine roomtemperature
    readingRoom = interface.getAnalogY();
    first = false;
  }
  readingNTC = interface.getAnalogY();  // retrieve NTC reading;

  if (doorClosed() && bWash) {
    if (readingNTC >= readingRoom) {
      startHeat();
    } else if (readingNTC < (readingRoom-15)) {
      stopHeat();
      for (int i = 1; i <= 3; i++) {
        bWash = startWashing(CCW);
        delay(200);
        bWash = startWashing(CW);
        delay(200);
      }
    }
  } else {
    interface.setAllMotorsSTOP();
    if (!doorClosed()) {
      bWash = true;
      first = true;
    }
  }
  interface.ftUpdateDisplay();
  delay(50);
}

// return true if the door is closed
bool doorClosed() {
  return (interface.getInput(DOOR));
}

// switch on the heating element
void startHeat() {
  interface.setMotorCCW(HEAT);
}

// switch off the heating element
void stopHeat() {
  interface.setMotorSTOP(HEAT);
}

// the washing routine
bool startWashing(motorDirection dir) {
  bool go_on = true;
  while (go_on) {
    go_on = interface.setMotorUntilCount(ROTATE, TURN, ON, dir, 500);
  }
  return (go_on);
}
