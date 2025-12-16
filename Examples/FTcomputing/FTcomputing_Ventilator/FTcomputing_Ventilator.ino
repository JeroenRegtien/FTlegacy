/*******************************************************************************************
 * fischertechnik CVK Computing box with parallel interface. 
 * Using FTlegacy library
 * 
 * Model :CVK-7 Ventilator
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

const char* programName = "CVK7 Ventilator";
FTlegacy interface(PAR, 1);
FTcontroller controller(D2004);

FTtimer pauze(2000);

// define constants for inputs and outputs
int HEAT = ft_M1;
int ROTATE = ft_M2;

int readingNTC;     // NTC reading
int readingRoom;    // room NTC reading
bool bWash = true;  // status parameter for washing operation
bool first = true;  // first pass through loop

void setup() {

  Serial.begin(9600);
  controller.begin(programName);
  interface.begin();

  startLamp();
}

void loop() {

  interface.getInputs();  // always need getInputs, otherwise timeout
  interface.getAnalogInputs();
  interface.printInputBuffer();

  if (first) {
    // determine roomtemperature
    readingRoom = interface.getAnalogX();
    first = false;
  }

  readingNTC = interface.getAnalogX();   // retrieve NTC reading;
  // startLamp();

  // start heating
  if (readingNTC <= readingRoom-15) {
    startVentilator();
  } else if (readingNTC >= (readingRoom)) {
    stopVentilator();
  }
  delay(50);
  interface.ftUpdateDisplay();
}

void startLamp() {
  interface.setMotorCCW(HEAT);
}

void startVentilator() {
  interface.setMotorCW(ROTATE);
}

void stopVentilator() {
  interface.setMotorSTOP(ROTATE);
}