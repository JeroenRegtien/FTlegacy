/*******************************************************************************************
 * fischertechnik Computing box 30554 with parallel interface. 
 * Using FTlegacy library
 * 
 * Plotter: Simple functionality test
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
 *
 *
 * Note : the memory requirements for the ascii character set exceed the Arduino Uno R3
 *        memory limits. Use and Arduino Uno R4 or Arduino Mega instead.
 * 
 **********************************************************************************************/

#include <FTlegacy.h>

#define PEN ft_M4
#define COILX1 ft_M1
#define COILX2 ft_M2
#define COILY1 ft_M1
#define COILY2 ft_M3
#define STOP_X ft_E7
#define STOP_Y ft_E8

const char* programName = "FTplotter step";
const char* test = "TEST";

// create the interface 
FTlegacy interface(PAR, 1);
// create the controller
FTcontroller controller(NONE);
// create the plotter
FTstepperXY plotter(interface, COILX1, COILX2, COILY1, COILY2, PEN);

void setup() {

  int curveX[90];
  int curveY[90];

  Serial.begin(9600);
  controller.begin(programName);
  interface.begin();

  // plotter.setArea(0, 0, 680, 500);
  plotter.findOrigin(STOP_X, STOP_Y);

  plotter.moveToPosition(1000,750);  // check on out of bounds

  plotter.axis(0,100,0,100,100, 500, 120, 420, true, "Time (s)", "Speed (m/s)", "Sinus example", 4, 4, 7);

  plotter.moveToPosition(100, 250);
  for (int i=0; i<90; i++) {
    curveX[i] = int(100. + 400. * 4.* float(i)/360.);
    curveY[i] = int(270. + 100. * sin(float(i)/45. * PI));
  }
  plotter.curve(90, curveX, curveY);
  
  // other examples
  // ==============
  //plotter.plotText(170, 370, 4, 0, test);
  //plotter.plotText(170, 370, 4, 1, test);
  //plotter.plotText(170, 370, 4, 2, test);
  //plotter.plotText(170, 370, 4, 3, test);

  //plotter.ellips(400,200,200,100);
  //plotter.moveToPosition(100, 75);
  //plotter.box(400, 300, 100, 100, 0);
  //plotter.lineRelative(0,-20);

  interface.setAllMotorsSTOP();
}

void loop() {
  delay(4000);
}
