/*******************************************************************************************
 * fischertechnik Computing box 30554 with parallel interface. 
 * Using FTlegacy library
 * 
 * Plotter: plot a christmas wish to demonstrate functionality
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
//const char* xmas = "Frohe Weihnachten";
//const char* newYear = "Ein gutes neues Jahr 2026";
//const char* xmas = "Fijne kerstdagen";
//const char* newYear = "Een heel gelukkig 2026";
const char* xmas = "Merry Christmas";
const char* newYear = "A very happy 2026";
int scaleX = 2;
int scaleY = 3;

// create the instances/objects
FTlegacy interface(PAR, 1);
FTcontroller controller(D2004);
FTstepperXY plotter(interface, COILX1, COILX2, COILY1, COILY2, PEN);

// plot coordinates for tree and balls
int xmasTree[20][2] = { { 0, -10 }, { 30, 5 }, { 0, -45 }, { 50, 20 }, { -5, -5 }, { 50, 15 }, { -5, -5 }, 
                        { 50, 15 }, { -5, -5 }, { 50, 15 }, { -50, 15 }, { 5, -5 }, { -50, 15 }, { 5, -5 }, 
                        { -50, 15 }, { 5, -5 }, { -50, 20 }, { 0, -45 }, { -30, 5 }, { 0, -10 } };
int numPoints = 20;
int xmasBalls[10][2] = { { 180, 0 }, { 155, -10 }, { 145, 10 }, { 110, 15 }, { 90, 0 }, 
                         { 110, -15 }, { 60, -25 }, { 50, -15 }, { 60, 15 }, { 50, 25 } };
int numBalls = 10;

void setup() {
  // library initalisation
  Serial.begin(9600);
  controller.begin(programName);
  interface.begin();

  // plotter initialisation
  interface.setAllMotorsSTOP();
  plotter.setArea(0, 0, 600, 500);
  plotter.findOrigin(STOP_X, STOP_Y);

  // draw tree
  drawTree(200, 250, scaleX, scaleY);
  // add greetings
  greetings();
  // shut down motors
  interface.setAllMotorsSTOP();
}

void loop() {
}

void greetings() {
  // text labels
  plotter.plotText(120, 400, 3, 3, xmas);
  plotter.plotText(90, 450, 3, 3, newYear);
  plotter.plotText(300, 250, 2, 0, "JMMR 2025");
}

void drawTree(int posX, int posY, int scaleX, int scaleY) {
  // start position
  plotter.moveToPosition(posX, posY);
  // draw the tree
  plotter.penDown();
  for (int i = 0; i < numPoints; i++) {
    plotter.moveRelative(scaleX * xmasTree[i][0], scaleY * xmasTree[i][1]);
  }
  plotter.penUp();
  // draw the balls
  plotter.moveToPosition(posX, posY);
  for (int i = 0; i < numBalls; i++) {
    plotter.circle(posX + scaleX * xmasBalls[i][0], posY + scaleY * xmasBalls[i][1], 5 * scaleX);
  }
}
