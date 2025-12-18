/*******************************************************************************************
 * fischertechnik Computing box 30572 with parallel interface. 
 *
 * Using FTlegacy library
 * 
 * Model 30572 (Trainingsroboter) test program with basic encoder functionality
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
   Program to test Trainingsroboter model 39461 with one parallel interface

   If used with a different interface note that the max voltage on the 
   optocouplers that measure motor movement is 5V maximum

   Note the connection of the end point switches  has been changes from
   normally closed to normally open.

*/

// include the FT legacy controller classes
#include <FTlegacy.h>
#include <String.h>

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

  'u' : upper arm up
  'd' : upper arm down
  'f' : lower arm forward
  'b' : lower arm backward
  'l' : rotate left
  'r' : rotate right
  'c' : close gripper
  'o' : open gripper
  'i' : reset all stations
  'm' : execute demo sequence

*/

#define ROTATE ft_M1        //    M1 : Rotation
#define ROTATE_END  ft_E1   //    rotation end pin
#define ROTATE_PIN  ft_E2   //    rotation photocell
#define SHOULDER ft_M2      //    M2 : Upper arm
#define SHOULDER_END ft_E3  //    shoulder end pin
#define SHOULDER_PIN ft_E4  //    shoulder photocell
#define ELBOW ft_M3         //    M3 : Lower arm 
#define ELBOW_END ft_E5     //    elbow end pin
#define ELBOW_PIN ft_E6     //    elbow photocell
#define CLAW ft_M4          //    M4 : Claw
#define CLAW_END ft_E7

// define Program name
const char* programName = "39461-tst";

// create interface objects. first argument = {PAR, PAREX, SER, SEREX, ROBO},
// second is interface number for the type.
FTlegacy interface(PAR, 1);

// create board object. first argument = {UNO, MEGA, DUE),
// second is the display {D1306L, D1306S, D1604, D1602, D2004}
FTcontroller controller(D2004);

FTencoderMotor upperArm(&interface, SHOULDER, E_STD, SHOULDER_PIN);
FTencoderMotor lowerArm(&interface, ELBOW, E_STD, ELBOW_PIN);
FTencoderMotor rotateArm(&interface, ROTATE, E_STD, ROTATE_PIN);

int position[4] = { 0, 0, 0, 0 };
int minPosition[4] = { 0, -900, 0, 0 };
int maxPosition[4] = { 0, 900, 1100, 1100 };
bool homeState[4] = { false, false, false, false };

void setup() {

  Serial.begin(9600);  // set up Serial library at 9600 bps
  Serial.println("programName");

  // prepare for serial port to be used in library
  controller.addInterface(interface);
  controller.begin(programName);

  interface.begin();
  interface.setAllMotorsSTOP();

  // initialise the robot arm
  resetAllStations();
}

// variables to read the serial monitor
String command, numberChar, choice;
int steps;

void loop() {

  int targetP2 = 0;
  int targetP3 = 0;

  interface.getInputs();
  // interface.printInputBuffer();

  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    choice = command.substring(0, 1);
    steps = 0;
    if (command.length() > 1) {
      numberChar = command.substring(1, 4);
      steps = int max(1, numberChar.toInt());
    }

    if (choice == "u") { Serial.println("up"); armLower(-steps); }
    else if (choice == "d") { Serial.println("down"); armLower(steps); }
    else if (choice == "l") { Serial.println("left"); rotate(-steps); } 
    else if (choice == "r") { Serial.println("right"); rotate(steps); } 
    else if (choice == "f") { Serial.println("forward"); armUpper(steps); } 
    else if (choice == "b") { Serial.println("backward"); armUpper(-steps); } 
    else if (choice == "c") { Serial.println("close"); gripperClose(); } 
    else if (choice == "o") { Serial.println("open"); gripperOpen(); } 
    else if (choice == "i") { resetAllStations(); } 
    else if (choice == "m") { executeDemoSequence(); }
   
    // set command / coice to zero, serial read goes bezerk...
    choice = "0";
    command = "0";
  }
  interface.ftUpdateDisplay();
}

//------------------------------------------------------------------------
// Generic function to rotate the arm
bool rotate(int steps) {
  rotateArm.setSteps(steps);
  Serial.println(rotateArm.getPosition());
}

//------------------------------------------------------------------------
// Generic function to move the lower arm up or down
bool armLower(int steps) {
  lowerArm.setSteps(steps);
}

//------------------------------------------------------------------------
// Generic function to move the upper arm backward or forward
bool armUpper(int steps) {
  upperArm.setSteps(steps);
}

//------------------------------------------------------------------------
// execute the demo sequence
void executeDemoSequence(){
  resetAllStations();
  gotoPosition(660, 1030);
  gripperClose();
  gotoPosition(200, 200);
  rotate(-500);
  gotoPosition(660, 1030);
  gripperOpen();
  armUpper(-300);
  gotoPosition(200, 200);
  resetAllStations();
}

//------------------------------------------------------------------------
// go to a combine position for upper and lower arm
bool gotoPosition(int targetP2, int targetP3) {
  if (checkLimit(targetP2, targetP3)) {
       lowerArm.moveToPosition(targetP3);
       upperArm.moveToPosition(targetP2);
    return (true);
  } else {
    return (false);
  }
}

//------------------------------------------------------------------------
// checks the validty of the proposed coordinates
bool checkLimit(int targetP2, int targetP3) {
  if ((targetP2 > minP2(targetP2, targetP3)) && (targetP2 < maxP2(targetP2, targetP3)) && (targetP3 > minP3(targetP2, targetP3)) && (targetP3 < maxP3(targetP2, targetP3))) {
    return true;
  } else {
    return false;
  }
}

//------------------------------------------------------------------------
// checks the validity of the proposed coordinates subroutine
int maxP2(int targetP2, int targetP3) {
  int P2;

  P2 = min(1200, max(targetP2, 1200));
  if (targetP3 >= 0) {
    P2 = min(1200, targetP3+400);
  }
  return (P2);
}

//------------------------------------------------------------------------
// checks the validity of the proposed coordinates subroutine
int minP2(int targetP2, int targetP3) {
  int P2;

  P2 = min(0, max(targetP2, 0));
  if (targetP3>940) {
    P2 = 2 * (targetP3-940);
  }
  return (P2);
}

//------------------------------------------------------------------------
// checks the validity of the proposed coordinates subroutine
int maxP3(int targetP2, int targetP3) {
  int P3;

  P3 = max(1140, min(targetP3, 1140));
  if (targetP2<400) {
    P3 = 940 + targetP2/2;
  }
  return (P3);
}

int minP3(int targetP2, int targetP3) {
  int P3;

  P3 = min(0, max(targetP3, 0));
  if (targetP2>400) {
    P3 = targetP2-400;
  }
  return (P3);
}

//------------------------------------------------------------------------
// checks the validity of the proposed coordinates subroutine
bool resetRotation() {
  Serial.println("reset rotation");
  return (rotateArm.findHome(ROTATE_END, CW));
}

//------------------------------------------------------------------------
// checks the validity of the proposed coordinates subroutine
bool resetArmUpper() {
  position[SHOULDER] = 0;
  return (upperArm.findHome(SHOULDER_END, CCW));
}

bool resetArmLower() {
  position[ELBOW] = 0;
  return (lowerArm.findHome(ELBOW_END, CCW));
}

//------------------------------------------------------------------------
// reset all stations to avoid interference or clashes
bool resetAllStations() {
  resetRotation();
  resetArmUpper();
  resetArmLower();
  gripperInitial();
  gripperOpen();
  return (true);
}

//------------------------------------------------------------------------
// open the gripper
bool gripperOpen() {
  bool go_on = true;
  FTtimer gripperTimer(200);
  if (!homeState[CLAW]) {
    while (!gripperTimer.ready()) {
      interface.setMotorCW(CLAW);
    }
    go_on = interface.setMotorUntil(CLAW, CLAW_END, ON, CW);
    go_on = interface.setMotorUntil(CLAW, CLAW_END, OFF, CW);
    homeState[CLAW] = true;
  }
  return (true);
}

//------------------------------------------------------------------------
// close the gripper
bool gripperClose() {
  bool go_on = true;

  FTtimer gripperTimer(2000);
  if (homeState[CLAW]) {
    while (!gripperTimer.ready()) {
      interface.setMotorCCW(CLAW);
      interface.getInputs();
    }
    interface.setMotorSTOP(CLAW);
    homeState[CLAW] = false;
  }
  return (true);
}

//------------------------------------------------------------------------
// set the initial gripper position
bool gripperInitial() {
  bool go_on = true;

  FTtimer gripperTimer(2000);
  while (!gripperTimer.ready()) {
    interface.setMotorCCW(CLAW);
    interface.getInputs();
  }
  interface.setMotorSTOP(CLAW);
  homeState[CLAW] = false;
  return (true);
}