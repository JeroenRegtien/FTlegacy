/* 
   Prototype program to test the serial (intelligent) interface (30402)
   
   Assumes a standard test setup with 8 switches, 4 motors, 2 potentiometers and 8 lamps
  
  Interface 1:
     M1: motor
     M2: motor
     M3: motor
     M4: motor
     E1-8: switch 1-8
     Ax: potentiometer X
     Ay: potentiometer Y

  This program assumes a test bed consisting of:
  8 switches connected to E1-E8
  2 5K potmeters connected to Ex and Ey
  4 standard motors connected to M1-M4
  8 lamps connected parallel to the motor inputs with the other side to GND on the interface

  The software used is the FTlegacy library.

  Jeroen Regtien 2023-2025
*/

// include the FT legacy controller classes
#include <FTlegacy.h>

// define and initialise status parameters
bool first = true;

// define Program name
const char* programName = "Ser iFace Test";

// Create interface objects. First argument = {PAR, PAREX, SER, SEREX, ROBO},
// Second is interface number for the type.
FTlegacy interface(SER, 1);

// argument is the display {D1604, D1602, D2004}
FTcontroller controller(D2004);

void setup() {
  Serial.begin(9600);
  Serial.println(programName);
  controller.addInterface(interface);
  controller.begin(programName);
  interface.begin();
}

void loop() {

  // retrieve input from interfaces if any
  interface.getInputs();
  interface.getAnalogInputs();
  interface.printInputBuffer();

  if (first) {
    interface.zeroInput();
    interface.setAllMotorsSTOP();
    first = false;
  }

  if (interface.getInput(ft_E1)) {
     interface.setMotorCW(ft_M1);
  }
  else if (interface.getInput(ft_E2)) {
     interface.setMotorSTOP(ft_M1);
  }

  if (interface.getInput(ft_E3)) {
    interface.setOutputON(ft_O3);
    interface.setOutputOFF(ft_O4);
  }
  else if (interface.getInput(ft_E4)) {
    interface.setOutputON(ft_O4);
    interface.setOutputOFF(ft_O3);
  }

  if (interface.getInput(ft_E5)) {
    interface.setMotorCW(ft_M3);
  }
  else if (interface.getInput(ft_E6)) {
    interface.setMotorSTOP(ft_M3);
  }

  // press two switches at the same time
  if (interface.getInput2(ft_E7, ft_E8)) {
    interface.setMotorCCW(ft_M4);
  } 
  else if (interface.getInput(ft_E7)) {
    interface.setMotorCW(ft_M4);
  }
  else if (interface.getInput(ft_E8)) {
    interface.setAllMotorsSTOP();
  }

  if (interface.getAnalogX()>100) {
     interface.setOutputON(ft_O1);
  } else {
     interface.setOutputOFF(ft_O1);
  }

  if (interface.getAnalogY()>100) {
     interface.setOutputON(ft_O2);
  } else {
     interface.setOutputOFF(ft_O2);
  }

  interface.ftUpdateDisplay();
};

