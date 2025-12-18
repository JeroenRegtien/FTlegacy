/* 
   Prototype program to test the univeral parallel interface (30520) and its predecessors
   30566 (1984,Centronics), 39319/66843 (1985,CVK), 30562 (Commodore), 30563 (Apple)
   
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
  2 5K potmeters connected directly to A2 and A3 of the Arduino
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
const char* programName = "Par iFace Test";

// create interface objects. first argument = {PAR, PAREX, SER, SEREX, ROBO},
// second is interface number for the type.
FTlegacy interface(PAR, 1);

// second is the display {D1604, D1602, D2004}
FTcontroller controller(D2004);

void setup() {
  Serial.begin(9600);
  Serial.println(programName);

  // prepare for serial port to be used in library
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

