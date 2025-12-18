// include the legacy FT classes
#include <FTlegacy.h>

// define Program name
const char* programName = "Antennen Rotor";

// create interface objects. first argument = {PAR, SER, ROBO, DID, ADA}, 
// second is interface number for the type.
FTlegacy sFace1 (SER, 1);
FTlegacy sFace2 (SER, 2);

// create board object. first argument = {UNO, MEGA, DUE), 
// second is the display {D1306L, D1306S, D1604, D1602, D2004}
FTcontroller controller(D2004);

void setup() {
  Serial.begin(9600);
  Serial.print(programName);

  // controller.initDisplay();

  // setup serial communicates for main program
  if (board == 1) {
    Serial.println(" Arduino Uno ");
  } else if (board == 2) {
    Serial.println(" Arduino Mega XX ");
  } else if (board == 1) {
    Serial.println(" unknown board error ");
  }

  // prepare for serial port to be used in library
  controller.addInterface(sFace1);
  controller.addInterface(sFace2);
  controller.begin(programName);

  int i = controller.getNumberInterfaces();
  int j = controller.getBoard();
  Serial.print("number of interfaces: ");
  Serial.println(i);
  Serial.print("board type: ");
  Serial.println(j);

  sFace1.setAllMotorsSTOP();
  sFace2.setAllMotorsSTOP();
}

int iAX;
int iAY;

void loop() {

  int iDiff;

  //Serial.println("In loop ");
  //delay(5000);

  // retrieve input from interfaces if any
  sFace1.getInputs();
  //  sFace1.printInputBuffer();

  sFace1.getAnalogInputs();
  iAX = sFace1.getAnalogX();
  iAY = sFace1.getAnalogY();
   
  iDiff = abs(iAX - iAY);

  if (iDiff > 10) {
    if (iAY > iAX) {
      sFace1.setMotorCW(ft_M1);
//      Serial.println(" CW ");
    } else if (iAX > iAY) {
      sFace1.setMotorCCW(ft_M1);
//      Serial.println(" CCW ");
    }
  } else {
    delay(1);  //Bremsphase
    sFace1.setMotorSTOP(ft_M1);
//    Serial.println(" STOP ");
  }
  sFace1.ftUpdateDisplay ();

  // retrieve input from interfaces if any
  sFace2.getInputs();
  //  sFace1.printInputBuffer();
  sFace2.getAnalogInputs();
  iAX = sFace2.getAnalogX();
  iAY = sFace2.getAnalogY();
 
  iDiff = abs(iAX - iAY);

  if (iDiff > 10) {
    if (iAY > iAX) {
      sFace2.setMotorCW(ft_M1);
  //    Serial.println(" CW ");
    } else if (iAX > iAY) {
      sFace2.setMotorCCW(ft_M1);
  //    Serial.println(" CCW ");
    }
  } else {
    sFace2.setMotorSTOP(ft_M1);
  //  Serial.println(" STOP ");
  }
  sFace2.ftUpdateDisplay ();

}
