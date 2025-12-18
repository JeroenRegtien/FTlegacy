/* 
   Program to test the WKS-30581 Plan&Simulation 'taktstrasse' using
   3 serial/intelligent interfaces (one with extension).
   
   See also the article on this model in:
   https://www.ftcommunity.de/ftpedia/2025/2025-2/ftpedia-2025-2.pdf
*/

// include the FT legacy controller classes

#include <FTlegacy.h>

/*
  Interface 1:
    E1  : Frees 1 boven (bruin-rood)
    E2  : Frees 1 onder (lila-wit)
    E3  : Frees 1 positie (grijs-bruin)
    E4  : initiator band 2 
    E5  : initiator band 3
*/
#define START ft_E6  //  E6  : Start
#define RESET ft_E7  //  E7  : Reset
#define STOP ft_E8   //  E8  : Stop

#define F1_Z ft_M1    //    M1 : Frees 1 op/neer (
#define F1_W ft_M2    //    M2 : Frees 1 wissel
#define BAND_2 ft_O5  //    O5 : Band 2
#define F1_R ft_O6    //    O6 : Frees 1 rotatie
#define BAND_3 ft_O7  //    O7  : Band 3

/*
  Interface 1 Extensie
    E1(9) : X – laden links	(grijs-wit)
    E2(10) : X – laden rechts (geel)
    E3(11) : geladen (blauw-wit)
    E4(12) : gereed (O) (grijs-wit)
    E5(13) : initiator (groen-wit)
    E6(14) : boor boven (grijs-roze)
    E7(16) : boor onder (zwart-wit)
    E8 : 
*/
#define LAAD ft_M5     //    M1(5) : motor laden (rood, rood)
#define BAND_1 ft_M6   //    M2(6) : motor band 1 (groen, geel)
#define BOOR_Z ft_M7   //    M3(7) : motor boor up/down
#define BOOR_R ft_O15  //    O7(15) : motor boor roteren
#define LAMP_1 ft_O16  //    O8(16) : lamp lichtcel (groen-wit)
/*
  Interface 2:
    E1 : Frees 2 boven (geel-zwart)
    E2 : Frees 2 onder (geel-wit)
    E3 : Frees 2 voor (roze)
    E4 : Frees 2 achter (bruin-blauw)
    E5 : positie plateau recht (groen-wit)
    E6 : positie plateau dwars (grijs-bruin)
    E7 : lichtcel (rood-wit)
    E8: Z-magneet (grijs)
*/
#define F2_Z ft_M1    //    M1 : Frees 2 op/neer (grijs, bruin-groen)
#define F2_Y ft_M2    //    M2 : Frees 2 voor/achter (blauw, blauw)
#define BAND_4 ft_O5  //    O5 : Band plateau (paars, ) & Lamp (groen-wit, groen)
#define F2_R ft_O6    //    O6 : Frees 2 rotatie (roze-paars)
#define ROTATE ft_M4  //    M4 : Rotatie plateau (zwart, rose)
/*
  Interface 3:
    E1 : X – links (geel-paars)	
    E2 : X –rechts (blauw-wit)
    E3 : X – laden  (groen-wit)
    E4 : X – lossen (lila-grijs)
    E5 : Y – achter (wit-zwart)
    E6 : Y – voor (blauw-bruin)
    E7 : Z – boven (grijs-zwart)
    E8 : Z – onder (groen-bruin)

*/
#define MOTOR_X ft_M1  //  M1 : motor X (bruin-wit, bruin-groen)
#define MOTOR_Y ft_M2  //  M2 : motor Y (grijs-wit, geel-wit)
#define MOTOR_Z ft_M3  //  M3 : motor Z (blauw-wit, groen)
#define MAGNET ft_M4   //  M4 : magneet (blauw-rood, geel)

// define and initialise status parameters
bool reset = false;
bool b_left = false;

// define Program name
const char* programName = "TST-30581";

// create interface objects. first argument = {PAR, PAREX, SER, SEREX, ROBO, DID, ADA},
// second is interface number for the type.
FTlegacy i1(SEREX, 1);
FTlegacy i2(SER, 2);
FTlegacy i3(SER, 3);

// create board object. first argument = {UNO, MEGA, DUE),
// second is the display {D1306L, D1306S, D1604, D1602, D2004}
FTcontroller controller(D2004);

void setup() {
  Serial.begin(115200);  // set up Serial library at 9600 bps
  Serial.println("WKS 30581 input test");

  // prepare for serial port to be used in library
  controller.addInterface(i1);
  controller.addInterface(i2);
  controller.addInterface(i3);
  controller.begin(programName);

  i1.begin();
  i2.begin();
  i3.begin();

  i1.setAllMotorsSTOP();
}

void loop() {
  uint8_t i, m;
  int M;
  bool b_start;

  i1.getInputs();
  i2.getInputs();
  i3.getInputs();

  i1.printInputBuffer();
  // i2.printInputBuffer();
  // i1.printInputBuffer();
  controller.ftMessageToDisplay(1, 1, "Wachten", true);

  b_start = false;

  if (i1.getInput(STOP)) {  // stop all if stop button pressed
    i1.setAllMotorsSTOP();
    i2.setAllMotorsSTOP();
    i3.setAllMotorsSTOP();
    reset = false;
  } else if (i1.getInput(RESET)) {  // reset button pressed
    controller.ftMessageToDisplay(1, 1, "Alles Resetten", true);
    reset = resetAllStations();
  } else {
    if (i1.getInput(START)) {  // start button pressed
      if (!reset) {
        reset = resetAllStations();
      }
      b_start = true;
    }
    if (b_start) {  // het onderdeel kan geladen worden

      // schuif sample op band
      loadSample();
      // start band 1
      controller.ftMessageToDisplay(1, 1, "Naar band 1", true);
      runBeltsUntil(1);
      // start drill
      controller.ftMessageToDisplay(1, 1, "Boren", true);
      drill();

      // start band 2 & 1
      controller.ftMessageToDisplay(1, 1, "Naar band 2", true);
      runBeltsUntil(2);

      // start router 1
      controller.ftMessageToDisplay(1, 1, "Frees 1", true);
      router1();

      // start band 3 & 2
      controller.ftMessageToDisplay(1, 1, "Naar band 3", true);
      runBeltsUntil(3);
      // start router 2
      controller.ftMessageToDisplay(1, 1, "Frees 2", true);
      router2();

      // start band 4 and 3
      controller.ftMessageToDisplay(1, 1, "Naar band 4", true);
      exitBelt();

      // roteer band
      rotateBelt();

      // pickup de parcel and move to beginning
      controller.ftMessageToDisplay(1, 1, "Naar lossen", true);
      transport();

      i1.setAllMotorsSTOP();
      i2.setAllMotorsSTOP();
      i3.setAllMotorsSTOP();
      reset = false;
    }
  }
}

//
// rotate belt 90 degrees
void rotateBelt() {
  bool go_on = true;

  controller.ftMessageToDisplay(1, 1, "Roteer band", true);

  while ((go_on) && proceed()) {
    i1.getInputs();
    i2.getInputs();
    delay(5);
    go_on = i2.getMotorUntil(ft_M4, ft_E6, ON, CCW);
  }
}

//
// operate the top gantry
void transport() {

  bool go_on = true;

  i1.setAllMotorsSTOP();
  i2.setAllMotorsSTOP();
  i3.setAllMotorsSTOP();

  // beweeg naar voren naar de laadpositie
  while ((go_on) && proceed()) {
    i1.getInputs();
    delay(5);
    i3.getInputs();
    delay(5);
    go_on = i3.getMotorUntil(MOTOR_Y, ft_E5, ON, CCW);
  }

  // arm naar beneden
  go_on = true;
  i3.setMotor(MOTOR_Z, CCW);
  while (go_on && proceed()) {
    i1.getInputs();
    i2.getInputs();
    i3.getInputs();
    delay(8);

    //   if ((i3.getInput(ft_E8) == ON) || (i2.getInput(ft_E8) == ON) ) {
    if (i2.getInput(ft_E8)) {
      controller.ftMessageToDisplay(1, 1, "Arm Stop", true);
      go_on = false;
    }
  }
  i3.setMotorSTOP(MOTOR_Z);

  // switch on magnet
  setMagnetON();

  // beweeg naar boven
  go_on = true;
  while ((go_on) && proceed()) {
    i1.getInputs();
    i3.getInputs();
    delay(5);
    go_on = i3.getMotorUntil(MOTOR_Z, ft_E7, ON, CW);
  }

  // beweeg naar achter
  go_on = true;
  while ((go_on) && proceed()) {
    i1.getInputs();
    i3.getInputs();
    delay(5);
    go_on = i3.getMotorUntil(MOTOR_Y, ft_E6, ON, CW);
  }

  // beweeg links naar de lospositie
  go_on = true;
  while ((go_on) && proceed()) {
    i1.getInputs();
    i3.getInputs();
    delay(5);
    go_on = i3.getMotorUntil(MOTOR_X, ft_E4, ON, CW);
  }

  // beweeg naar voren naar de lospositie
  go_on = true;
  while ((go_on) && proceed()) {
    i1.getInputs();
    i3.getInputs();
    delay(5);
    go_on = i3.getMotorUntil(MOTOR_Y, ft_E5, ON, CCW);
  }

  // beweeg naar beneden
  go_on = true;
  while ((go_on) && proceed()) {
    i1.getInputs();
    i3.getInputs();
    delay(5);
    go_on = i3.getMotorUntil(MOTOR_Z, ft_E8, ON, CCW);
  }

  // release load, switch off magnet
  setMagnetOFF();

  // beweeg naar boven
  go_on = true;
  while ((go_on) && proceed()) {
    i1.getInputs();
    i3.getInputs();
    delay(5);
    go_on = i3.getMotorUntil(MOTOR_Z, ft_E7, ON, CW);
  }

  // beweeg naar achter
  go_on = true;
  while ((go_on) && proceed()) {
    i1.getInputs();
    i3.getInputs();
    delay(5);
    go_on = i3.getMotorUntil(MOTOR_Y, ft_E6, ON, CW);
  }

  // beweeg rechts naar de laadpositie
  go_on = true;
  while ((go_on) && proceed()) {
    i1.getInputs();
    i3.getInputs();
    delay(5);
    go_on = i3.getMotorUntil(MOTOR_X, ft_E3, ON, CCW);
  }
}

void loadSample() {
  // neem aan dat er gereset is. controleer of er een vorm ligt.

  bool go_on1 = true;
  bool go_on2 = true;
  bool b_light = false;
  bool b_band = false;

  controller.ftMessageToDisplay(1, 1, "laad werkstuk", true);

  while ((go_on1 || go_on2) && proceed()) {
    i1.getInputs();

    if (i1.getInput(ft_E11) && !b_light) {
      // switch on light
      i1.setOutputON(LAMP_1);
      delay(5);  // needs a bit of time to switch on the light
      b_light = true;
    }

    if (b_light) {
      if (go_on1) {
        go_on1 = i1.getMotorUntil(LAAD, ft_E10, ON, CCW);
      }

      if (!b_band) {
        if (!i1.getInput(ft_E12)) {
          // start band
          b_band = true;
          // controller.ftMessageToDisplay(1, 1, "lichtcel uit");
        }
      } else {
        go_on2 = i1.getMotorUntil(BAND_1, ft_E13, ON, CCW);
      }
    }
  }

  // switch off lamp
  i1.setOutputOFF(LAMP_1);
  // return push lever
  go_on1 = true;
  while ((go_on1) && proceed()) {
    i1.getInputs();
    delay(5);
    go_on1 = i1.getMotorUntil(LAAD, ft_E9, ON, CW);
  }
}

//
// boorprocedure
void drill() {

  FTtimer drillTime(5000);  // boortijd 10 seconden

  bool go_on = true;

  while (go_on && proceed()) {  // boorkop naar beneden
    i1.getInputs();
    go_on = i1.getMotorUntil(BOOR_Z, ft_E15, ON, CW);
  }
  drillTime.reset();       // start timer
  i1.setOutputON(BOOR_R);  // boor roteren

  go_on = true;
  while (go_on && proceed()) {
    i1.getInputs();  // the interface needs I/O during timers
    if (drillTime.ready()) {
      i1.setOutputOFF(BOOR_R);  // boor stoppen
      go_on = false;
    }
  }

  go_on = true;
  while (go_on && proceed()) {  // boorkop omhoog
    i1.getInputs();
    go_on = i1.getMotorUntil(BOOR_Z, ft_E14, ON, CCW);
  }
}

//
// frees 1 procedure
void router1() {

  FTtimer routerTime(4000);  // freestijd 10 seconden

  bool b_routerDown = true;
  bool go_on = true;

  while (go_on && proceed()) {  // freeskop naar beneden
    i1.getInputs();
    delay(3);
    go_on = i1.getMotorUntil(F1_Z, ft_E2, ON, CCW);
  }

  while (go_on && proceed()) {  // free switch
    i1.getInputs();
    delay(3);
    go_on = i1.getMotorUntil(F1_W, ft_E3, OFF, CCW);
  }

  for (int i = 0; i < 3; i++) {  //3
    // move to next correct position
    // move to next free position
    // i1.getMotorUntilCount(F1_W, ft_E3, ON, CCW, 3);
    go_on = true;
    while (go_on && proceed()) {  // free switch
      i1.getInputs();
      delay(3);
      go_on = i1.getMotorUntil(F1_W, ft_E3, OFF, CCW);
    }

    go_on = true;
    while (go_on && proceed()) {
      i1.getInputs();  // the interface needs I/O during timers
      delay(3);
      go_on = i1.getMotorUntil(F1_W, ft_E3, ON, CCW);
    }

    routerTime.reset();    // start timer
    i1.setOutputON(F1_R);  // frees roteren

    go_on = true;
    while (go_on && proceed()) {
      i1.getInputs();  // the interface needs I/O during timers
      if (routerTime.ready()) {
        i1.setOutputOFF(F1_R);  // frees stoppen
        go_on = false;
      }
    }
    i1.setOutputOFF(F1_R);  
  }
/*
  go_on = true;
  while (go_on && proceed()) {  // routerkop omhoog
    i1.getInputs();
    delay(3);
    go_on = i1.getMotorUntil(F1_Z, ft_E1, ON, CW);
  }
  */
}

//
// frees 2 procedure
void router2() {

  bool b_routerDown = true;
  bool go_on = true;

  while (go_on && proceed()) {  // freeskop naar beneden
    i2.getInputs();
    go_on = i2.getMotorUntil(F2_Z, ft_E2, ON, CW);
  }

  go_on = true;
  i2.setOutputON(F2_R);  // frees roteren

  while (go_on && proceed()) {  // freeskop naar voren
    i2.getInputs();
    // i1.getInputs();
    go_on = i2.getMotorUntil(F2_Y, ft_E3, ON, CW);
  }

  i2.setOutputOFF(F2_R);  // frees rotatie stoppen

  go_on = true;
  while (go_on && proceed()) {  // freeskop omhoog
    i2.getInputs();
    i1.getInputs();

    go_on = i2.getMotorUntil(F2_Z, ft_E1, ON, CCW);
  }

  // band 4 en lamp aan
  i2.setOutputON(BAND_4);  // do this so the light cell activates early

  go_on = true;
  while (go_on && proceed()) {  // freeskop naar achter
    i2.getInputs();
    i1.getInputs();

    go_on = i2.getMotorUntil(F2_Y, ft_E4, ON, CCW);
  }
}


// generic belt procedure
void runBeltsUntil(int belt) {
  // the first belt is the preceeding one, the second is the latter one with the eye/switch
  bool go_on = true;
  bool b_first = true;

  Serial.println("run belts until");
  while (go_on && proceed()) {  //

    i1.getInputs();
    i1.printInputBuffer();

    if (belt == 1) {

      go_on = i1.getMotorUntil(BAND_1, ft_E13, ON, CCW);

    } else if (belt == 2) {

      Serial.println("belt 2");
      // also switch on Belt 1
      if (b_first) {
        i1.setMotorCCW(BAND_1);
        b_first = false;
      }
      go_on = i1.getOutputUntil(BAND_2, ft_E4, ON);
      if (!go_on) {
        i1.setMotorSTOP(BAND_1);
      }

    } else if (belt == 3) {

      // first switch on Belt 2
      if (b_first) {
        i1.setOutputON(BAND_2);
        b_first = false;
      }
      go_on = i1.getOutputUntil(BAND_3, ft_E5, ON);
      if (!go_on) {
        i1.setOutputOFF(BAND_2);
      }

    } else {
    }  // do nothing
  }
}

//
// start bent 3 & 4
void exitBelt() {
  bool go_on = true;

  i2.setOutputON(BAND_4);
  i1.setOutputON(BAND_3);
  delay(4);

  while (go_on && proceed()) {  //

    i1.getInputs();
    i2.getInputs();
    delay(3);
    go_on = i2.getOutputUntil(BAND_4, ft_E7, OFF);

    if (!go_on) {
      i1.setOutputOFF(BAND_3);
    }
  }
}
//
// utility function that checks whether the STOP button is pressed
bool proceed() {

  if (i1.getInput(STOP)) {
    Serial.println("STOP");
    return (false);
  } else {
    return (true);
  }
}

//
// reset all stations to avoid interference or clashes
bool resetAllStations() {

  bool go_on1 = true;
  bool go_on2 = true;
  bool go_on3 = true;
  bool go_on4 = true;
  bool go_on5 = true;
  bool go_on6 = true;
  bool go_on7 = true;
  bool go_on8 = true;
  bool go_on9 = true;

  while (go_on1 || go_on2 || go_on3 || go_on4 || go_on5) {

    i1.getInputs();
    i3.getInputs();
    // beweeg transport arm omhoog
    go_on1 = i3.getMotorUntil(MOTOR_Z, ft_E7, ON, CW);
    // beweeg transport arm naar voren
    go_on2 = i3.getMotorUntil(MOTOR_Y, ft_E6, ON, CW);
    // beweeg boor omhoog
    go_on3 = i1.getMotorUntil(BOOR_Z, ft_E14, ON, CCW);
    // beweeg frees 1 omhoog
    go_on4 = i1.getMotorUntil(F1_Z, ft_E1, ON, CW);
    // beweeg laadschuif links
    go_on5 = i1.getMotorUntil(LAAD, ft_E9, ON, CW);
  }

  while (go_on6 || go_on7 || go_on8 || go_on9) {

    i2.getInputs();
    i3.getInputs();
    // beweeg transport arm naar rechts
    go_on6 = i3.getMotorUntil(MOTOR_X, ft_E3, ON, CCW);
    // beweeg frees 2 omhoog
    go_on7 = i2.getMotorUntil(F2_Z, ft_E1, ON, CCW);
    // beweeg frees 2 naar achter
    go_on8 = i2.getMotorUntil(F2_Y, ft_E4, ON, CCW);
    // beweeg plateau recht
    go_on9 = i2.getMotorUntil(ROTATE, ft_E5, ON, CW);
  }

  return (true);
}

void setMagnetON() {
  i3.setMotorCCW(MAGNET);
}

void setMagnetOFF() {
  i3.setMotorCW(MAGNET);
  i3.setMotorSTOP(MAGNET);
  i3.setMotorCCW(MAGNET);
  i3.setMotorCW(MAGNET);
  i3.setMotorSTOP(MAGNET);
}