/* 
 *  Prototype program for Controllino Modification Module 1
 *  
 * uses one serial interfaces with extension 
 * 
 * Interface:
 *    M1: lopende band lamp
 *    M2: lopende band motor 
 *    M3: schuiver motor
 *    M4: kolom motor
 *    E1: switch 1: fotocel start band
 *    E2: switch 2: schuiver links
 *    E3: switch 3: schuiver rechts
 *    E4: kolom boven
 *    E5: kolom onder
 * Extension:
 *    M5: boor motor
 *    M6: lopende band exit motor
 *    
 *
 * functiebeschrijving
 * ===================
 * 1. Bij initialisatie gaat de schuifarm naar links en de kolomboor omhoog
 * 2. Wanneer de lichtcel aan het begin van de band wordt onderbroken gaat de band lopen
 * 3. De band loopt voor drie/vier seconden en stopt
 * 4. De kolomboor gaat naar beneden en stopt
 * 5. De boormachine draait 3 seconden
 * 6. De kolomboor gaat omhoog
 * 7. De schuiarm gaat naar rechts
 * 8. De lopende band rechts loopt gedurende drie seconden
*/

// include the FT legacy controller classes
#include <FTlegacy.h>

// define Program name
const char* programName = "Cont. Boor 1";

// create interface objects. first argument = {PAR, PAREX, SER, SEREX, ROBO, DID, ADA},
// second is interface number for the type.
FTlegacy sFace1(SEREX, 1);

// create board object. first argument = {UNO, MEGA, DUE),
// second is the display {D1306L, D1306S, D1604, D1602, D2004}
FTcontroller controller(D2004);

FTtimer drillTimer(5000);
FTtimer beltTimer(7000);

// belt status parameters
bool beltStarted = false;
bool beltReady = false;
bool beltRightReady = false;

// arm status parameters
bool armLeft = false;
bool armRight = false;
bool armDown = false;
bool armRaised = false;
bool motorON = false;

void setup() {
  Serial.begin(9600);
  Serial.println(programName);

  // setup serial communicates for main program
  if (board == 1) {
    Serial.println("Arduino Uno");
  } else if (board == 2) {
    Serial.println("Arduino Mega");
  } else if (board == 1) {
    Serial.println("unknown/error");
  }

  // prepare for serial port to be used in library
  // controller.addInterface(sFace1);
  // controller.addInterface(sFace2);
  controller.begin(programName);
  sFace1.begin();

  sFace1.setAllMotorsSTOP();
}

//---------------------------------------------------------------------------
// moves arm left until switch is closed
// returns true when condition is met, else false
bool moveArmLeft() {
  Serial.println("move arm left");
  return (!sFace1.getMotorUntil(ft_M3, ft_E2, ON, CW));
}

//---------------------------------------------------------------------------
// moves arm right until switch is closed
// returns true when condition is met, else false
bool moveArmRight() {
  Serial.println("move arm right");
  return (!sFace1.getMotorUntil(ft_M3, ft_E3, ON, CCW));
}

//---------------------------------------------------------------------------
// moves arm up until switch is closed
// returns true when condition is met, else false
bool moveArmUp() {
  Serial.println("move arm up");
  return (!sFace1.getMotorUntil(ft_M4, ft_E4, ON, CW));
}

//---------------------------------------------------------------------------
// moves arm down until switch is closed
// returns true when condition is met, else false
bool moveArmDown() {
  Serial.println("move arm down");
  return (!sFace1.getMotorUntil(ft_M4, ft_E5, ON, CCW));
}

//---------------------------------------------------------------------------
// Run belt for several seconds after parcel appears at gate
// Returns true when belt finally stopped, else false
bool startBelt(int E, int M) {  // run belt for several seconds

  // either wait for fotocell or start immediatel (E==0)
  if (!sFace1.getInput(E) || E == 0) {
    if (!beltStarted) {
      beltTimer.reset();
      beltStarted = true;
    }
  }

  if (beltStarted) {
    // stop the belt after beltDuration millis, continue running otherwise
    if ( beltTimer.ready() ) {
      sFace1.setMotorSTOP(M);
      beltStarted = false;
      return (true);
    } else {
      sFace1.setMotorCCW(M);
      return (false);
    }
  } else {
    return (false);
  }
}


//---------------------------------------------------------------------------
void loop() {

  // retrieve input from interfaces if any :: problem is that it alse sends output
  sFace1.getInputs();

  // move the arm to start position
  if (!armLeft) {
    armLeft = moveArmLeft();
    sFace1.setOutputON(ft_O1);
  }

  // wait for a parcel to appear run the belt
  if (!beltReady && armLeft) {
    beltReady = startBelt(ft_E1, ft_M2);
  }

  if (beltReady) {

    // lower arm when left and parcel delivered
    if (armLeft && !armDown) {
      armDown = moveArmDown();
      sFace1.setOutputOFF(ft_O1);
      drillTimer.reset();
    }

    // initiate drill motor when arm down
    if (armDown && !motorON) {
      sFace1.setMotorCW(ft_M5);
      if (drillTimer.ready() && !motorON) {
        motorON = true;
      }
    }

    // raise arm when done drilling
    if (motorON && !armRaised) {
      armRaised = moveArmUp();
      sFace1.setMotorSTOP(ft_M5);
    }

    // move arm rigth when raised
    if (armRaised && !armRight) {
      armRight = moveArmRight();
     // beltRightReady = startBelt(0, ft_M6);
    }

    // release suction
    if (armRaised && !beltRightReady) {
      // switch on belt right
       beltRightReady = startBelt(0, ft_M6);
    }

    // Reset all state variables
    if (beltRightReady) {
      armLeft = false;
      armRight = false;
      beltReady = false;
      armRaised = false;
      armDown = false;
      motorON = false;
      beltRightReady = false;
    }
  }

  // update the display
  sFace1.ftUpdateDisplay();
}
