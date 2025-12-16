/* 
   Program to test Trainingsroboter model 39461 with one parallel interface

   If used with a different interface note that the max voltage on the 
   optocouplers that measure motor movement is 5V maximum
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
  'g0': move to (400,400)
  'g1': move to (400,1100)
  'g2': move to (200,200)
  'g3': move to (600,600)
  'm1': relative move to (100,100)
  'm2': relative move to (-100,-100)

*/

#define STOP ft_E8  //  E8  : Stop

#define ROTATE ft_M1    //    M1 : Frees 1 op/neer (
#define SHOULDER ft_M2  //    M2 : Frees 1 wissel
#define ELBOW ft_M3     //    O5 : Band 2
#define CLAW ft_M4      //    O6 : Frees 1 rotatie

// define Program name
const char* programName = "39461-tst";

// create interface objects. first argument = {PAR, PAREX, SER, SEREX, ROBO},
// second is interface number for the type.
FTlegacy interface(PAR, 1);

// create board object. first argument = {UNO, MEGA, DUE),
// second is the display {D1306L, D1306S, D1604, D1602, D2004}
FTcontroller controller(D2004);

int position[4] = { 0, 0, 0, 0 };
int minPosition[4] = { 0, -900, 0, 0 };
int maxPosition[4] = { 0, 900, 1100, 1100 };
bool homeState[4] = { false, false, false, false };

bool armForward(int steps);
bool armBackward(int steps);
bool armFB(int steps);
bool rotateCW(int steps);
bool rotateCCW(int steps);
bool armUp(int steps);
bool armDown(int steps);

void setup() {
  Serial.begin(9600);  // set up Serial library at 9600 bps
  Serial.println("WKS 30581 input test");

  // prepare for serial port to be used in library
  controller.addInterface(interface);
  controller.begin(programName);

  interface.begin();
  interface.setAllMotorsSTOP();
}

bool first = true;
int i = 0;
int j = 0;
bool done = false;

String command, numberChar, choice;
int number;


void loop() {

  int targetP2 = 0;
  int targetP3 = 0;

  interface.getInputs();
  interface.printInputBuffer();

  if (first) {
    resetAllStations();
    first = false;
  }

  while (i < 5) {
    armLower(50);
    i++;
  }

  while (j < 5) {
    armUpper(40);
    j++;
  }

  if (!done) {
    done = rotate(-400);
    done = rotate(400);
  }

  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    choice = command.substring(0, 1);
    Serial.println(command);
    Serial.println(choice);

    if (choice == "u") {
      Serial.println("up");
      armLower(-100);
    } else if (choice == "d") {
      Serial.println("down");
      armLower(100);
    } else if (choice == "l") {
      Serial.println("left");
      rotate(-100);
    } else if (choice == "r") {
      Serial.println("right");
      rotate(100);
    } else if (choice == "f") {
      Serial.println("forward");
      armUpper(100);
    } else if (choice == "b") {
      Serial.println("backward");
      armUpper(-100);
    } else if (choice == "c") {
      Serial.println("close");
      gripperClose();
    } else if (choice == "o") {
      Serial.println("open");
      gripperOpen();
    } else if (choice == "i") {
      resetAllStations();
    } else if (command == "g0") {
      Serial.println("goto");
      targetP2 = 400;
      targetP3 = 400;
      Serial.println(minP2(targetP2, targetP3));
      Serial.println(maxP2(targetP2, targetP3));
      Serial.println(minP3(targetP2, targetP3));
      Serial.println(maxP3(targetP2, targetP3));
      gotoPosition(targetP2, targetP3);
    } else if (command == "g1") {
      Serial.println("goto");
      targetP2 = 400;
      targetP3 = 1100;
      Serial.println(minP2(targetP2, targetP3));
      Serial.println(maxP2(targetP2, targetP3));
      Serial.println(minP3(targetP2, targetP3));
      Serial.println(maxP3(targetP2, targetP3));
      gotoPosition(targetP2, targetP3);
    } else if (command == "g2") {
      Serial.println("goto");
      targetP2 = 200;
      targetP3 = 200;
      Serial.println(minP2(targetP2, targetP3));
      Serial.println(maxP2(targetP2, targetP3));
      Serial.println(minP3(targetP2, targetP3));
      Serial.println(maxP3(targetP2, targetP3));
      gotoPosition(targetP2, targetP3);
    } else if (command == "g3") {
      //Serial.println("goto");
      targetP2 = 600;
      targetP3 = 600;
      Serial.println(minP2(targetP2, targetP3));
      Serial.println(maxP2(targetP2, targetP3));
      Serial.println(minP3(targetP2, targetP3));
      Serial.println(maxP3(targetP2, targetP3));
      gotoPosition(targetP2, targetP3);
    }

    if (choice = "m") {
      if (command == "m0") {
        targetP2 = 100;
        targetP3 = 100;
      } else if (command == "m1") {
        targetP2 = -100;
        targetP3 = -100;
      }
      move2(int(ELBOW), ft_E6, targetP2, int(SHOULDER), ft_E4, targetP3);
    }
    // set command / coice to zero, serial read goes bezerk...
    choice = "0";
    command = "0";
  }

  interface.ftUpdateDisplay();
}

bool rotate(int steps) {
  return move(ROTATE, ft_E2, steps);
}

bool armLower(int steps) {
  return move(ELBOW, ft_E6, steps);
}

bool armUpper(int steps) {
  return move(SHOULDER, ft_E4, steps);
}

bool move2(int M2, int E2, int steps2, int M3, int E3, int steps3) {

  int s2 = 0;
  int state2 = 0;
  int prev_state2 = 0;
  int s3 = 0;
  int state3 = 0;
  int prev_state3 = 0;


  if ((steps2 > 0) && (steps3 > 0)) {
    interface.setMotorCW(M2);
    interface.setMotorCW(M3);
    while ((s2 < steps2) || (s3 < steps3)) {
      interface.getInputs();
      state2 = interface.getInput(E2);
      state3 = interface.getInput(E3);
      if (state2 != prev_state2) {
        prev_state2 = state2;
        s2++;
      }
      if (state3 != prev_state3) {
        prev_state3 = state3;
        s3++;
      }
      Serial.print(s2);
      Serial.print(" ");
      Serial.println(s3);

      if (s2 > steps2 - 1) {
        interface.setMotorSTOP(M2);
      }
      if (s3 > steps3 - 1) {
        interface.setMotorSTOP(M3);
      }
    }
  } else if ((steps2 < 0) && (steps3 < 0)) {
    interface.setMotorCCW(M2);
    interface.setMotorCCW(M3);
    while ((s2 < -steps2) || (s3 < -steps3)) {
      interface.getInputs();
      state2 = interface.getInput(E2);
      state3 = interface.getInput(E3);
      if (state2 != prev_state2) {
        prev_state2 = state2;
        s2++;
      }
      if (state3 != prev_state3) {
        prev_state3 = state3;
        s3++;
      }
      Serial.print(s2);
      Serial.print(" ");
      Serial.println(s3);
      if (s2 > -(steps2 + 1)) {
        interface.setMotorSTOP(M2);
      }
      if (s3 > -(steps3 + 1)) {
        interface.setMotorSTOP(M3);
      }
    }
  }

  homeState[M2] = false;
  position[M2] = position[M2] + steps2;
  Serial.print("position M2: ");
  Serial.print(M2);
  Serial.print(" ");
  Serial.println(position[M2]);
  homeState[M3] = false;
  position[M3] = position[M3] + steps2;
  Serial.print("position M3: ");
  Serial.print(M3);
  Serial.print(" ");
  Serial.println(position[M3]);
  return (true);
}


bool move(int M, int E, int steps) {

  int s = 0;
  int state = 0;
  int prev_state = 0;

  if (steps > 0) {
    while (s < steps) {
      interface.getInputs();
      state = interface.getInput(E);
      interface.setMotorCW(M);
      if (state != prev_state) {
        prev_state = state;
        s++;
      }
    }
    interface.setMotorSTOP(M);
  } else if (steps < 0) {
    while (s < -steps) {
      interface.getInputs();
      interface.setMotorCCW(M);
      state = interface.getInput(E);
      if (state != prev_state) {
        prev_state = state;
        s++;
      }
    }
    interface.setMotorSTOP(M);
  }
  homeState[M] = false;
  position[M] = position[M] + steps;
  Serial.print("position M: ");
  Serial.print(M);
  Serial.print(" ");
  Serial.println(position[M]);

  return (true);
}

bool gotoPosition(int targetP2, int targetP3) {

  if (checkLimit(targetP2, targetP3)) {
    armLower(targetP3 - position[3]);
    armUpper(targetP2 - position[2]);
    return (true);
  } else {
    return (false);
  }
}

bool checkLimit(int targetP2, int targetP3) {
  if ((targetP2 > minP2(targetP2, targetP3)) && (targetP2 < maxP2(targetP2, targetP3)) && (targetP3 > minP3(targetP2, targetP3)) && (targetP3 < maxP3(targetP2, targetP3))) {
    return true;
  } else {
    return false;
  }
}

int maxP2(int targetP2, int targetP3) {

  int P2;
  P2 = min(1200, max(targetP2, 1200));

  if (targetP3 >= 0) {
    P2 = min(1200, targetP3 + 400);
  }
  return (P2);
};

int minP2(int targetP2, int targetP3) {

  int P2;
  P2 = min(0, max(targetP2, 0));

  if (targetP3 > 940) {
    P2 = 2 * (targetP3 - 940);
  }
  return (P2);
};

int maxP3(int targetP2, int targetP3) {

  int P3;
  P3 = max(1140, min(targetP3, 1140));

  if (targetP2 < 400) {
    P3 = 940 + targetP2 / 2;
  }
  return (P3);
};

int minP3(int targetP2, int targetP3) {

  int P3;
  P3 = min(0, max(targetP3, 0));

  if (targetP2 > 400) {
    P3 = targetP2 - 400;
  }
  return (P3);
};

//
// utility function that checks whether the STOP button is pressed
bool proceed() {

  if (interface.getInput(STOP)) {
    Serial.println("STOP");
    return (false);
  } else {
    return (true);
  }
}

bool resetRotation() {
  homeState[ROTATE] = true;
  position[ROTATE] = 0;
  return (interface.setMotorUntil(ROTATE, ft_E1, OFF, CW));
};

bool resetArmUpper() {
  homeState[SHOULDER] = true;
  position[SHOULDER] = 0;
  return (interface.setMotorUntil(SHOULDER, ft_E3, OFF, CCW));
};

bool resetArmLower() {
  homeState[ELBOW] = true;
  position[ELBOW] = 0;
  return (interface.setMotorUntil(ELBOW, ft_E5, OFF, CCW));
};

//
// reset all stations to avoid interference or clashes
bool resetAllStations() {

  resetRotation();
  resetArmUpper();
  resetArmLower();
  gripperInitial();
  return (true);
}

bool gripperOpen() {
  bool go_on = true;
  if (!homeState[CLAW]) {
    go_on = interface.setMotorUntil(CLAW, ft_E7, OFF, CW);
    go_on = interface.setMotorUntil(CLAW, ft_E7, ON, CW);
    go_on = interface.setMotorUntil(CLAW, ft_E7, ON, CW);
    go_on = interface.setMotorUntil(CLAW, ft_E7, ON, CW);
    homeState[CLAW] = true;
  }
  return (true);
}

bool gripperClose() {
  bool go_on = true;
  if (homeState[CLAW]) {
    go_on = interface.setMotorUntil(CLAW, ft_E7, OFF, CCW);
    go_on = interface.setMotorUntil(CLAW, ft_E7, ON, CCW);
    go_on = interface.setMotorUntil(CLAW, ft_E7, ON, CCW);
    go_on = interface.setMotorUntil(CLAW, ft_E7, ON, CCW);
    homeState[CLAW] = false;
  }
  return (true);
}

bool gripperInitial() {
  bool go_on = true;
  FTtimer gripperTimer(2000);

  if (!homeState[CLAW]) {
    // close for two seconds
    while (!gripperTimer.ready()) {
      interface.setMotorCCW(CLAW);
      interface.getInputs();
    }
    interface.setMotorSTOP(CLAW);
    // open until switch is released
    go_on = interface.setMotorUntil(CLAW, ft_E7, OFF, CW);
    go_on = interface.setMotorUntil(CLAW, ft_E7, ON, CW);
    go_on = interface.setMotorUntil(CLAW, ft_E7, ON, CW);
    homeState[CLAW] = true;
  }

  return (true);
}