/*******************************************************************************************
 * Arduino Library to control a legacy parallel and serial fischertechnik(r) computing interfaces.
 * Original code by Jeroen Regtien, December 2023, with revisions since.
 *
 * Supported interfaces:
 *      1984:  Parallel Interface Commodore, IBM, Atari: 30562, 30563, 30565
 *      1991:  Parallel Universal Interface: 30520
 *      1991:  Parallel CVK Interface: XXXXX
 *      1997:  Serial Intelligent Interface: 30402
 *      2004:  ROBO interface: 93293
 * 
 * The library consists of four files:
 * FTcontroller.h - headerfile for the FTcontroller class, which covers the type of Arduino board used
 * FTcontroller.cpp - C++ implementation of class, methods and utility functions
 * FTlegacy.h - header file for the FTlegacy class containing information to read/write from/to the supported interfaces
 * FTlegacy.c - C++ implementation of class, methods and utility functions
 *
 * References:
 *      https://www.ftcommunity.de/ftpedia/2014/2014-1/ftpedia-2014-1.pdf
 *      https://www.ftcommunity.de/ftpedia/2014/2014-2/ftpedia-2014-2.pdf
 *      https://www.ftcommunity.de/ftpedia/2023/2023-4/ftpedia-2023-4.pdf
 *      https://www.ftcommunity.de/ftpedia/2025/2025-2/ftpedia-2025-2.pdf
 *
 * The MIT License (MIT)
 * 
 * Copyright (c) 2023 Jeroen Regtien
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

  Version 0.6 - Added LCD 2004 option
  Version 0.7 - serial extension capability, code cleanup, add actuators, lamps
  Version 0.7X - Added functionality for Didacta Uno and Meg. code cleanup
  Version 0.8 - Used for several workstations. Actuators removed as object. 
  Version 0.9 - Parallel Extensions added. Analog input via parallel interface now works.
                Consolidated and rationalised. Given Arduino UNO memory limitations, 
                two separate classes introduced: FTlegacy for legacy fischertechnik 
                interfaces and FTmodule also including 3rd party shields and controllers. 
                Version after many prototype tests.
  Version 1.0 - First operational release. Added FTplotter and FTencoder classes \n

 *********************************************************************************************/
#include "FTlegacy.h"

extern FTlegacy* ifaceList[];

int mIndex1(int);
int mIndex2(int);

unsigned long previousMillis = 0;
unsigned long lcdIntervalMillis = 2000;
bool firstLCD = true;
bool initLCD = true;

// set LCD address, number of columns and rows
hd44780_I2Cexp lcd(0x27, lcdColumns, lcdRows);

// define serial ports
#if defined(ARDUINO_UNOR4_MINIMA)
SoftwareSerial mySerial(1, 0);  // RX, TX
#else
SoftwareSerial mySerial(10, 11);  // RX, TX
#endif

// global variable for display
ftDisplayTypes ftDisplayType;

// global constant to indicate presence of LCD
bool connectedToLCD;
//-------------------------------------------------------------------------
// Simpler constructor for when connected to consecutive Arduino pins
FTlegacy::FTlegacy(int typeFT, int numberFT) {

  type = typeFT;      // type of interface
  number = numberFT;  // sequence number of the interface

  ifaceList[ifaceCount] = this;
  ifaceCount++;

  if (type == PAR || type == PAREX) {
    InitPar(
      startPin[number - 1],
      startPin[number - 1] + 1,
      startPin[number - 1] + 2,
      startPin[number - 1] + 3,
      startPin[number - 1] + 4,
      startPin[number - 1] + 5,
      startPin[number - 1] + 6,
      startPin[number - 1] + 7);
      
      if (type == PAREX) {
        numUnits = 2;
      } else {
        numUnits = 1;
      }
  } else if (type == SER) {
    // for serial assume interfaces are connected to correct serial port
  } else if (type == ROBO) {
      type = SER;
  }
}

//---------------------------------------------------------------------------
// zero constructor
FTlegacy::FTlegacy() {
}

//---------------------------------------------------------------------------
// Initialize & Reset
bool FTlegacy::begin() {

  bool result = true;
  int code = 1;

  if (type == PAR || type == PAREX) {
    // Initialize pins
    for (int u = 0; u < NumPins; u++) {
      if (u == DATACOUNTIN || u == COUNTIN) {
        pinMode(m_pin[u], INPUT);
      } else {
        pinMode(m_pin[u], OUTPUT);
      }
    }
    pinMode(pinParEx, INPUT);
    pinMode(pinParEy, INPUT);

    // Reset all outputs
    setOutputs();

    // Initialize the output from the interface (our input).
    // The three possible output sources (the two timers of the analog inputs
    // and the shift-out pin of the digital inputs) are ORed together and
    // inverted on the interface. The timers may be in a triggered state,
    // so we un-trigger them first. The digital input shifter may also be
    // in any state until we clock it enough times to shift data in from the
    // open (pulled-down) serial input of the last cascaded interface.
    // The analog input timers may take up to ~2.8ms to return their outputs
    // to LOW state (see the Analog() function). If we keep pulsing the
    // clock while we wait for the timers, all individual sources will
    // eventually turn LOW, which makes the NOR circuit (our input) go HIGH.
    unsigned long starttime = millis();

    // Un-trigger the analog inputs
    digitalWrite(m_pin[TRIGGERX], HIGH);
    digitalWrite(m_pin[TRIGGERY], HIGH);

    long int tBegin = 0;
    long int tEnd = 0;
    int pinTrigger;

    tBegin = micros();

    while (!digitalRead(m_pin[DATACOUNTIN])) {
      tEnd = millis() - starttime;
      if (ftInfo > 2) {Serial.println(tEnd);}
      if (tEnd > 50) {
        result = false;
          if (ftInfo > 2) {Serial.println("No analog signal");}
        break;
      }
    }

    if (result) {
      if (connectedAnalog() > 0) {
        result = false;
        if (ftInfo > 0) {Serial.println(F("No constant analog signal"));}
      } else {
        if (ftInfo > 0) {Serial.println(F("Analog signal OK"));}
      }
    }
  }

  setAllMotorsSTOP();

  return result;
}

FTlegacy* FTlegacy::ifaceList[8];
int FTlegacy::ifaceCount = 0;

//---------------------------------------------------------------------------
// Get 8 digital inputs
void FTlegacy::getInputs() {

  if (type == PAR || type == PAREX) {

    digitalWrite(m_pin[LOADIN], HIGH);
    digitalWrite(m_pin[CLOCK], LOW);
    digitalWrite(m_pin[CLOCK], HIGH);
    digitalWrite(m_pin[LOADIN], LOW);

    for (int j = 1; j <= numUnits; j++) {
      for (int i = 8; i > 0; i--) {
        m_in[i + (j - 1) * 8] = (digitalRead(m_pin[DATACOUNTIN]) == LOW);  // inverse notation to get similarity between interfaces
        digitalWrite(m_pin[CLOCK], LOW);
        digitalWrite(m_pin[CLOCK], HIGH);
      }
    }

  } else if (type == SER) {
    outByte[0] = 0xC1;

    if (board == MEGA) {
      readWriteMEGA(number, 2);
    } else if (board == UNO) {
      readWriteUNO(2);
    }

    // interpret first byte
    iD = (char)inByte[0];
    for (int i = 0; i <= 7; i++) {
      // use offset as m_ini starts at index 1
      m_in[i + 1] = bitRead(iD, i);
    }

  } else if (type == SEREX) {
    outByte[0] = 0xC2;

    if (board == MEGA) {
      readWriteMEGA(number, 3);
    } else if (board == UNO) {
      readWriteUNO(3);
    }

    // interpret first byte
    iD = (char)inByte[0];
    for (int i = 0; i <= 7; i++) {
      // use offset as m_ini starts at index 1
      m_in[i + 1] = bitRead(iD, i);
    }
    // interpret second byte
    iD = (char)inByte[1];
    for (int i = 0; i <= 7; i++) {
      // use offset as m_ini starts at index 1, +8 for extension
      m_in[i + 9] = bitRead(iD, i);
    }
  }
}

//---------------------------------------------------------------------------
// Set 8 digital outputs
void FTlegacy::setOutputs() {

  // check whether change has occurred
  bool change = false;
  for (int i = 0; i <= 16; i++) {
    if (m_prev[i] != m_out[i]) {
      change = true;
    };
  }

  // update previous array
  if (change) {
    // update previous array
    for (int i = 0; i <= 16; i++) {
      m_prev[i] = m_out[i];
    }

    if (type == PAR || type == PAREX) {

      digitalWrite(m_pin[LOADOUT], LOW);

      // Shift outputs right to left
      for (int j = numUnits; j > 0; j--) {
        for (int i = 8; i > 0; i--) {
          digitalWrite(m_pin[CLOCK], LOW);
          digitalWrite(m_pin[DATAOUT], m_out[i + (j - 1) * 8]);
          digitalWrite(m_pin[CLOCK], HIGH);
        }
      }

      // Enable strobe, then disable it again so that the outputs don't change
      // if we generate clock pulses to read the inputs.
      digitalWrite(m_pin[LOADOUT], HIGH);
      digitalWrite(m_pin[LOADOUT], LOW);
      if (ftInfo > 1) {printOutputBuffer();}

    } else if (type == SER) {

      outByte[0] = 0xC1;
      outByte[1] = 0b00000000;
      // convert m_pin array to a byte
      for (int i = 0; i <= 7; i++) {
        if (m_out[i + 1] == 1) {
          outByte[1] |= 1 << i;
        }
      }

      if (board == MEGA) {
        readWriteMEGA(number, 2);
      } else if (board == UNO) {
        readWriteUNO(2);  // this has to be a read/write, do nothing with the read part
      }
    } else if (type == SEREX) {

      outByte[0] = 0xC2;
      outByte[1] = 0b00000000;
      // convert m_pin array to a byte
      for (int i = 0; i <= 7; i++) {
        if (m_out[i + 1] == 1) {
          outByte[1] |= 1 << i;
        }
      }
      outByte[2] = 0b00000000;
      // convert m_pin array to a byte
      for (int i = 0; i <= 7; i++) {
        if (m_out[i + 9] == 1) {
          outByte[2] |= 1 << i;
        }
      }

      if (board == MEGA) {
        readWriteMEGA(number, 3);
      } else if (board == UNO) {
        readWriteUNO(3);  // this has to be a read/write, do nothing with the read part
      }
    }
  }
}

//---------------------------------------------------------------------------
// Get analog input from parallel interface
long int FTlegacy::getParallelAnalog(int xory) {  // 0=x, 1=Y
  long int tBegin = 0;
  long int tEnd = 0;
  int pinTrigger;

  // trigger clock to avoid timeouts when asking for analog data only
  digitalWrite(m_pin[CLOCK],LOW);
  digitalWrite(m_pin[CLOCK],HIGH);

  pinTrigger = m_pin[xory ? TRIGGERY : TRIGGERX];
  tBegin = micros();
  digitalWrite(pinTrigger, LOW);
  digitalWrite(pinTrigger, HIGH);

  while (!digitalRead(m_pin[DATACOUNTIN])) {}
  tEnd = micros() - tBegin;

  return (tEnd / 10);
}

//---------------------------------------------------------------------------
// Get 2 analog inputs Ex and Ey
void FTlegacy::getAnalogInputs() {

  int iAx = 33;
  int iAy = 33;

  long int cnt_X = 0;
  long int cnt_Y = 0;

  if (type == PAR || type == PAREX) {

    m_ana[0] = (int)getParallelAnalog(0);
    m_ana[1] = (int)getParallelAnalog(1);

    // m_ana[0] = map(analogRead(pinParEx), 0, 1023, 0, 255);
    // m_ana[1] = map(analogRead(pinParEy), 0, 1023, 0, 255);

  } else if (type == SER || type == SEREX) {

    iAx = getSerialAnalog(1);
    iAy = getSerialAnalog(2);

    if (iAx != iAy) {
      m_ana[0] = map(iAx, 11, 988, 255, 0);
      m_ana[1] = map(iAy, 14, 1024, 255, 0);
    }
  }
}
//---------------------------------------------------------------------------
// check whether there are analog sensors givibg a signal by checking whether
// the signal is random by sampling repeatedly and checking for spread
int FTlegacy::connectedAnalog() {

  int tempX = 0;
  int tempMinX = 2550;
  int tempMaxX = 0;
  int tempY = 0;
  int tempMinY = 2550;
  int tempMaxY = 0;
  int codeX = 0;
  int codeY = 0;

  for (int i = 1; i <= 2; i++) {  // cleanup
    tempX = getParallelAnalog(0);
    tempY = getParallelAnalog(1);
  }

  for (int i = 1; i <= 5; i++) {

    tempX = getParallelAnalog(0);
    // Serial.println(tempX);
    if (tempX > tempMaxX) {
      tempMaxX = tempX;
    } else if (tempX < tempMinX) {
      tempMinX = tempX;
    }

    tempY = getParallelAnalog(1);
    // Serial.println(tempY);
    if (tempY > tempMaxY) {
      tempMaxY = tempY;
    } else if (tempY < tempMinY) {
      tempMinY = tempY;
    }
  }

  if ((tempMaxX - tempMinX) > 10) {
    // random analog data Ex
    codeX = 1;
      if (ftInfo > 0) { Serial.println(F("Ex issue")); }
  }
  if ((tempMaxY - tempMinY) > 10) {
      if (ftInfo > 0) { Serial.println(F("Ey issue")); }
    codeY = 2;
  }

  return (codeX + codeY);
}

//---------------------------------------------------------------------------
// Get analog input from a serial (intelligent) interface
int FTlegacy::getSerialAnalog(int xory) {
  // 1 for X & 2 for Y

  int value = 0;
  int n_input = 0;

  if (type == SER) {
    if (xory == 1) {
      outByte[0] = 0xC5;  // Ex
    } else if (xory == 2) {
      outByte[0] = 0xC9;  // Ey
    } else {
      outByte[0] = 0xC1;  // No analog input
    }

    if (board == MEGA) {
      readWriteMEGA(number, 2);
    } else if (board == UNO) {
      readWriteUNO(2);
    }

    value = ftDecodeAnalog(xory, 1);  // firstByte at index 1

  } else if (type == SEREX) {
    if (xory == 1) {
      outByte[0] = 0xC6;  // Ex
    } else if (xory == 2) {
      outByte[0] = 0xCA;  // Ey
    } else {
      outByte[0] = 0xC2;  // No analog input
    }

    if (board == MEGA) {
      readWriteMEGA(number, 3);
    } else if (board == UNO) {
      readWriteUNO(3);
    }

    value = ftDecodeAnalog(xory, 2);  // firstByte at index 2
  }

  delay(5);
  return (value);
}

//---------------------------------------------------------------------------
// decode the input bytes to retrieve analog (Ex, EY) data
int FTlegacy::ftDecodeAnalog(int xory, int firstByte) {

  union A_tag {
    byte b[2];
    int ival;
  } Ax, Ay;

  int value = 0;

  if (xory == 1) {
    Ax.b[0] = inByte[firstByte + 1];
    Ax.b[1] = inByte[firstByte];

    if (Ax.ival == pAy) {
      //  value = pAx;
    } else {
      value = Ax.ival;
    }
    pAx = value;

  } else if (xory == 2) {
    Ay.b[0] = inByte[firstByte + 1];
    Ay.b[1] = inByte[firstByte];

    if (Ay.ival == pAx) {
      //  value = pAy;
    } else {
      value = Ay.ival;
    }
    pAy = value;

  } else {
    value = 0;
  }
  // delay(10);
  return (value);
}


//---------------------------------------------------------------------------
// Print the 8bit input buffer to the monitor
void FTlegacy::printInputBuffer() {

  int max = 8;
  Serial.print(F("IN: "));
  Serial.print(number);
  Serial.print(F(" signal: "));

  if (type == SEREX || type == PAREX) {
    max = 16;
  }
  for (int i = 1; i <= max; i++) {
    Serial.print(m_in[i]);
    Serial.print(' ');
  }
  Serial.print(F(" Analog x: "));
  Serial.print(m_ana[0]);
  Serial.print(F(" Analog y: "));
  Serial.println(m_ana[1]);
}

//---------------------------------------------------------------------------
// Print the 8bit output buffer to the monitor
void FTlegacy::printOutputBuffer() {

  int max = 8;
  Serial.print(F("OUT: "));
  Serial.print(number);
  Serial.print(F(" signal: "));

  if (type == SEREX) {
    max = 16;
  }
  for (int i = 1; i <= 16; i++) {
    Serial.print(m_out[i]);
    Serial.print(' ');
  }
  Serial.println();
}

//---------------------------------------------------------------------------
// Provide the analog X value from the buffer
int FTlegacy::getAnalogX() {

  return m_ana[0];
}

//---------------------------------------------------------------------------
// Provide the analog Y value from the buffer
int FTlegacy::getAnalogY() {
  return m_ana[1];
}

//---------------------------------------------------------------------------
// Returns the state of switch E-pin from the buffer
bool FTlegacy::getInput(int pin) {
  return (m_in[pin] == 1);
}

//---------------------------------------------------------------------------
// Returns the state of two E-pins
bool FTlegacy::getInput2(int pin1, int pin2) {
  return (m_in[pin1] == 1 && m_in[pin2] == 1);
}

//---------------------------------------------------------------------------
// Set all digital input pins to zero
void FTlegacy::zeroInput() {
  for (int i = 1; i <= 16; i++) {
    m_in[i] = 0;
  }
}

//---------------------------------------------------------------------------
//  test whether switch E is met Whilt motor M is on
bool FTlegacy::getMotorUntil(int M, int E, bool until, motorDirection dir) {

  getInputs();
  if (getInput(E) == until) {
    setMotorSTOP(M);
    return (false);
  } else {
    setMotor(M, dir);
    return (true);
  }
}

//---------------------------------------------------------------------------
//  switch on actuator M until condition (ON/OFF) on switch E is met
bool FTlegacy::setMotorUntil(int M, int E, bool until, motorDirection dir) {

  bool go_on = true;

  while (go_on) {
    getInputs();
    if (getInput(E) == until) {
      setMotorSTOP(M);
      // Serial.println("STOP");
      go_on = false;

    } else {
      setMotor(M, dir);
      // Serial.println("GO");
    }
  }
  return (go_on);
}

bool FTlegacy::setMotorUntilCount(int M, int E, bool until, motorDirection dir, int maxCount) {

  bool go_on = true;

  go_on = setMotorUntilOrCount(M, E, until, 0, false, dir, maxCount);
  return (go_on);
}

bool FTlegacy::setMotorUntilOrCount(int M, int E1, bool until1, int E2, bool until2, motorDirection dir, int maxCount) {

  bool go_on = true;
  int count = 0;
  int lastState = 0;
  int aState = 0;
  unsigned long startTimer = 0;

  startTimer = millis();
  getInputs();
  lastState = getInput(E1);
  setMotor(M, dir);
  while ((count < maxCount) && go_on) {
    getInputs();
    aState = getInput(E1);
    if (aState != lastState) {
      count++;
        // Serial.println(count);
      lastState = aState;
    }
    if (E2 > 0) {
      if (getInput(E2) == until2) {
        go_on = false;
      }
    }
    if ((millis() - startTimer) > 20000) {
      go_on = false;
      Serial.println("timer");
    }
  }
  setMotorSTOP(M);
  return (false);
}

//---------------------------------------------------------------------------
//  switch on actuator M until condition (ON/OFF) on switch E is met
bool FTlegacy::getOutputUntil(int O, int E, bool until) {

  getInputs();
  if (getInput(E) == until) {
    setOutputOFF(O);
    return (false);
  } else {
    setOutputON(O);
    return (true);
  }
}

//---------------------------------------------------------------------------
//  switch on actuator M until condition (ON/OFF) on switch E is met
bool FTlegacy::setOutputUntil(int O, int E, bool until) {

  bool go_on = true;

  while (go_on) {
    getInputs();
    if (getInput(E) == until) {
      setOutputOFF(O);
      go_on = false;
    } else {
      setOutputON(O);
    }
  }
  return (go_on);
}


bool FTlegacy::setOutputUntilCount(int O, int E, bool until, int maxCount) {

  bool go_on = true;
  go_on = setOutputUntilOrCount(O, E, until, 0, false, maxCount);
  return (go_on);
}

bool FTlegacy::setOutputUntilOrCount(int O, int E1, bool until1, int E2, bool until2, int maxCount) {

  bool go_on = true;
  int count = 0;
  int lastState = 0;
  int aState = 0;
  unsigned long startTimer = 0;

  startTimer = millis();
  getInputs();
  lastState = getInput(E1);
  setOutputON(O);
  while ((count < maxCount) && go_on) {
    getInputs();
    aState = getInput(E1);
    if (aState != lastState) {
        count++;
        lastState = aState;
    }
    if (E2 > 0) {
      if (getInput(E2) == until2) {
        go_on = false;
      }
    }
    if ((millis() - startTimer) > 10000) {
      go_on = false;
      Serial.println("timer");
    }
  }
  setOutputOFF(O);
  return (false);
}
//---------------------------------------------------------------------------
//  switch on Output O (assume connected between Mi and GND)
void FTlegacy::setOutputON(int O) {
  setOutput(O, 1);
}

//---------------------------------------------------------------------------
//  switch off Output O (assume connected between Mi and GND)
void FTlegacy::setOutputOFF(int O) {
  setOutput(O, 0);
}

//---------------------------------------------------------------------------
// control Lamp L (assume connected between Mi and GND) with status
// NOTE: the two lamps on one motor output can never be on at the same time
// this is probably nonsense, all lights should be on at the same time.
void FTlegacy::setOutput(int O, int status) {
  if (m_prev[O] == status) {  // do no change, do nothing}
  } else {
    m_out[O] = status;
  }
  setOutputs();
}

//---------------------------------------------------------------------------
// Give instruction (CW, CCW, STOP) to Motor 1-4
void FTlegacy::setMotor(int M, motorDirection D) {

  switch (D) {

    case CCW:
      setMotorCCW(M);
      break;

    case CW:
      setMotorCW(M);
      break;

    case STOP:
      setMotorSTOP(M);
      break;
  }
}

//---------------------------------------------------------------------------
//  rotate Motor M to clockwise
void FTlegacy::setMotorCW(int M) {
  if (m_prev[mIndex1(M)] == 1 && m_prev[mIndex2(M)] == 0) {  // no change, do nothing
  } else {
    m_out[mIndex1(M)] = 1;
    m_out[mIndex2(M)] = 0;
    setOutputs();
  }
}

//---------------------------------------------------------------------------
//  rotate Motor M to counter clockwise
void FTlegacy::setMotorCCW(int M) {
  if (m_prev[mIndex1(M)] == 0 && m_prev[mIndex2(M)] == 1) {  // no change, do nothing
  } else {
    m_out[mIndex1(M)] = 0;
    m_out[mIndex2(M)] = 1;
    setOutputs();
  }
}

//---------------------------------------------------------------------------
//  stop Motor M
void FTlegacy::setMotorSTOP(int M) {
  if (m_prev[mIndex1(M)] == 0 && m_prev[mIndex2(M)] == 0) {  // no change, do nothing
  } else {
    m_out[mIndex1(M)] = 0;
    m_out[mIndex2(M)] = 0;
    setOutputs();
  }
}

//---------------------------------------------------------------------------
// return index1 for motor M in output buffer
int mIndex1(int m) {
  return (2 * (m - 1) + 1);
}

// return index2 for motor M in output buffer
int mIndex2(int m) {
  return (2 * (m - 1) + 2);
}

//---------------------------------------------------------------------------
// return the state of Motor M
motorDirection FTlegacy::getMotor(int M) {
  motorDirection dir;

  if (m_out[mIndex1(M)] == 0 && m_out[mIndex2(M)] == 0) {
    dir = STOP;
  } else if (m_out[mIndex1(M)] == 0 && m_out[mIndex2(M)] == 1) {
    dir = CCW;
  } else if (m_out[mIndex1(M)] == 1 && m_out[mIndex2(M)] == 0) {
    dir = CW;
  }

  return dir;
}

//---------------------------------------------------------------------------
// return the state of Motor M
char FTlegacy::displayMotor(int M) {

  char mCode;

  if (m_out[mIndex1(M)] == 0 && m_out[mIndex2(M)] == 0) {
    mCode = '0';
  } else if (m_out[mIndex1(M)] == 0 && m_out[mIndex2(M)] == 1) {
    mCode = 'L';
  } else if (m_out[mIndex1(M)] == 1 && m_out[mIndex2(M)] == 0) {
    mCode = 'R';
  }
  return (mCode);
}

//---------------------------------------------------------------------------
// switch magnet M on
void FTlegacy::magnetON(int M) {
  setMotorCW(M);
}

//---------------------------------------------------------------------------
// switch magnet M off, do a cycle to avoid 'stickiness'
void FTlegacy::magnetOFF(int M) {
  setMotorCCW(M);
  setMotorSTOP(M);
  setMotorCW(M);
  setMotorCCW(M);
  setMotorSTOP(M);
}
//---------------------------------------------------------------------------
// set all motors stop
void FTlegacy::setAllMotorsSTOP() {
  for (int i = 0; i < 17; i++) {
    m_out[i] = 0;
  }
  setOutputs();
}


//---------------------------------------------------------------------------
// read from serial interface from a MEGA
void FTlegacy::readWriteMEGA(int numSerial, int numBytes) {
#if defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA)

  int n_input = 0;
  if (numSerial == 1) {
    n_written = Serial1.write(outByte, numBytes);
    delay(10);
    n_input = Serial1.available();
    for (int i = 0; i <= n_input; i++) {
      inByte[i] = Serial1.read();
    }
  } else if (numSerial == 2) {
    n_written = Serial2.write(outByte, numBytes);
    delay(10);
    n_input = Serial2.available();
    for (int i = 0; i <= n_input; i++) {
      inByte[i] = Serial2.read();
    }
  } else if (numSerial == 3) {
    n_written = Serial3.write(outByte, numBytes);
    delay(10);
    n_input = Serial3.available();
    for (int i = 0; i <= n_input; i++) {
      inByte[i] = Serial3.read();
    }
  } else if (numSerial == 4) {
    n_written = mySerial.write(outByte, numBytes);
    delay(10);
    n_input = mySerial.available();
    for (int i = 0; i <= n_input; i++) {
      inByte[i] = mySerial.read();
    }
  }
#endif
}

//---------------------------------------------------------------------------
// read from serial interface from a UNO
void FTlegacy::readWriteUNO(int numBytes) {
  int n_input = 0;
  n_written = mySerial.write(outByte, numBytes);
  delay(8);
  n_input = mySerial.available();
  // Serial.println(n_input);
  for (int i = 0; i <= n_input; i++) {
    inByte[i] = mySerial.read();
  }
}

//---------------------------------------------------------------------------
// write to serial interface from a MEGA
void FTlegacy::writeMEGA(int numSerial, int numBytes) {
#if defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA)
  if (numSerial == 1) {
    n_written = Serial1.write(outByte, numBytes);
  } else if (numSerial == 2) {
    n_written = Serial2.write(outByte, numBytes);
  } else if (numSerial == 3) {
    n_written = Serial3.write(outByte, numBytes);
  } else if (numSerial == 4) {
    n_written = mySerial.write(outByte, numBytes);
  }
#endif
}

//---------------------------------------------------------------------------
// write to serial interface from a UNO
void FTlegacy::writeUNO(int numBytes) {
  n_written = mySerial.write(outByte, numBytes);
}

//---------------------------------------------------------------------------
// set the desired information levele 0-3. May impact performance
void FTlegacy::setInfo(int level) {
  ftInfo = level;
}

//---------------------------------------------------------------------------
// update the display with the current status information
void FTlegacy::ftUpdateDisplay() {

  if (ftDisplayType == D1604 || ftDisplayType == D2004 || ftDisplayType == D1602) {
    if (initLCD) {
      previousMillis = millis();
      initLCD = false;
    }
    ftUpdateLCD(0);
  }
}

//---------------------------------------------------------------------------
// internal function to update the display with the current status information
void FTlegacy::ftUpdateLCD(int typeDisplay) {

  if (connectedToLCD) {
    unsigned long currentMillis = millis();
    unsigned long next;

    String ana_str[2] = { "Ax: ", "Ay: " };

    if ((currentMillis - previousMillis) >= lcdIntervalMillis) {
      firstLCD = false;
      previousMillis = currentMillis;
      next = lcdIntervalMillis / 2;
    }

    if ((currentMillis - previousMillis) >= next) {
      firstLCD = true;
      next = 100 * lcdIntervalMillis;
    }

    lcd.setCursor(0, 0);

    if (typeDisplay == 1) {

      // print motor settings
      for (int i = 1; i < 5; i++) {
        ftLCD_M(0, i - 1, i);
      }

      // print digital inputs
      for (int j = 1; j < 5; j++) {
        ftLCD_E(7, j - 1, j + 4 * (!firstLCD));
      }

      for (int k = 1; k < 3; k++) {

        lcd.setCursor(12, 0 + 2 * (k - 1));
        lcd.print("    ");
        lcd.setCursor(12, 0 + 2 * (k - 1));
        lcd.print(ana_str[k - 1]);
        lcd.setCursor(12, 2 * k - 1);
        lcd.print("    ");
        lcd.setCursor(12, 2 * k - 1);
        lcd.print(m_ana[k - 1]);
      }
    } else {

      // print name
      lcd.setCursor(0, 0);
      if (type == SEREX || type == PAREX) {
        lcd.print("E");
        for (int i = 9; i < 17; i++) {
          lcd.setCursor(i - 8, 0);
          lcd.print(m_in[i]);
        }
      } else {
        lcd.print("Status");
      }

      // print digital inpyt
      lcd.setCursor(0, 1);
      lcd.print("E");
      for (int i = 1; i < 9; i++) {
        lcd.setCursor(i, 1);
        lcd.print(m_in[i]);
      }

      // print motor settings
      for (int i = 1; i < 5; i++) {
        ftLCD_M(10, i - 1, i);
      }

      if (type == SEREX || type == PAREX) {
        // print motor settings
        for (int i = 1; i < 5; i++) {
          lcd.setCursor(17, i - 1);
          switch (getMotor(i + 4)) {
            case CCW: lcd.print("CCW"); break;
            case CW: lcd.print("CW "); break;
            case STOP: lcd.print("STP"); break;
          }
        }
      }

      for (int k = 1; k < 3; k++) {
        int z_off = 0;  // lcd_offset;
        lcd.setCursor(z_off, k + 1);
        lcd.print("    ");
        lcd.setCursor(z_off, k + 1);
        lcd.print(ana_str[k - 1]);
        lcd.setCursor(3 + z_off, k + 1);
        lcd.print("    ");
        lcd.setCursor(3 + z_off, k + 1);
        lcd.print(m_ana[k - 1]);
      }
    }
  }
}

//---------------------------------------------------------------------------
// Write the motor status information
void FTlegacy::ftLCD_M(int x, int y, int M) {

  lcd.setCursor(x, y);
  lcd.print("M");
  lcd.print(M);
  lcd.print(":");
  lcd.setCursor(x + 3, y);
  switch (getMotor(M)) {
    case CCW: lcd.print("CCW"); break;
    case CW: lcd.print("CW "); break;
    case STOP: lcd.print("STP"); break;
  }
}

//---------------------------------------------------------------------------
// Write the digital input status information
void FTlegacy::ftLCD_E(int x, int y, int E) {

  lcd.setCursor(x, y);
  lcd.print("S");
  lcd.print(E);
  lcd.print(":");
  lcd.setCursor(x + 3, y);
  lcd.print(getInput(E));
}

//---------------------------------------------------------------------------
// debug code to allow serial prints from within libraries
void FTcontroller::begin(char* message) {

  int offset;

  Serial.println(F("Controller begin"));
  if (board == UNO) {  // Arduino UNO works with Software Serial

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_UNOR4_MINIMA)
    mySerial.begin(9600);
#endif

  } else if (board == MEGA) {  // Arduino MEGA works with hardware Serial1-3 and Serial
                               // SoftwareSerial mySerial(10,11);
#if defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA)
    mySerial.begin(9600);  // make sure this is mySerial
    Serial1.begin(9600);
    Serial2.begin(9600);
    Serial3.begin(9600);
#endif
  } else if (board == ESP32) {

#if defined(ARDUINO_ARCH_ESP32)
    Serial1.begin(9600, SERIAL_8N1, 27, 26);  // RX1, TX1
    Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX2, TX2
#endif
  }

  if (ftDisplayType == D1604) {
    numLCDcolumns = 16;
    numLCDrows = 4;
  } else if (ftDisplayType == D1602) {
    numLCDcolumns = 16;
    numLCDrows = 2;
  } else {
    numLCDcolumns = 20;
    numLCDrows = 4;
  }

  connectedToLCD = isLCDconnected();

  if ((ftDisplayType == D1604 || ftDisplayType == D2004) && connectedToLCD) {

    lcd.begin(numLCDcolumns, numLCDrows);
    // initialise LCD display
    lcd.init();
    lcd.backlight();
    lcd.clear();

    if (ftDisplayType == D2004) {
      offset = 2;
    } else {
      offset = 0;
    }
    // Print a message to the LCD.
    Serial.print(F("display : "));
    Serial.println(ftDisplayType);
    lcd.backlight();
    lcd.setCursor(4 + offset, 0);
    lcd.print("Arduino UNO");
    lcd.setCursor(2 + offset, 1);
    lcd.print(VERSION);
    lcd.setCursor(1 + offset, 2);
    lcd.print(message);
    lcd.setCursor(3 + offset, 3);
    lcd.print("JMMR 2023");
    delay(1000);
    lcd.clear();

  }

  else {
    Serial.println("No display ");
  }
}

//----------------------------------------------------------------------------
// add an interface to the list of interfaces
void FTcontroller::addInterface(FTlegacy ftLegacy) {

  ftLegacies[numInterfaces] = ftLegacy;
  numInterfaces++;
  if (ftLegacy.type == SER) numSer++;
  if (ftLegacy.type == PAR) numPar++;

  Serial.print(F("adding type: "));
  Serial.println(ftLegacy.type);

  if (FTcontroller::boardType == UNO) {

    if (FTcontroller::numPar == 1 || (FTcontroller::numSer == 1)) {
      Serial.println(F("Max boards"));
    }
  } else if (FTcontroller::boardType == MEGA) {

    if (numSer > 4 || numPar > 4) {
      Serial.println(F("Max boards"));
    }
  } else {
  }
  FTcontroller::numTot++;
  // Serial.println(F("End adding interface "));
}

//-----------------------------------------------------------------------------
// return the number of interfaces created
int FTcontroller::getNumberInterfaces() {
  return (numTot);
}

//-----------------------------------------------------------------------------
// return the tpye of Arduin board
int FTcontroller::getBoard() {
  return (board);
}

//-----------------------------------------------------------------------------
// checks presence of an I2C device, assume it is an LCD
bool FTcontroller::isLCDconnected() {

  bool onLCD = false;
  byte address, error;
  int numDevices = 0;

  Wire.begin();

  // loop over all I2C addresses to see whether an LCD is connected
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      onLCD = true;
      Serial.print(F("I2C device found at address 0x"));
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");
    }
    numDevices++;
  }
  return (onLCD);
}

void FTcontroller::ftMessageToDisplay(int x, int y, char* message, bool clear) {

  int offset;
  if (clear) {
       lcd.clear();
  }

  if (ftDisplayType == D2004) {
    offset = 2;
  } else {
    offset = 0;
  }
  if (connectedToLCD) {
    lcd.setCursor(x + offset, y);
    lcd.print(message);
  }
  Serial.println(message);
}

//
// timer class
FTtimer::FTtimer(unsigned long interval)
  : _interval(interval) {
  _start = millis();
}

bool FTtimer::ready() {
  return _start + _interval <= millis();
}

void FTtimer::interval(unsigned long interval) {
  _interval = interval;
}

void FTtimer::reset() {
  _start = millis();
}
int translate(int, int, int, int, float);

const int Ascii[94][9] = {
  { 33, 2975, 2473, 0, 0, 0, 0, 0, 0 },
  { 34, 3987, 1967, 0, 0, 0, 0, 0, 0 },
  { 35, 1369, 3983, 4757, 595, 0, 0, 0, 0 },
  { 36, 2379, 4788, 6857, 6686, 9584, 6455, 0, 0 },
  { 37, 9908, 5767, 6858, 4594, 8485, 9500, 0, 0 },
  { 38, 4573, 6354, 5587, 8879, 6958, 5793, 0, 0 },
  { 39, 2989, 8877, 0, 0, 0, 0, 0, 0 },
  { 40, 3977, 7583, 0, 0, 0, 0, 0, 0 },
  { 41, 1977, 7563, 0, 0, 0, 0, 0, 0 },
  { 42, 696, 3864, 1884, 0, 0, 0, 0, 0 },  //10
  { 43, 696, 2874, 0, 0, 0, 0, 0, 0 },
  { 44, 2484, 8372, 0, 0, 0, 0, 0, 0 },
  { 45, 696, 0, 0, 0, 0, 0, 0, 0 },
  { 46, 2374, 0, 0, 0, 0, 0, 0, 0 },
  { 47, 9900, 0, 0, 0, 0, 0, 0, 0 },
  { 48, 557, 6989, 9795, 8363, 5500, 0, 0, 0 },
  { 49, 3363, 2379, 6800, 0, 0, 0, 0, 0 },
  { 50, 869, 8998, 9754, 5393, 0, 0, 0, 0 },
  { 51, 869, 8998, 9786, 7636, 9594, 8363, 5400, 0 },
  { 52, 4555, 7939, 8300, 0, 0, 0, 0, 0 },  //20
  { 53, 463, 8394, 9687, 6756, 5999, 0, 0, 0 },
  { 54, 566, 8695, 9483, 6354, 5869, 8998, 0, 0 },
  { 55, 9959, 0, 0, 0, 0, 0, 0, 0 },
  { 56, 1657, 5869, 8998, 9786, 6655, 5463, 8394, 9586 },
  { 57, 463, 8394, 9889, 6958, 5766, 6697, 0, 0 },
  { 58, 2374, 2677, 0, 0, 0, 0, 0, 0 },
  { 59, 2675, 2473, 6200, 0, 0, 0, 0, 0 },
  { 60, 4956, 9300, 0, 0, 0, 0, 0, 0 },
  { 61, 595, 4757, 0, 0, 0, 0, 0, 0 },
  { 62, 996, 5300, 0, 0, 0, 0, 0, 0 },  //30
  { 63, 869, 8998, 9775, 2473, 0, 0, 0, 0 },
  { 64, 463, 8394, 8565, 5667, 8796, 8517, 5869, 8998 },
  { 65, 5679, 9693, 4555, 0, 0, 0, 0, 0 },
  { 66, 8394, 9586, 6636, 9798, 8959, 1963, 0, 0 },
  { 67, 4889, 6958, 5463, 8394, 0, 0, 0, 0 },
  { 68, 8394, 9889, 5919, 6300, 0, 0, 0, 0 },
  { 69, 5999, 3656, 393, 0, 0, 0, 0, 0 },
  { 70, 5999, 3656, 0, 0, 0, 0, 0, 0 },
  { 71, 4889, 6958, 393, 0, 0, 0, 0, 0 },
  { 72, 5906, 9649, 9300, 0, 0, 0, 0, 0 },  //40
  { 73, 1383, 2379, 1989, 0, 0, 0, 0, 0 },
  { 74, 463, 7384, 8929, 9900, 0, 0, 0, 0 },
  { 75, 5906, 9906, 9300, 0, 0, 0, 0, 0 },
  { 76, 953, 9300, 0, 0, 0, 0, 0, 0 },
  { 77, 5976, 9993, 0, 0, 0, 0, 0, 0 },
  { 78, 5993, 9900, 0, 0, 0, 0, 0, 0 },
  { 79, 458, 6989, 9894, 8363, 5400, 0, 0, 0 },
  { 80, 1369, 989, 9897, 8666, 0, 0, 0, 0 },
  { 81, 458, 6989, 9894, 8363, 5425, 9300, 0, 0 },
  { 82, 1369, 989, 9897, 8666, 2693, 0, 0, 0 },  //50
  { 83, 463, 8394, 9586, 6657, 5869, 8998, 0, 0 },
  { 84, 2379, 999, 0, 0, 0, 0, 0, 0 },
  { 85, 954, 6383, 9499, 0, 0, 0, 0, 0 },
  { 86, 973, 9900, 0, 0, 0, 0, 0, 0 },
  { 87, 956, 6375, 8396, 9900, 0, 0, 0, 0 },
  { 88, 993, 4953, 0, 0, 0, 0, 0, 0 },
  { 89, 976, 7326, 9900, 0, 0, 0, 0, 0 },
  { 90, 999, 5393, 0, 0, 0, 0, 0, 0 },
  { 91, 5679, 9693, 4555, 958, 4998, 0, 0, 0 },
  { 92, 457, 6888, 9794, 8363, 5409, 5849, 9800, 0 },  //60
  { 93, 954, 6383, 9499, 1968, 3988, 0, 0, 0 },
  { 94, 779, 9700, 0, 0, 0, 0, 0, 0 },
  { 95, 90, 0, 0, 0, 0, 0, 0, 0 },
  { 96, 3979, 7887, 0, 0, 0, 0, 0, 0 },
  { 97, 667, 8796, 9345, 6554, 6383, 9400, 0, 0 },
  { 98, 953, 667, 8796, 9483, 6354, 0, 0, 0 },
  { 99, 4687, 6756, 5463, 8394, 0, 0, 0, 0 },
  { 100, 4993, 4687, 6756, 5463, 8394, 0, 0, 0 },
  { 101, 585, 9687, 6756, 6463, 8394, 0, 0, 0 },
  { 102, 2378, 8999, 3666, 0, 0, 0, 0, 0 },  //70
  { 103, 261, 8192, 9687, 6756, 5463, 8394, 0, 0 },
  { 104, 5906, 6787, 9693, 0, 0, 0, 0, 0 },
  { 105, 3363, 2377, 6728, 7900, 0, 0, 0, 0 },
  { 106, 161, 7277, 6728, 7900, 0, 0, 0, 0 },
  { 107, 1963, 1597, 1593, 0, 0, 0, 0, 0 },
  { 108, 303, 3363, 2379, 6900, 0, 0, 0, 0 },
  { 109, 5787, 9693, 2377, 0, 0, 0, 0, 0 },
  { 110, 753, 667, 8796, 9300, 0, 0, 0, 0 },
  { 111, 456, 6787, 9694, 8363, 5400, 0, 0, 0 },
  { 112, 157, 667, 8796, 9483, 6354, 0, 0, 0 },  //80
  { 113, 4197, 4687, 6756, 5463, 8394, 0, 0, 0 },
  { 114, 5706, 6787, 9600, 0, 0, 0, 0, 0 },
  { 115, 463, 8394, 8565, 5667, 8796, 0, 0, 0 },
  { 116, 1686, 2973, 8300, 0, 0, 0, 0, 0 },
  { 117, 754, 6383, 9447, 9300, 0, 0, 0, 0 },
  { 118, 773, 9700, 0, 0, 0, 0, 0, 0 },
  { 119, 755, 6375, 8395, 9700, 0, 0, 0, 0 },
  { 120, 793, 397, 0, 0, 0, 0, 0, 0 },
  { 121, 773, 4761, 0, 0, 0, 0, 0, 0 },
  { 122, 797, 5393, 0, 0, 0, 0, 0, 0 },  //90
  { 123, 667, 8796, 6345, 6554, 6383, 9419, 6839, 8800 },
  { 124, 456, 6787, 9694, 8363, 5439, 8819, 6800, 0 },
  { 125, 3988, 1968, 754, 6383, 9447, 9300, 0, 0 },
  { 126, 5869, 8998, 9786, 7636, 9594, 8373, 6400, 0 }
};

const int numAscii = 94;

//---------------------------------------------------------------------------
// set a 'steps' stepmotor step
bool FTstepperXY::setStepXandY(int stepsX, int stepsY) {
  int coil1, coil2, coil3;
  bool abort = false;
  int xStep, yStep;
  int xCount = 0;
  int yCount = 0;
  int distX, distY, dist;

  // use coils A1, A2 and B2
  coil1 = coilA1;
  coil2 = coilA2;
  coil3 = coilB2;

  if ((currentY + stepsY > maxY) || (currentY + stepsY < 0) || (currentX + stepsX > maxX) || (currentX + stepsX < 0)) {
    abort = true;
    Serial.println(F(" coordinates out of bounds"));
  }

  xStep = stepsX;
  yStep = stepsY;

  if (!abort) {
    xStep = stepsX;
    yStep = stepsY;
    distX = abs(xStep);
    distY = abs(yStep);
    dist = 0;

    while (xCount != distX || yCount != distY) {

      //Serial.print("in set StepsXandY ");
      //Serial.print(xStep);
      //Serial.print(" ");
      //Serial.println(yStep);

      if (abs(dist + distY) < abs(dist + distY - distX)) {  // step in x-direction
        if (xStep > 0) {                                    // +X direction
          stepper(CCW, CW, CW);
          stepper(CCW, CCW, CW);
          stepper(CW, CCW, CW);
          stepper(CW, CW, CW);
        } else if (xStep < 0) {  // -X direction
          stepper(CW, CCW, CW);
          stepper(CCW, CCW, CW);
          stepper(CCW, CW, CW);
          stepper(CW, CW, CW);
        }
        xCount += 1;
        dist += distY;
      } else if (abs(dist - distX) < abs(dist + distY - distX)) {  // step in y-direction
        if (yStep > 0) {                                           // +Y direction
          stepper(CCW, CW, CW);
          stepper(CCW, CW, CCW);
          stepper(CW, CW, CCW);
          stepper(CW, CW, CW);
        } else if (yStep < 0) {  // -Y direction
          stepper(CW, CW, CCW);
          stepper(CCW, CW, CCW);
          stepper(CCW, CW, CW);
          stepper(CW, CW, CW);
        }
        yCount += 1;
        dist -= distX;
      } else {
        if (xStep > 0 && yStep > 0) {  // +X,+Y direction
          stepper(CCW, CW, CW);
          stepper(CCW, CCW, CCW);
          stepper(CW, CCW, CCW);
          stepper(CW, CW, CW);
        } else if (xStep > 0 && yStep < 0) {  // +X,-Y direction
          stepper(CW, CW, CCW);
          stepper(CCW, CW, CCW);
          stepper(CCW, CCW, CW);
          stepper(CW, CCW, CW);
          stepper(CW, CW, CW);
        } else if (xStep < 0 && yStep > 0) {  // -X,+Y direction
          stepper(CW, CCW, CW);
          stepper(CCW, CCW, CW);
          stepper(CCW, CW, CCW);
          stepper(CW, CW, CCW);
          stepper(CW, CW, CW);
        } else if (xStep < 0 && yStep < 0) {  // -X,-Y direction
          stepper(CW, CCW, CCW);
          stepper(CW, CCW, CCW);
          stepper(CCW, CCW, CCW);
          stepper(CCW, CW, CW);
          stepper(CW, CW, CW);
        }
        xCount += 1;
        yCount += 1;
        dist = dist + distY - distX;
      }
    }
  }
  return (!abort);
}

void FTstepperXY::stepper(motorDirection C1, motorDirection C2, motorDirection C3) {

  int coil1 = coilA1;
  int coil2 = coilA2;
  int coil3 = coilB2;

  if (C1 == CW) {
    interface.setMotorCW(coil1);
  } else {
    interface.setMotorCCW(coil1);
  }
  if (C2 == CW) {
    interface.setMotorCW(coil2);
  } else {
    interface.setMotorCCW(coil2);
  }
  if (C3 == CW) {
    interface.setMotorCW(coil3);
  } else {
    interface.setMotorCCW(coil3);
  }
  delay(5);
}

void FTstepperXY::setStepX(int steps) {
  if (setStepXandY(steps, 0)) {
    currentX += steps;
  };
}

void FTstepperXY::setStepY(int steps) {
  if (setStepXandY(0, steps)) {
    currentY += steps;
  };
}

void FTstepperXY::setStepXY(int stepsX, int stepsY) {
  if (setStepXandY(stepsX, stepsY)) {
    currentX += stepsX;
    currentY += stepsY;
  }
}

//---------------------------------------------------------------------------
// set steppermotors stop
void FTstepperXY::setStepperSTOP() {
  interface.setMotorSTOP(coilA1);
  interface.setMotorSTOP(coilA2);
  interface.setMotorSTOP(coilB1);
  interface.setMotorSTOP(coilA2);
}

//---------------------------------------------------------------------------
// set the origin for this pair of stepmotors
void FTstepperXY::setOrigin(int origX, int origY) {
  originX = origX;
  originY = origY;
}

//---------------------------------------------------------------------------
// find the physical origin determined by the end switches
bool FTstepperXY::findOrigin(int stopX, int stopY) {

  // circumvent setStepXandY becaue the origin has not yet been set
  // and out of bounds check will fail
  interface.getInputs();
  // find X origin
  while (interface.getInput(stopX)) {
    stepper(CW, CCW, CW);
    stepper(CCW, CCW, CW);
    stepper(CCW, CW, CW);
    stepper(CW, CW, CW);
    interface.getInputs();
  }
  // find Y origin
  while (interface.getInput(stopY)) {
    stepper(CW, CW, CCW);
    stepper(CCW, CW, CCW);
    stepper(CCW, CW, CW);
    stepper(CW, CW, CW);
    interface.getInputs();
  }
  setStepXY(3, 3);
  setPosition(0, 0);
}

//---------------------------------------------------------------------------
// set the position for this pair of stepmotors
void FTstepperXY::setPosition(int posX, int posY) {
  currentX = posX;
  currentY = posY;
}

//---------------------------------------------------------------------------
// set the origin for this pair of stepmotors
void FTstepperXY::setArea(int origX, int origY, int mX, int mY) {
  originX = origX;
  originY = origY;
  maxX = mX;
  maxY = mY;
}

//---------------------------------------------------------------------------
// move to a positon posX, posY
void FTstepperXY::moveToPosition(int posX, int posY) {
  //Serial.print(" moveX :");
  //Serial.print(posX);
  //Serial.print(" moveY :");
  //Serial.println(posY);
  setStepXY(posX - currentX, posY - currentY);
}

//---------------------------------------------------------------------------
// move to a positon posX, posY
void FTstepperXY::moveRelative(int deltaX, int deltaY) {
  setStepXY(deltaX, deltaY);
}

//---------------------------------------------------------------------------
// move pen down
void FTstepperXY::penDown() {
  // Serial.println("pen down");
  interface.magnetON(actuator);
}

//---------------------------------------------------------------------------
// move pen up
void FTstepperXY::penUp() {
  // Serial.println("pen up");
  interface.magnetOFF(actuator);
}

void FTstepperXY::line(int posX, int posY) {
  penDown();
  moveToPosition(posX, posY);
  penUp();
}

void FTstepperXY::lineRelative(int deltaX, int deltaY) {
  penDown();
  moveRelative(deltaX, deltaY);
  penUp();
}
//---------------------------------------------------------------------------
// move to a positon posX, posY
void FTstepperXY::circle(int origX, int origY, int radius) {
  ellips(origX, origY, radius, radius);
}

//---------------------------------------------------------------------------
// draw a circle with radius R
void FTstepperXY::ellips(int origX, int origY, int radiusX, int radiusY) {

  float theta, increment, x, y;
  int numPoints = 720;

  theta = 0;
  increment = 2 * PI / numPoints;
  moveToPosition(origX + radiusX, origY);

  penDown();
  for (int i = 0; i <= numPoints; i++) {
    x = origX + radiusX * cos(theta);
    y = origY + radiusY * sin(theta);
    moveToPosition(int(x), int(y));
    theta += increment;
  }
  penUp();
}

void FTstepperXY::box(int posX, int posY, int sizeX, int sizeY, int hatch) {

  moveToPosition(posX, posY);
  penDown();
  lineRelative(sizeX, 0);
  lineRelative(0, sizeY);
  lineRelative(-sizeX, 0);
  lineRelative(0, -sizeY);
  penUp();
}

void FTstepperXY::axis(int xPB, int xPE, int yPB, int yPE,  // Plot coordinate range B=begin
                       int xAB, int xAE, int yAB, int yAE,  // Axis coordinate range E=end
                       bool label, char* xLabel, char* yLabel, char* title,
                       int xInterval, int yInterval, int ticklength) {

  float xScale, yScale;
  char buffer[16];

  box(xAB, yAB, (xAE - xAB), (yAE - yAB), 0);

  // x-axis annotation
  if (xInterval > 1) {
    for (int i = 0; i <= xInterval; i++) {
      xScale = i * float((100 * xPE - 100 * xPB) / xInterval);
      moveToPosition(translate(xPB, xPE, xAB, xAE, xScale / 100.), yAB);
      lineRelative(0, -ticklength);
      sprintf(buffer, "%d", int(xPB+xScale/100));
      Serial.println(buffer);
      plotText(translate(xPB, xPE, xAB, xAE, xScale/100.)-10, yAB -30, 2, 0, buffer);
    }
  }

  // y-axis annotation
  if (yInterval > 1) {
    for (int i = 0; i <= yInterval; i++) {
      yScale = i * float((100 * yPE - 100 * yPB) / yInterval);
      moveToPosition(xAB, translate(yPB, yPE, yAB, yAE, yScale / 100.));
      lineRelative(-ticklength, 0);
      sprintf(buffer, "%d", int(yPB+yScale/100));
      Serial.println(buffer);
      plotText(xAB-15, translate(yPB, yPE, yAB, yAE, yScale/100.)-10, 2, 1, buffer);
    }
  }
  if (label) {
    plotText(xAB+(xAE-xAB)/3, yAB-50, 2, 0, xLabel);
    plotText(xAB-40, yAB+(yAE-yAB)/3, 2, 1, yLabel);
    plotText(xAB+(xAE-xAB)/3, yAE+20, 3, 0, title);
  }
}

int translate(int PB, int PE, int AB, int AE, float x) {
  return (AB + float((x - PB) / (PE - PB)) * (AE - AB));
}

void FTstepperXY::curve(int numberOfPoints, int curveX[], int curveY[]) {
  moveToPosition(curveX[0], curveY[0]);
  for (int i = 0; i < numberOfPoints; i++) {
    line(curveX[i], curveY[i]);
  }
}


void FTstepperXY::plotText(int xPos, int yPos, int scale, int direction, char* textString) {
  int factorX, factorY;
  float angle;

  angle = direction * HALF_PI;
  for (int i = 0; i < strlen(textString); i++) {
    factorX = 5 * scale * int(cos(angle));
    factorY = 5 * scale * int(sin(angle));
    plotChar(xPos + i * factorX, yPos + i * factorY, scale, direction, textString[i]);
  }
}

void FTstepperXY::plotChar(int xPos, int yPos, int scale, int direction, char c) {

  int cIndex = int(c);
  int a;
  int xFrom, yFrom, xTo, yTo, xPrev, yPrev;
  float angle;

  xPrev = 0;
  yPrev = 3;
  penUp();
  moveToPosition(xPos + xPrev, yPos + yPrev);

  angle = direction * HALF_PI;

  for (int i = 0; i < numAscii; i++) {
    if (cIndex == Ascii[i][0]) {
      penUp();
      for (int j = 1; j < 9; j++) {
        a = Ascii[i][j];
        angle = direction * HALF_PI;
        if (a > 0) {
          xFrom = int(a / 1000);
          yFrom = int(a / 100 - 10 * xFrom);
          xTo = int(100 * (a - 1000 * xFrom - 100 * yFrom) / 1000);
          yTo = int(100 * (a - 1000 * xFrom - 100 * yFrom) / 100 - 10 * xTo);

          drawSegment(xPrev, xFrom, yPrev, yFrom, scale, angle);
          drawSegment(xPrev, xTo, yPrev, yTo, scale, angle);

        } else {
          penUp();
        }
      }
      break;
    }
  }
}

void FTstepperXY::drawSegment(int& X1, int X2, int& Y1, int Y2, int scale, float angle) {

  int dX, dX0, dY, dY0;

  if (X2 > 4) {
    X2 = X2 - 5;  // pen is down
    penDown();
  } else {
    penUp();
  }

  dX0 = scale * (X2 - X1);
  dY0 = scale * (Y2 - Y1);

  dX = dX0 * int(cos(angle)) - dY0 * int(sin(angle));
  dY = dX0 * int(sin(angle)) + dY0 * int(cos(angle));

  moveRelative(dX, dY);
  X1 = X2;
  Y1 = Y2;
}

//
// STEPPER class for single stepper motors

//---------------------------------------------------------------------------
// set a 'steps' stepmotor step
void FTstepper::setStep(int steps) {

  Serial.println("in set Steps");

  if (steps > 0) {
    for (int i = 0; i < steps; i++) {
      interface.setMotorCCW(coilA);
      //   delay(5);
      interface.setMotorCCW(coilB);
      //   delay(5);
      interface.setMotorCW(coilA);
      //    delay(5);
      interface.setMotorCW(coilB);
      //    delay(5);
    }
  } else if (steps < 0) {
    for (int i = 0; i < -steps; i++) {
      interface.setMotorCCW(coilB);
      //   delay(5);
      interface.setMotorCCW(coilA);
      //   delay(5);
      interface.setMotorCW(coilB);
      //   delay(5);
      interface.setMotorCW(coilA);
      //   delay(5);
    }
  }
}

//---------------------------------------------------------------------------
// set the origin for this stepmotor
void FTstepper::setOrigin(int newOrigin) {
  origin = newOrigin;
}

//---------------------------------------------------------------------------
// set the range for this stepmotor
void FTstepper::setRange(int newMinimum, int newMaximum) {
  minimum = newMinimum;
  maximum = newMaximum;
}

//---------------------------------------------------------------------------
// move to the origin
void FTstepper::moveToOrigin() {
  moveToPosition(origin);
}

//---------------------------------------------------------------------------
// move to a positon
void FTstepper::moveToPosition(int target) {
  int steps;

  steps = target - currentPosition;
  moveRelative(steps);
}

//---------------------------------------------------------------------------
// move a relative number of steps
void FTstepper::moveRelative(int delta) {
  int target;

  if (abs(delta) > 0) {
    target = currentPosition + delta;
    if ((target > minimum) && (target < maximum)) {
      setStep(delta);
      currentPosition += delta;
    }
  }
}

//---------------------------------------------------------------------------
// move a relative number of steps
void FTstepper::setStepperSTOP() {
  interface.setMotorSTOP(coilA);
  interface.setMotorSTOP(coilB);
}

//
// ENCODER class for single encoder motors

int numberOfEncoderMotors = 0;

// the constructor
FTencoderMotor* FTencoderMotor::sEncoder = 0;

FTencoderMotor::FTencoderMotor(FTlegacy* choice, int motorID, FTencoderMode modeChoice, int sensorID) {
  sEncoder = this;
  interface = choice;
  motor = motorID;
  sensor = sensorID;
  encoderSignal = sensorID;
  encoderMode = modeChoice;  // mode=INT: Arduino pin interrupt, mode=STD: ft pin / no interrupt
  origin = 0;
    
  if (modeChoice==E_INT) {
     attachInterrupt(digitalPinToInterrupt(sensorID), FTencoderMotor::updateEncoderISR, CHANGE);
  }
};

void FTencoderMotor::updateEncoderISR() {
  if (sEncoder != 0)
    sEncoder->updateEncoder();
}

void FTencoderMotor::begin() {

  pinMode(encoderSignal, INPUT);
  setMotorSTOP();

  origin = 0;
  maximum = 1E10;
  encoderPos = 0;

  Serial.print(" End Encoder begin...with motor: ");
  Serial.println(motor);
}

void FTencoderMotor::updateEncoder() {
  // Serial.print(" update Encoder begin..");
  if (movingRight) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}
//---------------------------------------------------------------------------
// set 'steps' encoder motor steps
bool FTencoderMotor::setSteps(int steps) {

  bool result = false;
  if (encoderMode == E_INT) {  // digital interrupt
    result = setStepsNEW(steps);
  } else if (encoderMode == E_STD) {
    result = setStepsOLD(steps);
  }
  return (result);
}
//---------------------------------------------------------------------------
// set 'steps' encoder motor steps using standard digital input
bool FTencoderMotor::setStepsOLD(int steps) {

  Serial.print("in set Steps Standard with steps: ");
  Serial.println(steps);

  int state = 0;
  int prev_state = 0;
  int delta = 1;
  bool go_on = true;

  if (abs(steps) > delta) {
    while (go_on) {
      interface->getInputs();
      Serial.println(encoderPos);
      if (steps > 0) {
        setMotorCW();
        if ((encoderPos - startPos) < (steps + delta)) {
          state = interface->getInput(sensor);
          if (state != prev_state) {
             prev_state = state;
             encoderPos++;
          }
        }
        if ((encoderPos - startPos) > (steps - delta)) {
          setMotorSTOP();
          go_on = false;
          startPos = encoderPos;
        }
      } else if (steps < 0) {
        // add code for negative steps
        setMotorCCW();
        if ((encoderPos - startPos) > (steps - delta)) {

          state = interface->getInput(sensor);
          if (state != prev_state) {
             prev_state = state;
             encoderPos--;
          }
        }
        if ((encoderPos - startPos) < (steps + delta)) {
          setMotorSTOP();
          go_on = false;
          startPos = encoderPos;
        }
      }
    }
  } else {  // allready in position
    go_on = false;
      Serial.println("already in position");
  }
return (!go_on);
}

//---------------------------------------------------------------------------
// set 'steps' encoder motor steps using intterupts digital input
bool FTencoderMotor::setStepsNEW(int steps) {

  // Serial.println("in set Steps Interrupts");

  // store initial encoder position
  int delta = 5;
  bool inPosition = false;

  delay(5);
  if (steps > (encoderPos - startPos + delta)) {
    //   Serial.println("CW");
    setMotorCW();
    movingRight = true;
  } else if ((steps < encoderPos - startPos - delta)) {
    //  Serial.println("CCW");
    setMotorCCW();
    movingRight = false;
  } else {
    setMotorSTOP();
    startPos = encoderPos;
    inPosition = true;
  }
  return (inPosition);
}


//---------------------------------------------------------------------------
// set the origin for this stepmotor
void FTencoderMotor::setOrigin(int newOrigin) {
  origin = newOrigin;
  encoderPos = newOrigin;
  startPos = encoderPos;
}

//---------------------------------------------------------------------------
// set the origin for this stepmotor
int FTencoderMotor::getPosition() {
  return(encoderPos);
}

//---------------------------------------------------------------------------
// set the range for this stepmotor
void FTencoderMotor::setRange(int newOrigin, int newMaximum) {
  origin = newOrigin;
  maximum = newMaximum;
}

//---------------------------------------------------------------------------
// move to the origin
bool FTencoderMotor::moveToOrigin() {
  return (moveToPosition(origin));
}

//---------------------------------------------------------------------------
// move to a positon
bool FTencoderMotor::moveToPosition(int position) {
  int newPos = 0;

  newPos = max(origin, position - startPos);
  newPos = min(newPos, maximum);

  return (setSteps(position - startPos));
}

//---------------------------------------------------------------------------
// move a relative number of steps
bool FTencoderMotor::moveRelative(int delta) {
  return (setSteps(delta));
}

bool FTencoderMotor::findHome(int endPin, motorDirection dir){
  bool found = interface->setMotorUntil(motor, endPin, ON, dir);
  setOrigin(0);
}


//---------------------------------------------------------------------------
// move a relative number of steps
void FTencoderMotor::setMotorCW() {
  movingRight = true;
  //Serial.println("in CW");
  //encoderPos++;
  interface->setMotorCW(motor);
}

//---------------------------------------------------------------------------
// move a relative number of steps
void FTencoderMotor::setMotorCCW() {
  movingRight = false;
  //encoderPos--;
  interface->setMotorCCW(motor);
}
//---------------------------------------------------------------------------
// move a relative number of steps
void FTencoderMotor::setMotorSTOP() {
  Serial.println("in STOP");
  interface->setMotorSTOP(motor);
}
