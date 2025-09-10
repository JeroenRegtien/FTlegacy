/*******************************************************************************************
 * Arduino Library to control a legacy parallel and serial fischertechnik(r) computing interfaces.
 * Original code by Jeroen Regtien, December 2023, with revisions since.
 *
 * Suported interfaces:
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
 *       http://www.ftcommunity.de/ftpedia_ausgaben/ftpedia-2014-1.pdf
 *       http://www.ftcommunity.de/ftpedia_ausgaben/ftpedia-2014-2.pdf
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

#if defined (ARDUINO_UNOR4_MINIMA)
SoftwareSerial mySerial(1, 0);  // RX, TX
#else
SoftwareSerial mySerial(10, 11);  // RX, TX
#endif

ftDisplayTypes ftDisplayType;

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
  } else if (type == SER) {
    // for serial assume interfaces are connected to correct serial port
  }

  if (type == PAREX) {
    numUnits = 2;
  } else {
    numUnits = 1;
  }

  // Reset();
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
    //digitalWrite(TRIGGERX, LOW);
    // digitalWrite(TRIGGERX, HIGH);

    while (!digitalRead(m_pin[DATACOUNTIN])) {
      tEnd = millis() - starttime;
      Serial.println(tEnd);
      if (tEnd > 50) {
        result = false;
        Serial.println("No analog signal");
        break;
      }
    }

    if (result) {
      if (connectedAnalog() > 0) {
        result = false;
        Serial.println(F("No constant analog signal"));
      } else {
        Serial.println(F("Analog signal OK"));
      }
    }
  }
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

    // Serial.println(F("Set output..."));

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
      printOutputBuffer();

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
        writeMEGA(number, 2);
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
        writeMEGA(number, 3);
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
    Serial.println(tempX);
    if (tempX > tempMaxX) {
      tempMaxX = tempX;
    } else if (tempX < tempMinX) {
      tempMinX = tempX;
    }

    tempY = getParallelAnalog(1);
    Serial.println(tempY);
    if (tempY > tempMaxY) {
      tempMaxY = tempY;
    } else if (tempY < tempMinY) {
      tempMinY = tempY;
    }
  }

  if ((tempMaxX - tempMinX) > 10) {
    // random analog data Ex
    codeX = 1;
    Serial.println(F("Ex issue"));
  }
  if ((tempMaxY - tempMinY) > 10) {
    Serial.println(F("Ey issue"));
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

  delay(8);
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
  delay(10);
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

  if (getInput(E) == until) {
    setMotorSTOP(M);
    // Serial.println("stop");
    return (false);
  } else {
    setMotor(M, dir);
    // Serial.println("go");
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
      Serial.println("stop");
      go_on = false;

    } else {
      setMotor(M, dir);
      Serial.println("go");
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
  unsigned long startTimer = 0;

  startTimer = millis();
  setMotor(M, dir);
  while ((count < maxCount) && go_on) {
    getInputs();
    if (getInput(E1) == until1) {
      count++;
    }
    if (E2>0) {
      if (getInput(E2) == until2) {
        go_on = false;
      }
    }
    if ((millis() - startTimer) > 10000) {
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
  unsigned long startTimer = 0;

  startTimer = millis();
  setOutputON(O);
  while ((count < maxCount) && go_on) {
    getInputs();
    if (getInput(E1) == until1) {
      count++;
    }
    if (E2>0) {
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

#if defined(ARDUINO_AVR_UNO) || defined (ARDUINO_UNOR4_MINIMA)
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
  } else {
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

  lcd.begin(numLCDcolumns, numLCDrows);

  if (ftDisplayType == D1604 || ftDisplayType == D2004) {

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
    delay(5000);
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

void FTcontroller::ftMessageToDisplay(int x, int y, char* message) {

  int offset;
  lcd.clear();

  if (ftDisplayType == D2004) {
    offset = 2;
  } else {
    offset = 0;
  }

  lcd.setCursor(x + offset, y);
  lcd.print(message);

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
