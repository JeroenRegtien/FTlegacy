/**
 * \file 
 * \brief Arduino Library to control legacy parallel and serial fischertechnik(r) computing interfaces.
 * Original code by Jeroen Regtien, December 2023 with revisions since.
 * 
 * @details
 * Suported interfaces:
 *      1984:  Parallel Interface Commodore, IBM, Atari: 30562, 30563, 30565
 *      1991:  Parallel Universal Interface: 30520
 *      1991:  Parallel CVK Interface: XXXXX
 *      1997:  Serial / Intelligent Interface: 30402
 *      2004:  ROBO interface: 93293
 * 
 * Supported PLCs: none
 *
 * Supported Shields: none
 * 
 * Supported Arduino's: UNO R3, UNO R4 Minima, MEGA, Nano
 * check UNO R4 Wifi, GIGA
 *
 * The library consists of two files:
 * FTlegacy.h - header file for the FTmodule, FTcontroller and FTtimer class \n 
 * FTlegacy.cpp - C++ implementation of class, methods and utility functions \n 
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

   Version 0.6 - Added LCD 2004 option. \n 
   Version 0.7 - Serial extension capability, code cleanup, add actuators, lamps. \n 
   Version 0.8 - Used for several workstations. Actuators removed as object.  \n 
   Version 0.9 - Parallel Extensions added. Analog input via parallel interface now works.
                Consolidated and rationalised. Given Arduino UNO memory limitations, 
                two separate classes introduced: FTlegacy for legacy fischertechnik 
                interfaces and FTmodule also including 3rd party shields and controllers. 
                Version after many prototype tests. \n 
                

*/

#ifndef _FT_LEGACY_H
#define _FT_LEGACY_H

/// @brief sotfware library release version
#define VERSION "FT_L V0.9"

/// @brief standardised E1 input channel
#define ft_E1 1
/// @brief standardised E2 input channel
#define ft_E2 2
/// @brief standardised E3 input channel
#define ft_E3 3
/// @brief standardised E4 input channel
#define ft_E4 4
/// @brief standardised E5 input channel
#define ft_E5 5
/// @brief standardised E6 input channel
#define ft_E6 6
/// @brief standardised E7 input channel
#define ft_E7 7
/// @brief standardised E8 input channel
#define ft_E8 8
/// @brief standardised E1 input channel optional extension interface
#define ft_E9 9
/// @brief standardised E2 input channel optional extension interface
#define ft_E10 10
/// @brief standardised E3 input channel optional extension interface
#define ft_E11 11
/// @brief standardised E4 input channel optional extension interface
#define ft_E12 12
/// @brief standardised E5 input channel optional extension interface
#define ft_E13 13
/// @brief standardised E6 input channel optional extension interface
#define ft_E14 14
/// @brief standardised E7 input channel optional extension interface
#define ft_E15 15
/// @brief standardised E8 input channel optional extension interface
#define ft_E16 16

/// @brief bidirectional Motor 1
#define ft_M1 1
/// @brief bidirectional Motor 2
#define ft_M2 2
/// @brief bidirectional Motor 3
#define ft_M3 3
/// @brief bidirectional Motor 4
#define ft_M4 4
/// @brief bidirectional Motor 1 optional extension interface
#define ft_M5 5
/// @brief bidirectional Motor 1 optional extension interface
#define ft_M6 6
/// @brief bidirectional Motor 1 optional extension interface
#define ft_M7 7
/// @brief bidirectional Motor 1 optional extension interface
#define ft_M8 8

// numbers for lamp actuators, two per motor
// NOTE: lamp 1&2, 3&4, etc can never be activated at the same time, it is either/or
/// @brief standardised output channel 1, using half of motor 1 output
#define ft_O1 1
/// @brief standardised output channel 1, using the other half of motor 1 output
#define ft_O2 2
/// @brief standardised output channel 1, using half of motor 2 output
#define ft_O3 3
/// @brief standardised output channel 1, using the other half of motor 2 output
#define ft_O4 4
/// @brief standardised output channel 1, using half of motor 3 output
#define ft_O5 5
/// @brief standardised output channel 1, using the other half of motor 3 output
#define ft_O6 6
/// @brief standardised output channel 1, using half of motor 4 output
#define ft_O7 7
/// @brief standardised output channel 1, using the other half of motor 4 output
#define ft_O8 8
/// @brief standardised output channel 1, using half of extension motor 1 output
#define ft_O9 9
/// @brief standardised output channel 1, using the other half of extension motor 1 output
#define ft_O10 10
/// @brief standardised output channel 1, using half of extension motor 2 output
#define ft_O11 11
/// @brief standardised output channel 1, using the other half of extension motor 2 output
#define ft_O12 12
/// @brief standardised output channel 1, using half of extension motor 3 output
#define ft_O13 13
/// @brief standardised output channel 1, using the other half of extension motor 3 output
#define ft_O14 14
/// @brief standardised output channel 1, using half of extension motor 4 output
#define ft_O15 15
/// @brief standardised output channel 1, using the other half of extension motor 4 output
#define ft_O16 16

/// @brief standardised global ON for interface outputs
#define ON true
/// @brief standardised global OFF for interface outputs
#define OFF false

#include <Arduino.h>

#include <Adafruit_GFX.h>
#define SSD1306_NO_SPLASH
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#include <SoftwareSerial.h>

#include <hd44780.h>                        // main hd44780 header for LCD
#include <hd44780ioClass/hd44780_I2Cexp.h>  // i2c expander i/o class header

const int UNO = 1;
const int MEGA = 2;
const int UNO4 =3;
const int NANO = 4;
const int OTHER = 0;

// retrieve type of board from Arduino 
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)
const int board = UNO;
const int maxBoards = 2;
#elif defined(ARDUINO_AVR_MEGA)
const int board = MEGA;
const int maxBoards = 6;
#elif defined(ARDUINO_AVR_MEGA2560)
const int board = MEGA;
const int maxBoards = 6;
#else
const int board = OTHER;
const int maxBoards = 0;
#endif

/// @brief supported LCD or OLED displays
/// @details
/// D1604  : 16x4 character LCD display with I2C \n 
/// D1602  : 16x2 character LCD display with I2C \n 
/// D2004  : 20x4 character LCD display with I2C \n 
enum ftDisplayTypes {
  NONE = 0,
  D1604 = 1,
  D1602 = 2,
  D2004 = 3
};

// define global LCD display defaults, these can be overwritten
const int lcdColumns = 20;
const int lcdRows = 4;

extern hd44780_I2Cexp lcd;

/// @brief define global softserial port
extern SoftwareSerial mySerial;

/// @brief define global display type
extern ftDisplayTypes ftDisplayType;

/// @brief define type for motor direction
/// @details
/// CCW : CounterClockWise \n 
/// STOP: stop \n 
/// CW  : ClockWise \n 
enum motorDirection {
  CCW = -1,
  STOP = 0,
  CW = 1
};

/// @brief define interface type
/// @details
///    PAR      : parallel or universal interface.\n 
///    PAREX    : parallel or universal interface with extension unit.\n 
///    SER      : serial or intelligent interface.\n 
///    SEREX    : serial or intelligent interface with extension unit.\n 
///    ROBO     : Robo interface.\n 
///    DID_UNO  : Didacta Arduino Uno Shield (supported in FTmodule library).\n 
///    DID_MEGA : Didacta Arduino Mega Shield (supported in FTmodule library).\n 
///    ADA      : Adafruit Motor Shield (supported in FTmodule library).\n 
///    CONT_MINI: Controllino PLC mini (supported in FTmodule library).\n 
///    CONT_MAXOUT: Controllino PLC maxi automation (supported in FTmodule library).\n 
///    CONT_MICRO: Controllino PLC micro (supported in FTmodule library).\n 
///    FT_NANO  : ftNano shield (supported in FTmodule library).\n 
enum typeIFace {
  PAR = 1, 
  PAREX = 2,       
  SER = 3,
  SEREX = 4,
  ROBO = 5,
  DID_UNO = 6,  
  DID_MEGA = 7,
  ADA = 8,
  CONT_MINI = 9,
  CONT_MAXOUT = 10,
  CONT_MICRO = 11,
  FT_NANO = 12
};

/// FTlegacy class
/**
  * \details   Class to manage one interface
  * \author    Jeroen Regtien
  * \date      2024-2025
  * \version   1.0


  PARALLEL INTERFACES
  --------------------
  20-pin  ribbon cable pinout

  Signal directions relative to Arduino.   The pin numbers on the PCB in the 30520 
  interface are printed backwards (20<->1, 19<->2 etc).
  The analog inputs (EX and EY) run from the Fischertechnik side of the
  interface to the computer, and then are looped back to two 556 timers on
  the interface. Two loopback wires must be installed (between pins 5 and 7,
  and between pins 6 and 8) on the Arduino side of the gray ribbon cable to
  make the analog inputs work as expected. It may be possible to connect
  those pins directly to two analog input pins on the Arduino, but this is
  not supported by the library.

     Cable PIN    | Aduino PIN  | Direction | Function
    ------------- | ----------- | --------- | ---------
  |      20 / 1   |             |           | GND
  |      19 / 2   |             |           | GND
  |      18 / 3   |       2     |    IN     | DATA/COUNT IN
  |      17 / 4   |             |           | Not connected
  |      16 / 5   |             |           | Connect to pin 14/7
  |      15 / 6   |             |           | Connect to pin 13/8
  |      14 / 7   |             |           | Connect from pin 16/5
  |      13 / 8   |             |           | Connect from pin 15/6
  |      12 / 9   |       3     |    OUT    | TRIGGER X
  |      11 / 10  |       4     |    OUT    | TRIGGER Y
  |      10 / 11  |       5     |    OUT    | DATA OUT
  |       9 / 12  |       6     |    OUT    | CLOCK
  |       8 / 13  |       7     |    OUT    | LOAD OUT
  |       7 / 14  |       8     |    OUT    | LOAD IN
  |       6 / 15  |             |           | Not connected
  |       5 / 16  |             |           | Not connected
  |       4 / 17  |             |           | Not connected
  |       3 / 18  |             |           | Not connected
  |       2 / 19  |      19     |           | GND
  |       1 / 20  |      20     |           | GND


  SERIAL (INTELLIGENT) INTERFACE WITHOUT EXTENSION MODULE 
  -----------------------------------------------------  
   Interface setup: Baudrate=9600, Databits=8, Parity=none, Stopbits=1

   Arduino Serial ports: \n 
    UNO: TX, RX
    MEGA: Serial 1 (TX1=18,RX1=19), Serial 2 (TX2=16, RX2=17), Serial 3 (TX3=14, RX3=15), TX0, RX0

   To control the interface, two bytes should be sent. The first byte is the interface 
   command (see below) and the second byte contains the motor state.

   The motor state describes which motors should be started in which direction.
   Depending on the interface command, the interface replies with one or three
   bytes.

   First byte: Interface command

   | Binary   | Hex | Decimal | Description                                    
   ---------- | --- |  ------ | ------------------------------------------------
   | 11000001 | C1  | 193     | Only I/O state                                 
   | 11000101 | C5  | 197     | I/O state and analog value EX                  
   | 11001001 | C9  | 201     | I/O state and analog value EY                  


   Second byte: Motor state

   | Bits | Description                                                        
   ------ | --------------------------------
   | 1    | Motor 1 counter-clockwise (CCW)                                    
   | 2    | Motor 1 clockwise (CW)                                             
   | 3    | Motor 2 counter-clockwise                                          
   | 4    | Motor 2 clockwise                                                  
   | 5    | Motor 3 counter-clockwise                                          
   | 6    | Motor 3 clockwise                                                  
   | 7    | Motor 4 counter-clockwise                                          
   | 8    | Motor 4 clockwise                                                  


   Motors can be controlled in parallel by setting the required bits, however, the CW and 
   CCW modes should not be set at the same time, this invalid command will be ignored.

   Replies from the interface:
   
   | Cmd Dec | Cmd Hex| CMD bin  | Reply bytes | Description                                    
    ---------|--------|----------| ------------|-------------------------------
   | 193     |  0xC1  | 11000001 | one byte    | Each bit represents the value of an input       
   | 197     |  0XC5  |          | three bytes | Byte 1 see above, Byte 2&3 analog Ex                            
   | 201     |  0XC9  |          | three bytes | Byte 1 see above, Byte 2&3 analog Ey     
       
   In general, programming should be done in threads. If the interface doesn't
   get a command every 300 ms, the motors are turned off automatically.


  SERIAL INTELLIGENT INTERFACES WITH EXTENSION MODULE
  ----------------------------------------------------- 

   Interface setup: Baudrate=9600, Databits=8, Parity=none, Stopbits=1

  To control the extension module, basically three bytes should be sent. The
  first byte is the interface command (see below), the second byte contains
  the motor state of the Intelligent Interface (interface 1), and the third
  byte contains the motor state of the extension module (interface 2).

  The motor state describes which motors should be started in which direction.
  Depending on the interface command, the interface replies with two or four
  bytes.

  The interface commands (193, 197, 201) which control only interface 1, can
  still be used 

  First byte: Interface command for interface and extension module

   | Binary   | Hex | Decimal | Description                                    
   ---------- | --- |  ------ | ------------------------------------------------
   | 11000001 | C2  | 194     | Only I/O state of interface 1 and 2                                    
   | 11000101 | C6  | 198     | I/O state interface 1 and 2, analog value EX                  
   | 11001001 | CA  | 202     | I/O state interface 1 and 2, analog value EY                  


  Second byte: Motor state for interface 1

   | Bits | Description                                                        
   ------ | --------------------------------
   | 1    | Motor 1 counter-clockwise (CCW)                                    
   | 2    | Motor 1 clockwise (CW)                                             
   | 3    | Motor 2 counter-clockwise                                          
   | 4    | Motor 2 clockwise                                                  
   | 5    | Motor 3 counter-clockwise                                          
   | 6    | Motor 3 clockwise                                                  
   | 7    | Motor 4 counter-clockwise                                          
   | 8    | Motor 4 clockwise               

  Third byte: Motor state for the extension

   | Bits | Description                                                        
   ------ | --------------------------------
   | 1    | Motor 1 counter-clockwise (CCW)  (motor ft_M5 in the software)                                    
   | 2    | Motor 1 clockwise (CW)           (motor ft_M5 in the software)                                        
   | 3    | Motor 2 counter-clockwise        (motor ft_M6 in the software)                                     
   | 4    | Motor 2 clockwise                (motor ft_M6 in the software)                                     
   | 5    | Motor 3 counter-clockwise        (motor ft-M7 in the software)                                     
   | 6    | Motor 3 clockwise                (motor ft_M7 in the software)                                    
   | 7    | Motor 4 counter-clockwise        (motor ft_M8 in the software)                                     
   | 8    | Motor 4 clockwise                (motor ft_M8 in the software)   

   Motors can be controlled in parallel by setting the required bits, however, the CW and 
   CCW modes should not be set at the same time, this invalid command will be ignored.


  Replies from the interface


   | Cmd Dec | Cmd Hex| CMD bin  | Reply bytes | Description                                    
    ---------|--------|----------| ------------|-------------------------------
   | 194     |  0xC2  | 11000001 | two byte    | Byte 1 = Interface 1, Byte 2 = extension.       
   | 198     |  0XC6  | 11000101 | four bytes  | Byte 1,2 see above, Byte 3&4 analog Ex interface 1                            
   | 202     |  0XCA  | 11001001 | four bytes | Byte  1,2 see above, Byte 3&4 analog Ey interface 1 


  In general, programming should be done in threads. If the interface doesn't
  get a command every 300 ms, the motors are turned off automatically.


 */
class FTlegacy {

public:
   int type;

protected:
  int number = 0;
  int numUnits = 1;

private:
  // parallel interface pin register
  int startPin[4] = { 2, 22, 32, 42 };
  // MEGA serial interface pin register , order is  Serial, Serial1, Serial2, Serial3
  int serialPin[8] = { 0, 1, 18, 19, 16, 17, 15, 14 };

private:  // only visible in the defined class
  int pinParEx = A2;
  int pinParEy = A3;
  unsigned long analogTimeout = 7000;

  static FTlegacy* ifaceList[8];
  static int ifaceCount;

private:  // only visible in the defined class and classes that inherit

  // parallel state parameters
  int m_out[17] = { 0 };   // output 8 motors, index 0 not used
  int m_in[17];            // input 16 switches, index 0 not used
  int m_ana[2];            // to store 2 analog values Ax and Ay
  int m_prev[17] = { 0 };  // previous output, index 0 not used

  enum {
    // Pin name / Interface pin
    DATACOUNTIN,  // DATA/COUNT IN
    TRIGGERX,     // TRIGGER X
    TRIGGERY,     // TRIGGER Y
    DATAOUT,      // DATA OUT
    CLOCK,        // CLOCK
    LOADOUT,      // LOAD OUT
    LOADIN,       // LOAD IN
    COUNTIN,      // COUNTIN

    // Helper
    NumPins  // Number of pins used
  };

  // Array that defines which interface pin is connected to which Arduino pin
  // See enum definition above for indexes.
  // This is initialized at construction time and not changed afterwards.
  byte m_pin[NumPins];

private:
  // serial state parameters
  byte outByte[3];
  int n_written = 0;
  unsigned int iD = 0;
  byte inByte[16];

  int pAx;
  int pAy;


private:
  //-------------------------------------------------------------------------
  // Private function called during construction
  void InitPar(
    byte pin_datacountin,
    byte pin_triggerx,
    byte pin_triggery,
    byte pin_dataout,
    byte pin_clock,
    byte pin_loadout,
    byte pin_loadin,
    byte pin_countin) {

    // Initialize Arduino pin numbers for each pin on the interface
    m_pin[DATACOUNTIN] = pin_datacountin;
    m_pin[TRIGGERX] = pin_triggerx;
    m_pin[TRIGGERY] = pin_triggery;
    m_pin[DATAOUT] = pin_dataout;
    m_pin[CLOCK] = pin_clock;
    m_pin[LOADOUT] = pin_loadout;
    m_pin[LOADIN] = pin_loadin;
    m_pin[COUNTIN] = pin_countin;
  }

public:

  /// @brief Simpler constructor for when connected to consecutive Arduino pins
  FTlegacy(int, int);

  /// @brief zero constructor
  FTlegacy();

  /// @brief initilisation of interface instance
  bool begin();  // Returns True=success False=failure

private:
  void readWriteMEGA(int, int);  // utility function to read/write serial information to the MEGA
  void readWriteUNO(int);        // utility function to read/write serial information to the UNO
  void writeMEGA(int, int);      // utility function to write serial information to the MEGA
  void writeUNO(int);            // utility function to write serial information to the UNO
  int ftDecodeAnalog(int, int);  // utility function to decode analog data from serial bytes

public:      
  /// @brief get digital input for all pins and store in buffer       
  void getInputs();  

  /// @brief get digital input for pin from buffer
  /// \param pin digital input channel one of ft_E1 to ft_E16
  bool getInput(int pin);

  /// @brief get  digital input for two pins from buffer and return true if both are pressed
  /// \param pin1 digital input channel one of ft_E1 to ft_E16
  /// \param pin2 digital input channel one of ft_E1 to ft_E16
  bool getInput2(int pin1, int pin2);

  /// @brief reset input buffer to zero
  void zeroInput();                    // set all digital inputs to zero

  /// @brief retrieve analog Ex, Ey inputs from interface and store in buffer
  void getAnalogInputs(); 

  /// @brief return analog X (Ex) value from buffer              
  int getAnalogX();        

  /// @brief return analog Y (Ey) value from buffer 
  int getAnalogY();

  /// @brief get serial analog input: 0=X, 1=Y
  int getSerialAnalog(int xory);

  /// @brief get parallel analog input, 0=X, 1=Y
  long int getParallelAnalog(int xory);  

    /// @brief print input buffer to serial monitor
  void printInputBuffer(); 
  
  /// @brief returns >0 if problems with Ex, Ey
  int connectedAnalog(); 

public:  // motor control methods

  /// @brief sets motors M to motorDirection dir.
  /// \param M motor number
  /// \param dir direction of type motorDirection
  void setMotor(int M, motorDirection dir);

  /// @brief sets motor M to motorDirection Clockwise
  /// \param M motor number
  void setMotorCW(int M);

  /// @brief sets motor M to motorDirection Clockwise
  /// \param M motor number
  void setMotorCCW(int M);

  /// @brief sets motor M to STOP
  /// \param M motor number
  void setMotorSTOP(int M);

  /// @brief sets all motors stop.
  void setAllMotorsSTOP();

  /// @brief gets motorDirection for motor M
  /// \param M motor number
  /// \return enum type motorDirection
  motorDirection getMotor(int M);
 
  /// @brief tests whether target end switch was hit whilst moving motor
  /// \param M motor number
  /// \param E switch number
  /// \param until ON/true or OFF/false
  /// \return bool returns true of target not hit.
  bool getMotorUntil(int M, int E, bool until, motorDirection dir);

  /// @brief keeps motor running until end switch action
  /// \param M motor number
  /// \param E switch number
  /// \param until ON/true or OFF/false
  /// \return bool returns true of target not hit.
  bool setMotorUntil(int M, int E, bool until, motorDirection dir);

  /// @brief keeps motor running until switch count is reached
  /// \param M motor number
  /// \param E switch number
  /// \param until ON/true or OFF/false
  /// \return bool returns false when end point reached
  bool setMotorUntilCount(int M, int E, bool until, motorDirection dir, int maxCount);

  /// @brief keeps motor running until E1 switch count is reached or switch E2 is triggered
  /// \param M motor number
  /// \param E1 switch number
  /// \param until1 ON/true or OFF/false
  /// \param E2 switch number
  /// \param until2 ON/true or OFF/false
  /// \return bool returns false when end point reached
  bool setMotorUntilOrCount(int M, int E1, bool until1, int E2, bool until2, motorDirection dir, int maxCount);
 
  /// @brief sets output channel to ON 
  /// @param O output number , one of ft_O1 to ft_O16
  void setOutputON(int O);

  /// @brief sets output channel to OFF
  /// @param O output number , one of ft_O1 to ft_O16
  void setOutputOFF(int O);

  /// @brief sets output channel ON or OFF. 
  /// @param O output number
  /// @param status ON (1) or OFF (0)
  void setOutput(int O, int status);

/// @brief tests whether target end switch was hit whilst output is active
  /// \param O motor number
  /// \param E switch number
  /// \param until ON/true or OFF/false
  /// \return bool returns true of target not hit.
  bool getOutputUntil(int O, int E, bool until);

  /// @brief keeps motor running until end switch action
  /// \param O motor number
  /// \param E switch number
  /// \param until ON/true or OFF/false
  /// \return bool returns true of target not hit.
  bool setOutputUntil(int O, int E, bool until);

  /// @brief keeps motor running until switch count is reached
  /// \param O motor number
  /// \param E switch number
  /// \param until ON/true or OFF/false
  /// \return bool returns false when end point reached
  bool setOutputUntilCount(int O, int E, bool until, int maxCount);

  /// @brief keeps motor running until E1 switch count is reached or switch E2 is triggered
  /// \param O motor number
  /// \param E1 switch number
  /// \param until1 ON/true or OFF/false
  /// \param E2 switch number
  /// \param until2 ON/true or OFF/false
  /// \return bool returns false when end point reached
  bool setOutputUntilOrCount(int O, int E1, bool until1, int E2, bool until2, int maxCount);

  /// @brief sends output buffer to interface
  void setOutputs();         // rename to sendoutputs

  /// @brief print output buffer to serial monitor
  void printOutputBuffer();

public:  // display IO methods
  /// @brief updates LCD display with current in- and output status information
  void ftUpdateDisplay();

private:
  void ftUpdateLCD(int type);
  void ftLCD_M(int x, int y, int M);
  void ftLCD_E(int x, int y, int M);
};

/// FTcontroller class
///
/// @details   Class to manage the (micro)controller
/// @author    Jeroen Regtien
/// @date      2024-2025
/// @version   1.0
/// 
/// Depending on the type of controller one or more interface can be added. 
class FTcontroller {

private:
  int numTot;     // total number of declared interfaces
  int numPar;     // total number of declared parallel interfaces
  int numSer;     // total number of declared serial interfaces
  int boardType;      // board indicator
  int numInterfaces; // check whether still needed
  int numLCDcolumns;
  int numLCDrows;

public:
  /// Constructor, creates controller
  /// @param display_type type of display, one of enum ftDisplayTypes
  FTcontroller(ftDisplayTypes display_type) {
    numTot = 0;
    boardType = board;
    ftDisplayType = display_type;
  }

  /// Initialise Controller
  /// \param Program Name
  void begin(char *message);

  /// Add interface to the list of interfaces
  /// \param instance of FTlegacy class
  void addInterface(FTlegacy ftLegacy);

  /// Retrieve the number of interfaces connected
  /// \return The number of declared interfaces
  int getNumberInterfaces();

  /// Get the type of the controller board
  /// \return an integer that defines the microcontroller board.
  int getBoard();

  /// Send a text message to the display (if connected)
  /// \param x-index on LCD display
  /// \param y-index on LCD display
  /// \param *message to display
  void ftMessageToDisplay(int x, int y, char *messsage);

public:
  FTlegacy ftLegacies[maxBoards];

};


/// FTtimer class
/**
  * \details   Class for simple user defined elegant timers
  * \author    Kiryanenko
  * \date      05.10.19
  * \version   1.0


EXAMPLE CODE TImer


FTtimer firstTimer(5000);     // Create a  timer and specify its interval in milliseconds

secondTimer.interval(3000);   // Re-Set an interval to 3 secs for a timer

if (firstTimer.ready()) {...} // check whether a timer is ready

secondTimer.reset();          // reset a timer

 */
 class FTtimer {
  unsigned long _start;
  unsigned long _interval;

public:
  /// Constructor, initializes timer
  /// \param interval An timing interval in msec
  explicit FTtimer(unsigned long interval = 0);

  /// Check if timer is ready
  /// \return True if is timer is ready
  bool ready();

  /// Set the time interval
  /// \param interval An interval in msec
  void interval(unsigned long interval);

  /// Reset a timer
  /// \details 
  void reset();

};

#endif


