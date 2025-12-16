/**
 * \file FTlegacy.h
 * \brief Arduino Library to control legacy parallel and serial fischertechnik(r) computing interfaces.
 * \author Jeroen Regtien
 * \date  December 2023 with revisions since.
 * 
 * @details
 * Supported parallel interfaces: Universal (30520), CVK (66843), Centronics (30566)
 * Supported serial interfaces: Intelligent (30402), ROBO (93293)
 * 
 * Supported PLCs: none
 *
 * Supported Shields: none
 * 
 * Supported Arduino's: UNO R3, UNO R4 Minima/Wifi, MEGA, Nano
 *
 * The library consists of two files:
 * FTlegacy.h - header file for the FTmodule, FTcontroller and FTtimer class \n 
 * FTlegacy.cpp - C++ implementation of class, methods and utility functions \n 
 *
 * References:
 *      https://www.ftcommunity.de/ftpedia/2014/2014-1/ftpedia-2014-1.pdf
 *      https://www.ftcommunity.de/ftpedia/2017/2017-2/ftpedia-2017-2.pdf
 *      https://www.ftcommunity.de/ftpedia/2017/2017-3/ftpedia-2017-3.pdf
 *      https://www.ftcommunity.de/ftpedia/2017/2017-4/ftpedia-2017-4.pdf
 *      https://www.ftcommunity.de/ftpedia/2023/2023-4/ftpedia-2023-4.pdf
 *      https://www.ftcommunity.de/ftpedia/2025/2025-2/ftpedia-2025-2.pdf
 *
 * The MIT License (MIT)
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
 *
 *   Version 0.6 - Added LCD 2004 option. \n
 *   Version 0.7 - Serial extension capability, code cleanup, add actuators, lamps. \n
 *   Version 0.8 - Used for several workstations. Actuators removed as object.  \n
 *   Version 0.9 - Parallel Extensions added. Analog input via parallel interface now works.
 *               Consolidated and rationalised. Given Arduino UNO memory limitations,
 *               two separate classes introduced: FTlegacy for legacy fischertechnik
 *               interfaces and FTmodule also including 3rd party shields and controllers.
 *               Version after many prototype tests. \n
 *  Version 1.0 - First operational release. Added FTstepper(XY) and FTencoder classes \n
 *********************************************************************************************/

#ifndef _FT_LEGACY_H
#define _FT_LEGACY_H

/// @brief software library release version
#define VERSION "FT_L V1.0"

#include <Arduino.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#include <SoftwareSerial.h>
#include <Wire.h>
#include <hd44780.h>                        // main hd44780 header for LCD
#include <hd44780ioClass/hd44780_I2Cexp.h>  // i2c expander i/o class header

const int UNO = 1;
const int MEGA = 2;
const int UNO4 =3;
const int NANO = 4;
const int ESP32 = 5;
const int OTHER = 0;

// retrieve type of board from Arduino 
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)
const int board = UNO;
const int maxBoards = 2;
#elif defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
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

/// @brief define global lcd object
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
///    ADA_UNO : Adafruit Motor Shield (supported in FTmodule library).\n
///    CONT_MINI: Controllino PLC mini (supported in FTmodule library).\n
///    CONT_MAXOUT: Controllino PLC maxi automation (supported in FTmodule library).\n 
///    CONT_MICRO: Controllino PLC micro (supported in FTmodule library).\n 
///    FT_NANO  : ftNano shield (supported in FTmodule library).\n
///    FT_DUINO : ftDuino controller/shield (supported in FTmodule library).\n

enum typeIFace {
  PAR = 1,
  PAREX = 2,       
  SER = 3,
  SEREX = 4,
  ROBO = 5,
  DID_UNO = 6,  
  DID_MEGA = 7,
  ADA_UNO = 8,
  CONT_MINI = 9,
  CONT_MAXOUT = 10,
  CONT_MICRO = 11,
  FT_NANO = 12,
  FT_DUINO = 13
};

/// FTlegacy class
/**
  * \details   Class to manage one interface
  * \author    Jeroen Regtien
  * \date      2024-2025
  * \version   1.0

 */
class FTlegacy {

public:
  int type;
  int ftInfo;

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

  // interface state parameters
  int m_out[17] = { 0 };  // output 8 motors, index 0 not used
  int m_prev[17] = { 0 }; // previous output, index 0 not used
  int m_in[17];           // input 16 switches, index 0 not used
  int m_ana[4];           // to store at least 2 analog values Ax and Ay

  enum {
    // Pin name / Interface pin
    DATACOUNTIN,  // DATA/COUNT IN
    TRIGGERY,     // TRIGGER X
    TRIGGERX,     // TRIGGER Y
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
    byte pin_triggery,
    byte pin_triggerx,
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

  /// @brief initialisation of interface instance
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

  /// @brief get direct digital input from Arduino pin, no buffer
  /// \param pin Arduino pin
  bool getDirectInput(int pin);

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
  /// \param xory choose between X (=0) or Y (=1)
  long int getParallelAnalog(int xory);

  /// @brief get direct analog value from Arduino or shields, if no interface is used
  /// \param pin  Arduino pin
  int getDirectAnalog(int pin);

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

  /// @brief Tests whether target end switch was hit whilst moving motor
  /// \param M motor number
  /// \param E switch number
  /// \param until ON/true or OFF/false
  /// \param motorDirection direction CCW or CW
  /// \return bool returns true of target not hit.
  bool getMotorUntil(int M, int E, bool until, motorDirection dir);

  /// @brief keeps motor running until end switch action
  /// \param M motor number
  /// \param E switch number
  /// \param until ON/true or OFF/false
  /// \param motorDirection direction CCW or CW
  /// \return bool returns true of target not hit.
  bool setMotorUntil(int M, int E, bool until, motorDirection dir);

  /// @brief keeps motor running until switch count is reached
  /// \param M motor number
  /// \param E switch number
  /// \param until ON/true or OFF/false
  /// \param motorDirection direction CCW or CW
  /// \param int maxCount maximum numer of counts
  /// \return bool returns false when end point reached
  bool setMotorUntilCount(int M, int E, bool until, motorDirection dir, int maxCount);

  /// @brief keeps motor running until E1 switch count is reached or switch E2 is triggered
  /// \param M motor number
  /// \param E1 switch number
  /// \param until1 ON/true or OFF/false
  /// \param E2 switch number
  /// \param until2 ON/true or OFF/false
  /// \param motorDirection direction CCW or CW
  /// \param int maxCount maximum numer of counts
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

  /// @brief switches magnet ON
  /// @param M motor number for magnet, one of ft_M1 to ft_M4
  void magnetON(int M);

  /// @brief switches magnet OFF
  /// @param M motor number for magnet, one of ft_M1 to ft_M4
  void magnetOFF(int M);
    
  /// @brief sends output buffer to interface
  void sendOutputs();

  /// @brief print output buffer to serial monitor
  void printOutputBuffer();

  /// @brief set information level towards Arduino IDE monitor.
  /// @param level can be 0-3, with 0 meaning no input and 3 maximum information. Impacts performance
  void setInfo(int level);

  // display IO methods
  /// @brief updates LCD display with current in- and output status information
  void ftUpdateDisplay();

private:
  void ftUpdateLCD(int type);
  void ftLCD_M(int x, int y, int M);
  void ftLCD_E(int x, int y, int M);
  char displayMotor(int M);

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
  /// @brief Constructor, creates controller
  /// @param display_type type of display, one of enum ftDisplayTypes
  FTcontroller(ftDisplayTypes display_type) {
    numTot = 0;
    boardType = board;
    ftDisplayType = display_type;
  }

  /// @brief Initialise Controller
  /// \param message Program name
  void begin(const char *message);

  /// @brief Add interface to the list of interfaces
  /// \param instance of FTlegacy class
  void addInterface(FTlegacy ftLegacy);

  /// @brief Retrieve the number of interfaces connected
  /// \return int The number of declared interfaces
  int getNumberInterfaces();

  /// @brief Get the type of the controller board
  /// \return int an integer that defines the microcontroller board.
  int getBoard();

  /// @brief Checks whether an I2C device is attached, assumes it to be display
  /// \return bool LCD connected (true or false)
  bool isLCDconnected();

  /// @brief Send a text message to the display (if connected)
  /// \param x-index on LCD display
  /// \param y-index on LCD display
  /// \param *message to display
  /// \param clear the display before adding message
  void ftMessageToDisplay(int x, int y, const char *messsage, bool clear);

public:
  FTlegacy ftLegacies[maxBoards];

};


/// FTtimer class
/**
  * \details   Class for simple user defined elegant timers
  * \author    Kiryanenko with extension by Regtien
  * \date      05.10.19 / 01.06.2024
  * \version   1.0

EXAMPLE CODE Timer

FTtimer firstTimer(5000);     // Create a  timer and specify its interval in milliseconds
secondTimer.interval(3000);   // Reset an interval to 3 secs for a timer
if (firstTimer.ready()) {...} // Check whether a timer is ready
secondTimer.reset();          // Reset a timer
time  = firstTimer.elapsed();  // Retrieve elapsed time
 */

class FTtimer {
  unsigned long _start;
  unsigned long _interval;

public:
  /// @brief Constructor, initializes timer
  /// \param interval An timing interval in msec
  explicit FTtimer(unsigned long interval = 0);

  /// @brief Check if timer is ready
  /// \return True if is timer is ready
  bool ready();

  /// @brief Set the time interval
  /// \param interval An interval in msec
  void interval(unsigned long interval);

  /// @brief Reset a timer
  void reset();

  /// @brief Return elapsed time
  unsigned long elapsed();

};


class FTstepper : public FTlegacy {

private:
  FTlegacy &interface;    // the associated interface
  int origin;             // origin
  int minimum;            // minimum of allowed range
  int maximum;            // maximum of allowed range
  int currentPosition;    // current position
  int coilA;              // coil A
  int coilB;              // coil B

public:
  /// @brief the constructor
  /// \param choice the associated interface
  /// \param MA motor for coil A
  /// \param MB motor for coil B
  FTstepper(FTlegacy &choice, int MA, int MB) : interface(choice) {
    coilA = MA;
    coilB = MB;
    origin = 0;
  }

  /// @brief sets origin
  /// \param newOrigin new origin
  void setOrigin(int newOrigin);

  /// @brief sets range for valid position
  /// \param newMinimum minimum of range
  /// \param newMaximum maximum of range
  void setRange(int newMinimum, int newMaximum);

  /// @brief move to origin
  void moveToOrigin();

  /// @brief move to position
  /// \param position number of steps to be made
  void moveToPosition(int position);

  /// @brief make relative move of steps
  /// \param delta number of steps to be made
  void moveRelative(int delta);

  /// @brief stop motor
  void setStepperSTOP();

private:
  // make steps and do the accounting
  void setStep(int steps);

};

class FTstepperXY : public FTlegacy {

private:
  FTlegacy &interface;      // the associated interface
  int originX;              // x-origin
  int originY;              // y-origin
  int maxX;                 // max range x
  int maxY;                 // max range y
  int coilA1;               // motor number for coil A1
  int coilA2;               // motor number for coil A2
  int coilB1;               // motor number for coil B1
  int coilB2;               // motor number for coil B2
  int actuator;             // motor number for the actuator (eg pen for plotter)
  int currentX;             // current x-position
  int currentY;             // current y-position

public:

  /// @brief the constructor
  /// \param choice the associated interface
  /// \param M1 motor for coil A1
  /// \param M2 motor for coil A2
  /// \param M3 motor for coil B1
  /// \param M4 motor for coil B2
  /// \param pen motor for actuator (eg pen for plotter)
  FTstepperXY(FTlegacy &choice, int Motor1, int Motor2, int Motor3, int Motor4, int pen)
    : interface(choice) {
    coilA1 = Motor1;
    coilA2 = Motor2;
    coilB1 = Motor3;
    coilB2 = Motor4;
    actuator = pen;
    originX = 0;
    originY = 0;
    maxX = 680;
    maxY = 500;
    currentX = 0;
    currentY = 0;
  }

  // basic methods

  /// @brief Sets steps in x-direction
  /// \param steps number of steps to be made
  void setStepX(int steps);

  /// @brief sets steps in y-direction
  /// \param steps number of steps to be made
  void setStepY(int steps);

  /// @brief sets step in x- and/or y-direction
  /// \param  stepX steps in x-direction
  /// \param  stepY steps in y-direction
  void setStepXY(int stepX, int stepY);

  /// @brief set the origin coordinates
  /// \param origX new x-origin
  /// \param origY new y-origin
  void setOrigin(int origX, int origY);

  /// @brief find the origin using end switches
  /// \param stopX the input pin for the x end switch
  /// \param stopY the input pin for the y end switch
  bool findOrigin(int stopX, int stopY);

  /// @brief overwrite the current plotter position
  /// \param posX new x-position
  /// \param posY new y-position
  void setPosition(int posX, int posY);

  /// @brief set the range for plotter (step) position
  /// \param origX x origin position
  /// \param origY y origin position
  /// \param maxX maximum x plotter position
  /// \param maxY maximum y plotter position
  void setArea(int origX, int origY, int maxX, int maxY);
  
  /// @brief move to position
  /// \param posX target x-position
  /// \param posY target y-position
   void moveToPosition(int posX, int posY);
  
  /// @brief relative number of steps
  /// \param deltaX incremental x-steps
  /// \param deltaY incremental y-steps
  void moveRelative(int deltaX, int deltaY);
  
  /// @brief stop all stepper motors
  /// \param deltaX incremental x-steps
  /// \param deltaY incremental y-steps
  void setSteppersSTOP();

  // utility functions
  /// @brief activate actuator
  void penDown();
  
  /// @brief deactivate actuator
  void penUp();
  
  /// @brief draw line from current position
  /// \param posX target x-position
  /// \param posY target y-position
  void line(int posX, int posY);
 
  /// @brief Send a text message to the display (if connected)
  /// \param 
  /// \param 
  void lineRelative(int posX, int posY);
 
  /// @brief draw circle with center (orX,orY) and r=radius
  /// \param orX origin X
  /// \param orY origin Y
  /// \param radius circle radius
  void circle(int orX, int orY, int radius);
 
  /// @brief draw ellips with center (400,200) and radii 200 and 100
  /// \param orX origin X
  /// \param orY origin Y
  /// \param radiusX ellipse x radius
  /// \param radiusY ellipse x radius
  void ellips(int orX, int orY, int radiusX, int radiusY);
  
  /// @brief draw box at (posX, posY) with size sizX and sizeY
  /// \param posX origin X
  /// \param posY origin Y
  /// \param radiusX ellipse x radius
  /// \param radiusY ellipse x radius
  /// \param hatch not yet implemented
  void box(int posX, int posY, int sizeX, int sizeY, int hatch);
  
  /// @brief draw an x- and y-axis with scales, tickmarks and legends 
  /// \param xPB plot x-origin in plotter coordinates
  /// \param xPE plot x-maximum in plotter coordinates
  /// \param yPB plot y-origin in plotter coordinates
  /// \param yPE plot y-maximum in plotter coordinates
  /// \param xAB plot x-origin in scaled graph coordinates
  /// \param xAE plot x-maximum in scaled graph coordinates
  /// \param yAB plot y-origin in scaled graph coordinates
  /// \param yAE plot y-maximum in scaled graph coordinates
  /// \param label label yes or no
  /// \param xLabel label for x-axis
  /// \param yLabel label for y-axis
  /// \param title title above the plot
  /// \param xInterval x-interval for tickmarks
  /// \param yInterval y-interval for tickmarks
  /// \param ticklength length of the tickmarks in plotter steps
  void axis(int xPB, int xPE, int yPB, int yPE,     // Plot coordinate range B=begin
            int xAB, int xAE, int yAB, int yAE,     // Axis coordinate range E=end
            bool label, const char *xLabel, const char *yLabel, const char *title,  // label ingo
            int xInterval, int yInterval, int ticklength );       // # tickmarks
  
  /// @brief draw curve
  /// \param numberOfPoints number of points
  /// \param curveX array of length numberOfPoints with x-coordinates 
  /// \param curveY array of length numberOfPoints with y-coordinates 
  void curve(int numberOfPoints, int curveX[], int curveY[]);
  
  /// @brief plot a character
  /// \param posX target x-position
  /// \param posY target y-position
  /// \param scale size of the character (1-4)
  /// \param direction direction in which the text is written (1-4)
  /// \param c the ascii character
  void plotChar(int posX, int posY, int scale, int direction, char c);
 
  /// @brief plot a string of characters
  /// \param posX target x-position
  /// \param posY target y-position 
  /// \param scale size of the character (1-4)
  /// \param direction direction in which the text is written (1-4)
  /// \param string the ascii text string
  void plotText(int posX, int posY, int scale, int direction, const char *string);

private:
  // utilify function to set the actual stepper steps
  void stepper(motorDirection Motor1, motorDirection Motor2, motorDirection Motor3);
  
  // set steps and do the accounting
  bool setStepXandY(int stepsX, int stepsY);
 
  // draw a segment
  void drawSegment(int &xFrom, int xTo, int &yFrom, int yTo, int scale, float angle);
};

// section for encoder motors

enum FTencoderMode { E_STD=0, E_INT=1};  // STD: standard 'E' type pulse, INT: arduino interrupt

class FTencoderMotor
{
private:
   FTlegacy* interface;   // the associated interface
   int encoderSignal;
   int encoderMode;
   int motor;
   int sensor;
   int origin;
   int maximum;
   volatile long int encoderPos = 0;
   long int startPos = 0;
   bool running = false;
   bool movingRight;

public:
    /// @brief Constructor
    /// \param choice the associated interface
    /// \param motorID the interface motor index 
    /// \param modeChoice: E_STD (counters) or E_INT (hardware interrupt)
    /// \param sensorID pin for encoder signal
    FTencoderMotor(FTlegacy* choice, int motorID, FTencoderMode modeChoice, int sensorID);

    /// @brief Updates the encoder counter
    void updateEncoder();

private:
   // the encoder motor
   static FTencoderMotor* sEncoder;
  
  // internal function that is called upon a hardware interrupt
  static void updateEncoderISR();
 
  // internal function to set steps using counters
  bool setStepsOLD(int steps);
 
  // internal function to set steps using hardware interrupts
  bool setStepsNEW(int steps);

public:

  /// @brief begin and initialise the encoder motor
  void begin();
 
  /// @brief  set steps and do the accounting
  /// \param steps the number of steps
  bool setSteps(int steps);

  /// @brief set the origin
  /// \param origin new origin
  void setOrigin(int origin);
  
   /// @brief get the current position
   /// \return int the position
  int getPosition();
  
  /// @brief set the range of allowed positions
  /// \param origin the minimum value
  /// \param maximum the maximum value
  void setRange(int origin, int maximum);
  
  /// @brief move to the origin
  /// \return bool true when done 
   bool moveToOrigin();
  
  /// @brief move to a position
  /// \param position
  bool moveToPosition(int position);

  /// @brief incrementally move a number of steps
  /// \param delta the incremental steps
   bool moveRelative(int delta);
  
  /// @brief find the home position
  /// \param the pin of the end switch
  /// \param dir the idrection the motor has to move in
   bool findHome(int endPin, motorDirection dir);
  
  /// @brief activate the motor in clock-wise (CW) position
   void setMotorCW();
  
  /// @brief activate the motor in counter-clock-wise (CCW) position
   void setMotorCCW();
  
  /// @brief deactivate the motor
   void setMotorSTOP();
};

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
/// @brief standardised output channel 1, using half of motor 1
#define ft_O1 1
/// @brief standardised output channel 2, using the other half of motor 1
#define ft_O2 2
/// @brief standardised output channel 3, using half of motor 2
#define ft_O3 3
/// @brief standardised output channel 4, using the other half of motor 2
#define ft_O4 4
/// @brief standardised output channel 5, using half of motor 3
#define ft_O5 5
/// @brief standardised output channel 6, using the other half of motor 3
#define ft_O6 6
/// @brief standardised output channel 7, using half of motor 4
#define ft_O7 7
/// @brief standardised output channel 8, using the other half of motor 4
#define ft_O8 8
/// @brief standardised output channel 9, using half of extension motor 1
#define ft_O9 9
/// @brief standardised output channel 10, using the other half of extension motor 1
#define ft_O10 10
/// @brief standardised output channel 11, using half of extension motor 2
#define ft_O11 11
/// @brief standardised output channel 12, using the other half of extension motor 2
#define ft_O12 12
/// @brief standardised output channel 13, using half of extension motor 3
#define ft_O13 13
/// @brief standardised output channel 14, using the other half of extension motor 3
#define ft_O14 14
/// @brief standardised output channel 15, using half of extension motor 4
#define ft_O15 15
/// @brief standardised output channel 16, using the other half of extension motor 4
#define ft_O16 16

/// @brief standardised global ON for interface outputs
#define ON true
/// @brief standardised global OFF for interface outputs
#define OFF false

#endif

/*

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
