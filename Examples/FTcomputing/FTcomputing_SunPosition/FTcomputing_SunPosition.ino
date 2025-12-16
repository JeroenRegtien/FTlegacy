/*******************************************************************************************
 * fischertechnik CVK Computing box with parallel interface. 
 * Using FTlegacy library
 * 
 * Model: Sunposition
 *
 * Copyright (c) 2025 Jeroen Regtien
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
 **********************************************************************************************/

// function:
// get timestamp (year, day, hour, min, time-zone, daylightsaving))
// get location (latitude, longitude)
//
// move panel to centerposition South
//
// requires LCD 2004

#include <FTlegacy.h>

const char* programName = "Sun Position";
FTlegacy interface(PAR, 1);
FTcontroller controller(D2004);

int userInput();

unsigned long timeIncrement = 1;     // timer increment in minutes
FTtimer pauze(timeIncrement * 250);  // update timer every 'timeIncrement' minutes.  // add *60

// define constants for inputs and outputs
int MOTAZ = ft_M1;
int MOTEL = ft_M2;

int S_UP = ft_E3;
int S_DOWN = ft_E4;
int S_OK = ft_E6;

const float RAD = 0.0174533;

float latitude = 38.538;    // degrees north
float longitude = 121.758;  //  121.758; // degrees west;

// date and time settings
int year = 1977;
int day = 120;  // april 30
int hour = 12;
int min = 0;
int sec = 0;
int zone = 7;
int daylightSaving = 0;
// output
float azimuth;
float elevation;

// the constants below are calibrated by moving the panel to the
// end conditions and reading
int xMin = 25;   // minimum Ex reading possible
int xMax = 185;    // maximum Ex reading possible
int yMin = 119;   // minimum Ey reading possible
int yMax = 242;   // maximum Ey reading possible
int aMin = -180;  // minimum possible azimuth angle
int aMax = 180;   // maximum possible azimuth angle
int eMin = 0;     // minimum possible elevation angle
int eMax = 90;    // maximum possible elevation angle
;
void setup() {

  int go_on = true;
  Serial.begin(9600);
  controller.begin(programName);
  delay(500);

  interface.begin();
  // interface.setAllMotorsSTOP();

  controller.ftMessageToDisplay(1, 1, "starting...", true);

  while (go_on) {
    if (userInput()) {
      go_on = false;
    };
  }
  controller.ftMessageToDisplay(0, 1, "Starting Calculation", true);
  // Serial.println("Hour  Min   Azimuth  Elevation");
  pauze.reset();
  interface.setAllMotorsSTOP();
}

void loop() {

  interface.getInputs();  // always need getInputs, otherwise timeout
  interface.getAnalogInputs();
  // interface.printInputBuffer();

  if (pauze.ready()) {
    incrementTime(10);
    // calculate sun position
    printTime();
    sunPosition(year, day, hour, min, zone, daylightSaving);  // calculate new sun position
    movePanel(azimuth, elevation);                            // move the panel
    pauze.reset();                                            // reset the timer
  }
}

void printTime() {
  Serial.print(F("Date:"));
  Serial.print(day);
  Serial.print(F(" "));
  Serial.print(year);
  Serial.print(F(" time: "));
  Serial.print(hour);
  Serial.print(F(":"));
  Serial.print(min);
  Serial.print(F(" Out:"));
}
// increment global minutes by increament and update hours / days if necessary
void incrementTime(int increment) {
  min += increment;

  if (min > 59) {
    hour += 1;
    min = 0;
  }
  if (hour > 24) {
    day += 1;
    hour = 0;
  }
  if (day > 364) {
    year += 1;
    day = 1;
  }
}
// the routine that calculates global variables azimuth and elevation from
// latitude, longitude and time stamp
void sunPosition(int year, int day, int hour, int min, int zone, int daylightSaving) {
  // input
  int sec = 0;

  // intermediate
  int deltaYear;
  int leap;
  float hours;
  float days;
  float declination;
  float theta;
  float sunLongitude;
  float earthAnomaly;
  float epsilon;
  float ascension;
  float siderialTime;
  float phi;
  float S;
  float H;

  deltaYear = year - 1980;
  leap = deltaYear / 4;

  hours = float(hour) + float((min + float(sec) / 60) / 60);
  days = deltaYear * 365 + leap + day - 1 + hours / 24;


  if (deltaYear == leap * 4) {
    days = days - 1;
  } else if (deltaYear < 0) {
    days = days - 1;
  }

  theta = 2 * PI * days / 365.25;
  // theta =  2*PI * 120. / 365.25;
  earthAnomaly = -0.031271 - (4.53963e-7 * days) + theta;
  sunLongitude = 4.900968 + 3.67474e-7 * days + (0.033434 - 2.3e-9 * days) * sin(earthAnomaly) + 0.000349 * sin(2 * earthAnomaly) + theta;
  epsilon = RAD * (23.4420 - 3.56e-7 * days);

  ascension = atan(sin(sunLongitude) * cos(epsilon) / cos(sunLongitude));
  if (ascension < 0) {
    ascension += 2 * PI;
  }

  declination = asin(sin(sunLongitude) * sin(epsilon));

  siderialTime = 1.759335 + 2 * PI * (days / 365.25 - deltaYear) + 3.694e-7 * days;
  // siderialTime = 6.720165 + 24 * (days/365.25 - deltaYear) + 14.11e-7 * days;
  if (siderialTime >= 2 * PI) {
    siderialTime -= 2 * PI;
  }

  S = siderialTime - longitude * RAD + 1.0027379 * (zone - daylightSaving + hours) * 15 * RAD;
  if (S >= 2 * PI) {
    S -= 2 * PI;
  }

  H = ascension - S;
  phi = latitude * RAD;
  elevation = asin(sin(phi) * sin(declination) + cos(phi) * cos(declination) * cos(H));
  azimuth = asin(cos(declination) * sin(H) / cos(elevation)) / RAD;

  if (sin(elevation) < sin(declination) / sin(phi)) {
    if (azimuth < 0) {
      azimuth += 360;
    }
    azimuth = 180 - azimuth;
  }
  elevation = elevation / RAD;

  /*
  Serial.print(hour);
  Serial.print(F("    "));
  Serial.print(min);
  Serial.print(F("    "));
  Serial.print(azimuth);
  Serial.print(F("    "));
  Serial.print(elevation);
  Serial.print(F("    "));
  */
}

int userInput() {

  int month = 0;
  int correct = 1;

  latitude = manualInput(latitude, "set latitude:");
  longitude = manualInput(longitude, "set long:");
  year = manualInput(year, "set year:");
  month = manualInput(month, "set month:");
  day = manualInput(day, "set day:");
  day = day + monthToDay(month);
  hour = manualInput(hour, "set hour:");
  zone = manualInput(hour, "set timezone:");
  daylightSaving = manualInput(daylightSaving, "daylight sav:");

  correct = manualInput(correct, "correct input:");

  return (correct);
}

int monthToDay(int month) {

  int days = 0;
  int daysMonth[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

  for (int i = 0; i < month - 1; i++) {
    days = days + daysMonth[i];
  }
  return (days);
}

int manualInput(int parameter, char* message) {
  // E1 = up
  // E2 = down
  // E3 = accept
  char cArray[6];
  itoa(parameter, cArray, 10);
  controller.ftMessageToDisplay(1, 1, message, true);
  controller.ftMessageToDisplay(15, 1, cArray, false);
  interface.getInputs();
  delay(150);
  while (!interface.getInput(S_OK)) {
    interface.getInputs();
    if (interface.getInput(S_UP)) {
      parameter += 1;
    } else if (interface.getInput(S_DOWN)) {
      parameter -= 1;
    }
    itoa(parameter, cArray, 10);
    controller.ftMessageToDisplay(15, 1, cArray, false);
    delay(150);
  }
  return (parameter);
}

bool movePanel(float azimuth, float elevation) {

  int targetX;
  int targetY;
  int iAX;
  int iAY;
  bool go_on = true;

  // map hour into Ex
  // long map(long x, long in_min, long in_max, long out_min, long out_max);
  targetY = map(long(azimuth), long(aMin), long(aMax), long(xMin), long(xMax));
  targetX = map(long(elevation), long(eMin), long(eMax), long(yMin), long(yMax));

  Serial.print(" azimuth : ");
  Serial.print(azimuth);
  Serial.print(" targetY : ");
  Serial.print(targetY);
  Serial.print(" elevation : ");
  Serial.print(elevation);
  Serial.print(" targetX : ");
  Serial.println(targetX);

  interface.setAllMotorsSTOP();
  if (elevation > 0) {
    while (go_on) {
      interface.getInputs();
      interface.getAnalogInputs();
      iAX = interface.getAnalogX();
        // Serial.println(iAX);
      if (abs(iAX - targetX) > 3) {
        if (iAX > targetX) {
          interface.setMotorCCW(MOTEL);
          // Serial.println(F("elevation CCW "));
        } else if (targetX > iAX) {
          interface.setMotorCW(MOTEL);
          // Serial.println(F("elevation CW "));
        }
      } else {
        delay(1);
        interface.setMotorSTOP(MOTEL);
        //  Serial.println(" STOP ");
        go_on = false;
      }
    }

    // only rotate for azimuth if elevation above the horizon
    if (targetY > 0) {
      go_on = true;
      while (go_on) {
        interface.getInputs();
        interface.getAnalogInputs();
        iAY = interface.getAnalogY();
        // Serial.println(iAY);
        if (abs(iAY - targetY) > 3) {
          if (iAY > targetY) {
            interface.setMotorCCW(MOTAZ);
             // Serial.println(F(" azimuth CCW "));
          } else if (targetY > iAY) {
            interface.setMotorCW(MOTAZ);
            // Serial.println(F(" azimuth CW "));
          }
        } else {
          delay(1);
          interface.setMotorSTOP(MOTAZ);
          //  Serial.println(" STOP ");
          go_on = false;
        }
      }
    }
  }
}