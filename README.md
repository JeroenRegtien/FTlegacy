# FTlegacy
fischertechnik C++ library for Arduino

Arduino Library to control legacy parallel and serial fischertechnik(r) computing interfaces.
 * Original code by Jeroen Regtien, December 2023 with revisions since.
 * 
 * @details
 * Supported interfaces:
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
