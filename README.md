# FTlegacy
 Arduino Library to control legacy parallel and serial fischertechnik(r) computing interfaces.
 author Jeroen Regtien
 date  December 2023 with revisions since.
  
 Supported parallel interfaces: Universal (30520), CVK (66843), Centronics (30566)
 Supported serial interfaces: Intelligent (30402), ROBO (93293)
  
 Supported PLCs: none
 
 Supported Shields: none
  
 Supported Arduino's: UNO R3, UNO R4 Minima/Wifi, MEGA, Nano
 
 The library consists of two files:
  1) FTlegacy.h - header file for the FTlegacy, FTcontroller, FTstepper, FTencoder and FTtimer classes 
  2) FTlegacy.cpp - C++ implementation of class, methods and utility functions
 
 References:
 
       https://www.ftcommunity.de/ftpedia/2014/2014-1/ftpedia-2014-1.pdf
       
       https://www.ftcommunity.de/ftpedia/2017/2017-2/ftpedia-2017-2.pdf
       
       https://www.ftcommunity.de/ftpedia/2017/2017-3/ftpedia-2017-3.pdf
       
       https://www.ftcommunity.de/ftpedia/2017/2017-4/ftpedia-2017-4.pdf
       
       https://www.ftcommunity.de/ftpedia/2023/2023-4/ftpedia-2023-4.pdf
       
       https://www.ftcommunity.de/ftpedia/2025/2025-2/ftpedia-2025-2.pdf

       https://www.ftcommunity.de/ftpedia/2025/2025-4/ftpedia-2025-4.pdf
 
 The MIT License (MIT)
  
 Copyright (c) 2023-2025 Jeroen Regtien
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 
    Version 0.6 - Added LCD 2004 option.
    Version 0.7 - Serial extension capability, code cleanup, add actuators, lamps.
    Version 0.8 - Used for several workstations. Actuators removed as object.
    Version 0.9 - Parallel Extensions added. Analog input via parallel interface now works.
                Consolidated and rationalised. Given Arduino UNO memory limitations,
                two separate classes introduced: FTlegacy for legacy fischertechnik
                interfaces and FTmodule also including 3rd party shields and controllers.
                Version after many prototype tests.
    Version 1.0 - First operational release. Added FTstepper(XY) and FTencoder classes

    
