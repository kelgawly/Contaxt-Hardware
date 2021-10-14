// Include Libraries
#include "Arduino.h"
#include "ADXL345.h"
#include "I2Cdev.h"
#include "FSR.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Pin Definitions
#define FSRSQUARE_PIN_1	A0
#define GPS_PIN_TX	11
#define GPS_PIN_RX	10



// Global variables and defines
int16_t adxlAx, adxlAy, adxlAz;
TinyGPSPlus gps;

// object initialization
ADXL345 adxl;
FSR fsrSquare(FSRSQUARE_PIN_1);


//Default baud of NEO-6M is 9600
int GPSBaud = 9600;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(GPS_PIN_RX, GPS_PIN_TX);


// define vars for testing menu
const int timeout = 10000;       //define timeout of 10 sec
char menuOption = 0;
long time0;

// Setup the essentials for your circuit to work. It runs first every time your circuit is powered with electricity.
void setup() 
{
    // Setup Serial which is useful for debugging
    // Use the Serial Monitor to view printed messages
    Serial.begin(9600);
    while (!Serial) ; // wait for serial port to connect. Needed for native USB
    Serial.println("start");
    
    adxl.init();
    menuOption = menu();
    
}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop() 
{
    
    
    if(menuOption == '1') {
    // SparkFun ADXL345 - Triple Axis Accelerometer Breakout - Test Code
    // read raw accel measurements from device
    adxl.getAcceleration(&adxlAx, &adxlAy, &adxlAz);
    // display tab-separated accel x/y/z values
    Serial.print(F("ADXL345 accel-\t")); 
    Serial.print(adxlAx); Serial.print(F("\t"));
    Serial.print(adxlAy); Serial.print(F("\t"));  
    Serial.println(adxlAz);

    }
    else if(menuOption == '2') {
    // Force Sensitive Resistor - Square - Test Code
    // Read FSR resistance value. try also fsrSquare.getResistance()
    // For more information see Sparkfun website - www.sparkfun.com/products/9375
    // Note, the default Vcc and external resistor values for FSR calculations are 5V ang 3300Okm, if you are not 
    //       using these default valuse in your circuit go to FSR.cpp and change default values in FSR constructor
    float fsrSquareForce = fsrSquare.getForce();
    Serial.print(F("Force: ")); Serial.print(fsrSquareForce); Serial.println(F(" [g]"));

    }
    else if(menuOption == '3')
    {
    // Disclaimer: The Ublox NEO-6M GPS Module is outdated, we might need to buy the 8M for continued support
    // for more information visit: https://lastminuteengineers.com/neo6m-gps-arduino-tutorial/
    //print out 10 readings from the GPS module
    // returns NMEA sentences which need to be parsed, visit the site above and section "Parsing NMEA Sentences" for more information
    //TinyGPS++ seems to be a good library for doing this
    int counter = 0;
    while (gpsSerial.available() > 0){
    if (gps.encode(gpsSerial.read())){
      counter ++;
      displayInfo();
      if(counter >= 10){
        break;
      }
    }
    }

    // If 5000 milliseconds pass and there are no characters coming in
    // over the software serial port, show a "No GPS detected" error
    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println("No GPS detected");
      while(true);
    }
    }
    
    if (millis() - time0 > timeout)
    {
        menuOption = menu();
    }
    
}



// Menu function for selecting the components to be tested
// Follow serial monitor for instrcutions
char menu()
{

    Serial.println(F("\nWhich component would you like to test?"));
    Serial.println(F("(1) SparkFun ADXL345 - Triple Axis Accelerometer Breakout"));
    Serial.println(F("(2) Force Sensitive Resistor - Square"));
    Serial.println(F("(3) Ublox NEO-6M GPS Module"));
    Serial.println(F("(menu) send anything else or press on board reset button\n"));
    while (!Serial.available());

    // Read data from serial monitor if received
    while (Serial.available()) 
    {
        char c = Serial.read();
        if (isAlphaNumeric(c)) 
        {   
            
            if(c == '1') 
    			Serial.println(F("Now Testing SparkFun ADXL345 - Triple Axis Accelerometer Breakout"));
    		else if(c == '2') 
    			Serial.println(F("Now Testing Force Sensitive Resistor - Square"));
    		else if(c == '3') 
    			Serial.println(F("Now Testing Ublox NEO-6M GPS Module"));
            else
            {
                Serial.println(F("illegal input!"));
                return 0;
            }
            time0 = millis();
            return c;
        }
    }
}


//display interface for GPS readings
void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
  }
  else
  {
    Serial.println("Location: Not Available");
  }
  
  Serial.print("Date: ");
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.println();
  Serial.println();
  delay(1000);
}

/*******************************************************

*    Circuito.io is an automatic generator of schematics and code for off
*    the shelf hardware combinations.

*    Copyright (C) 2016 Roboplan Technologies Ltd.

*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.

*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.

*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*    In addition, and without limitation, to the disclaimers of warranties 
*    stated above and in the GNU General Public License version 3 (or any 
*    later version), Roboplan Technologies Ltd. ("Roboplan") offers this 
*    program subject to the following warranty disclaimers and by using 
*    this program you acknowledge and agree to the following:
*    THIS PROGRAM IS PROVIDED ON AN "AS IS" AND "AS AVAILABLE" BASIS, AND 
*    WITHOUT WARRANTIES OF ANY KIND EITHER EXPRESS OR IMPLIED.  ROBOPLAN 
*    HEREBY DISCLAIMS ALL WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT 
*    NOT LIMITED TO IMPLIED WARRANTIES OF MERCHANTABILITY, TITLE, FITNESS 
*    FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT, AND THOSE ARISING BY 
*    STATUTE OR FROM A COURSE OF DEALING OR USAGE OF TRADE. 
*    YOUR RELIANCE ON, OR USE OF THIS PROGRAM IS AT YOUR SOLE RISK.
*    ROBOPLAN DOES NOT GUARANTEE THAT THE PROGRAM WILL BE FREE OF, OR NOT 
*    SUSCEPTIBLE TO, BUGS, SECURITY BREACHES, OR VIRUSES. ROBOPLAN DOES 
*    NOT WARRANT THAT YOUR USE OF THE PROGRAM, INCLUDING PURSUANT TO 
*    SCHEMATICS, INSTRUCTIONS OR RECOMMENDATIONS OF ROBOPLAN, WILL BE SAFE 
*    FOR PERSONAL USE OR FOR PRODUCTION OR COMMERCIAL USE, WILL NOT 
*    VIOLATE ANY THIRD PARTY RIGHTS, WILL PROVIDE THE INTENDED OR DESIRED
*    RESULTS, OR OPERATE AS YOU INTENDED OR AS MAY BE INDICATED BY ROBOPLAN. 
*    YOU HEREBY WAIVE, AGREE NOT TO ASSERT AGAINST, AND RELEASE ROBOPLAN, 
*    ITS LICENSORS AND AFFILIATES FROM, ANY CLAIMS IN CONNECTION WITH ANY OF 
*    THE ABOVE. 
********************************************************/
