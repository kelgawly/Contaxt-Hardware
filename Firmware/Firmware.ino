// Include Libraries

#include <SparkFun_ADXL345.h>
#include <TinyGPS++.h>

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
//#include <WiFiClient.h>
#include <ArduinoJson.h>

//configuration details for WiFi
const char* ssid = "NETGEAR56";
const char* password = "fancypotato115";

const String firebaseAPIKey = "AIzaSyD3-yBGhRrFQZhDM-1VE3vou5nDvO83Crw";
const String firebaseID = "contaxt-bc929";

const String apiKey = "dev";

//Domain name with URL path or IP address with path
const String serverName = "http://192.168.1.7:8080/postData";
//const String serverName = "http://18.191.219.252:8080/";

unsigned long lastTime = 0;
//how long we wait before timeout
unsigned long timerDelay = 5000;

// Pin Definitions
#define FSRSQUARE_PIN_1	A0

//operating at 3.3 V
const double vcc = 3.3;


// Global variables and defines
int adxlAx, adxlAy, adxlAz;


// object initialization
ADXL345 adxl;
TinyGPSPlus gps;

// define vars for testing menu
const int timeout = 2000000;       //define timeout of 10 sec
char menuOption = 0;
long time0;

void postSystemData(String jsonString) {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;

    // Your Domain name with URL path or IP address with path
    http.begin(client, (serverName + "?key=" + apiKey));


    // Specify content-type header
    http.addHeader("Content-Type", "application/json");
    // Data to send with HTTP POST
    int httpResponseCode = http.POST(jsonString);

    Serial1.print("HTTP Response code: ");
    Serial1.println(httpResponseCode);

    // Free resources
    http.end();
  } else {
    Serial1.println("WiFi has been disconnected");
  }

}

// Setup the essentials for your circuit to work. It runs first every time your circuit is powered with electricity.
void setup()
{
  //used to transmit data from gps to board
  Serial.begin(9600);
  delay(500);
  //GPIO 15 / D8 IS TX
  //GPIO 13 / D7 IS RX
  Serial.swap();

  //used for serial monitor
  Serial1.begin(9600);

  //  Serial1.println();
  //  Serial1.print("MAC: ");
  //  Serial1.println(WiFi.macAddress());

  //connect to wifi
  configureWifi();


  //ADXL345 additional info ->  https://github.com/sparkfun/SparkFun_ADXL345_Arduino_Library/blob/master/examples/SparkFun_ADXL345_Example/SparkFun_ADXL345_Example.ino
  //create accelerometer object
  ADXL345 adxl = ADXL345();
  adxl.powerOn();
  adxl.setRangeSetting(16);
  //  menuOption = menu();




}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop()
{
  postData();
  

}

void postData(){
  StaticJsonDocument<300> doc;
  int analogReadVal = analogRead(FSRSQUARE_PIN_1); // read the FSR pin
  double vOut = analogToVout(analogReadVal); //convert analog read to voltage
  double fsrResistance = vOutToResistance(vOut); //convert read voltage to FSR resistance
  doc.add(fsrResistance);

  double *gpsData = getGPSData();
  doc.add(gpsData[0]);
  doc.add(gpsData[1]);
  doc.add(gpsData[2]);
  doc.add(gpsData[3]);

  adxl.readAccel(&adxlAx, &adxlAy, &adxlAz);
  doc.add(adxlAx);
  doc.add(adxlAy);
  doc.add(adxlAz);
  

  String output;
  serializeJson(doc, output);
  postSystemData(output);
  
  
  
}


void printValsToSerial() {
  //print GPS values
  printGPS();

  //print force square value
  double fsrSquareForce = getForceNew(true);
  Serial1.print(F("Force: ")); Serial1.print(fsrSquareForce); Serial1.println(F(" lbs"));

  //print accelerometer values
  adxl.readAccel(&adxlAx, &adxlAy, &adxlAz);
  // display tab-separated accel x/y/z values
  Serial1.print(F("ADXL345 accel-\t"));
  Serial1.print(adxlAx); Serial1.print(F("\t"));
  Serial1.print(adxlAy); Serial1.print(F("\t"));
  Serial1.println(adxlAz);
}






void printGPS() {
  while (Serial.available() > 0) {
    if (gps.encode(Serial.read())) {

      displayGPSInfo();



    }
  }
}

//configuration for the WiFi
void configureWifi() {
  WiFi.begin(ssid, password);
  Serial1.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial1.print(".");
  }
  Serial1.println("");
  Serial1.print("Connected to WiFi network with IP Address: ");
  Serial1.println(WiFi.localIP());
}

double * getGPSData() {
  /*returns array consisting of 4 elements 
  0 = valid?
  1 = lat
  2=long
  3=alt

  */
  static double data[4];
  while (1) {
    while (Serial.available() > 0) {
      if (gps.encode(Serial.read())) {
        if (gps.location.isValid()){
          data[0] = 1;
          data[1] = gps.location.lat();
          data[2] = gps.location.lng();
          data[3] = gps.altitude.meters();
          return data;
        }
        else
        {
          
          for(int i = 0; i<4; i++){
            data[i] = 0;
          }
          return data;
        }
      }

    }
  }
  return data;
}

//display interface for GPS readings

void displayGPSInfo()
{
  if (gps.location.isValid())
  {
    Serial1.print("Latitude: ");
    Serial1.println(gps.location.lat(), 6);
    Serial1.print("Longitude: ");
    Serial1.println(gps.location.lng(), 6);
    Serial1.print("Altitude: ");
    Serial1.println(gps.altitude.meters());
  }
  else
  {
    Serial1.println("Location: Not Available");
  }

  Serial1.print("Date: ");
  if (gps.date.isValid())
  {
    Serial1.print(gps.date.month());
    Serial1.print("/");
    Serial1.print(gps.date.day());
    Serial1.print("/");
    Serial1.println(gps.date.year());
  }
  else
  {
    Serial1.println("Not Available");
  }

  Serial1.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial1.print(F("0"));
    Serial1.print(gps.time.hour());
    Serial1.print(":");
    if (gps.time.minute() < 10) Serial1.print(F("0"));
    Serial1.print(gps.time.minute());
    Serial1.print(":");
    if (gps.time.second() < 10) Serial1.print(F("0"));
    Serial1.print(gps.time.second());
    Serial1.print(".");
    if (gps.time.centisecond() < 10) Serial1.print(F("0"));
    Serial1.println(gps.time.centisecond());
  }
  else
  {
    Serial1.println("Not Available");
  }

  Serial1.println();
  Serial1.println();


}


//helper for FSR square

double getResistance() {
  double res = 10000; //using 10k resistor
  double Vcc = 3.3; //operates at 3.3V

  double senVoltage = analogRead(FSRSQUARE_PIN_1) * Vcc / 1023;
  return res * (Vcc / senVoltage - 1);
}

/**
   Converte resistance to force using curve from data sheet [in grams]
*/
double analogToVout(int analogReadValue) {

  return analogReadValue * vcc / 1023; //convert analog read value parameter to volts, can also use map function
}

double vOutToResistance(double vOut) {
  int refResistorVal = 10000; //we are currently using a 10k resistor
  return (refResistorVal * vcc / vOut) - 1;
}

double resistanceToForce(double resistanceFSR) {
  double kOhmResistance = resistanceFSR / 1000; //equation works in kohms
  return 1285.197 * pow(kOhmResistance, -1.41);
}


double getForceNew(bool inPounds) {
  int analogReadVal = analogRead(FSRSQUARE_PIN_1); // read the FSR pin
  double vOut = analogToVout(analogReadVal); //convert analog read to voltage
  double fsrResistance = vOutToResistance(vOut); //convert read voltage to FSR resistance
  double fsrForce = resistanceToForce(fsrResistance); //get force applied to sensor in grams
  if (inPounds) {
    fsrForce = gTolb(fsrForce);
  }
  return fsrForce;

}
double getForce()
{
  double resistance = getResistance();

  //calculate force using curve broken into two parts of different slope
  if (resistance <= 600)
    return (1.0 / resistance - 0.00075) / 0.00000032639 / pow(3.81, 2);
  else
    return (1.0 / resistance)  / 0.000000642857 / pow(3.81, 2);
}
double gTolb(double g) {
  return g * 0.00220462;
}

/*******************************************************

     Circuito.io is an automatic generator of schematics and code for off
     the shelf hardware combinations.

     Copyright (C) 2016 Roboplan Technologies Ltd.

     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.

     In addition, and without limitation, to the disclaimers of warranties
     stated above and in the GNU General Public License version 3 (or any
     later version), Roboplan Technologies Ltd. ("Roboplan") offers this
     program subject to the following warranty disclaimers and by using
     this program you acknowledge and agree to the following:
     THIS PROGRAM IS PROVIDED ON AN "AS IS" AND "AS AVAILABLE" BASIS, AND
     WITHOUT WARRANTIES OF ANY KIND EITHER EXPRESS OR IMPLIED.  ROBOPLAN
     HEREBY DISCLAIMS ALL WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT
     NOT LIMITED TO IMPLIED WARRANTIES OF MERCHANTABILITY, TITLE, FITNESS
     FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT, AND THOSE ARISING BY
     STATUTE OR FROM A COURSE OF DEALING OR USAGE OF TRADE.
     YOUR RELIANCE ON, OR USE OF THIS PROGRAM IS AT YOUR SOLE RISK.
     ROBOPLAN DOES NOT GUARANTEE THAT THE PROGRAM WILL BE FREE OF, OR NOT
     SUSCEPTIBLE TO, BUGS, SECURITY BREACHES, OR VIRUSES. ROBOPLAN DOES
     NOT WARRANT THAT YOUR USE OF THE PROGRAM, INCLUDING PURSUANT TO
     SCHEMATICS, INSTRUCTIONS OR RECOMMENDATIONS OF ROBOPLAN, WILL BE SAFE
     FOR PERSONAL USE OR FOR PRODUCTION OR COMMERCIAL USE, WILL NOT
     VIOLATE ANY THIRD PARTY RIGHTS, WILL PROVIDE THE INTENDED OR DESIRED
     RESULTS, OR OPERATE AS YOU INTENDED OR AS MAY BE INDICATED BY ROBOPLAN.
     YOU HEREBY WAIVE, AGREE NOT TO ASSERT AGAINST, AND RELEASE ROBOPLAN,
     ITS LICENSORS AND AFFILIATES FROM, ANY CLAIMS IN CONNECTION WITH ANY OF
     THE ABOVE.
********************************************************/





// Menu function for selecting the components to be tested
// Follow serial monitor for instrcutions

void oldMenuTesting() {
  if (menuOption == '1') {

    adxl.powerOn();                     // Power on the ADXL345

    adxl.setRangeSetting(16);           // Give the range settings
    // Accepted values are 2g, 4g, 8g or 16g
    // Higher Values = Wider Measurement Range
    // Lower Values = Greater Sensitivity
    adxl.readAccel(&adxlAx, &adxlAy, &adxlAz);
    // display tab-separated accel x/y/z values
    Serial.print(F("ADXL345 accel-\t"));
    Serial.print(adxlAx); Serial.print(F("\t"));
    Serial.print(adxlAy); Serial.print(F("\t"));
    Serial.println(adxlAz);

  }
  else if (menuOption == '2') {
    // Force Sensitive Resistor - Square - Test Code
    // Read FSR resistance value. try also fsrSquare.getResistance()
    // For more information see Sparkfun website - www.sparkfun.com/products/9375
    // Note, the default Vcc and external resistor values for FSR calculations are 5V ang 3300Okm, if you are not
    //       using these default valuse in your circuit go to FSR.cpp and change default values in FSR constructor
    double fsrSquareForce = getForceNew(true);
    Serial.print(F("Force: ")); Serial.print(fsrSquareForce); Serial.println(F(" lbs"));

  }
  else if (menuOption == '3')
  {
    // Disclaimer: The Ublox NEO-6M GPS Module is outdated, we might need to buy the 8M for continued support
    // for more information visit: https://lastminuteengineers.com/neo6m-gps-arduino-tutorial/
    //print out 10 readings from the GPS module
    // returns NMEA sentences which need to be parsed, visit the site above and section "Parsing NMEA Sentences" for more information
    //TinyGPS++ seems to be a good library for doing this

  }



  if (millis() - time0 > timeout)
  {
    menuOption = menu();
  }

}


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

      if (c == '1')
        Serial.println(F("Now Testing SparkFun ADXL345 - Triple Axis Accelerometer Breakout"));
      else if (c == '2')
        Serial.println(F("Now Testing Force Sensitive Resistor - Square"));
      else if (c == '3')
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
