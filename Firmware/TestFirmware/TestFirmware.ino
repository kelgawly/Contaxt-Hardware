
// Include Libraries

#include <SparkFun_ADXL345.h>
#include <TinyGPS++.h>

unsigned long lastTime = 0;
//how long we wait before timeout
unsigned long timerDelay = 5000;

// Pin Definitions
#define FSRSQUARE_PIN_1  A0

//operating at 3.3 V
const double vcc = 3.3;

// object initialization
ADXL345 adxl;
TinyGPSPlus gps;

// define vars for testing menu
const int timeout = 2000000;       //define timeout of 10 sec
char menuOption = 0;
long time0;


void setup() {
  // put your setup code here, to run once:
  //used to transmit data from gps to board
  Serial.begin(9600);
  delay(500);
  //GPIO 15 / D8 IS TX
  //GPIO 13 / D7 IS RX
  Serial.swap();

  //used for serial monitor
  Serial1.begin(9600);

  //ADXL345 additional info ->  https://github.com/sparkfun/SparkFun_ADXL345_Arduino_Library/blob/master/examples/SparkFun_ADXL345_Example/SparkFun_ADXL345_Example.ino
  //create accelerometer object
  ADXL345 adxl = ADXL345();
  adxl.powerOn();
  adxl.setRangeSetting(16);
}

void loop() {
  // put your main code here, to run repeatedly:
  printValsToSerial();
}




void printValsToSerial() {
  //print GPS values
  printGPS();

  //print force square value
  double fsrSquareForce = getForceNew(true);
  Serial1.print(F("Force: ")); Serial1.print(fsrSquareForce); Serial1.println(F(" lbs"));
  int adxlAx, adxlAy, adxlAz;
  //print accelerometer values
  adxl.readAccel(&adxlAx, &adxlAy, &adxlAz);
  // display tab-separated accel x/y/z values
  Serial1.print(F("ADXL345 accel-\t x: "));
  Serial1.print(adxlAx); Serial1.print(F("\ty: "));
  Serial1.print(adxlAy); Serial1.print(F("\tz: "));
  Serial1.println(adxlAz);
}







void printGPS() {
  displayGPSInfo();
  while (Serial.available() > 0) {
    gps.encode(Serial.read());


    }
  
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
// Menu function for selecting the components to be tested
// Follow serial monitor for instrcutions

void oldMenuTesting() {
  if (menuOption == '1') {

    adxl.powerOn();                     // Power on the ADXL345

    adxl.setRangeSetting(16);           // Give the range settings
    // Accepted values are 2g, 4g, 8g or 16g
    // Higher Values = Wider Measurement Range
    // Lower Values = Greater Sensitivity
    int adxlAx, adxlAy, adxlAz;
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
