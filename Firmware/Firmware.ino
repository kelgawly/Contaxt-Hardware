// Include Libraries

#include <SparkFun_ADXL345.h>
#include <TinyGPS++.h>

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
//#include <WiFiClient.h>
#include <ArduinoJson.h>
#include<WiFiClientSecure.h>


//configuration details for WiFi
const char* ssid = "SSID";
const char* password = "PASSWORD";

const String firebaseAPIKey = "AIzaSyD3-yBGhRrFQZhDM-1VE3vou5nDvO83Crw";
const String firebaseID = "contaxt-bc929";
const char fingerprint[] PROGMEM  = "D7 01 00 B5 C3 A7 F2 0D C7 4F C4 39 80 16 DD 58 4E E9 62 86";

// Pin Definitions
#define FSRSQUARE_PIN_1	A0

//operating at 3.3 V
const double vcc = 3.3;

// object initialization
ADXL345 adxl;
TinyGPSPlus gps;

unsigned short i = 0;

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




}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop()
{
  postData();
}

//MARK - main system logic

void postData() {

  readGPSData();
  
  //if year is equal to 2000, gps time has not been set
  if (gps.date.year() != 2000) {
    
    StaticJsonDocument<512> doc;

    JsonObject fields = doc.createNestedObject("fields");

    int adxlAx, adxlAy, adxlAz;
    adxl.readAccel(&adxlAx, &adxlAy, &adxlAz);
    JsonObject fields_accelerometer_mapValue_fields = fields["accelerometer"]["mapValue"].createNestedObject("fields");
    fields_accelerometer_mapValue_fields["z"]["integerValue"] = adxlAz;
    fields_accelerometer_mapValue_fields["x"]["integerValue"] = adxlAx;
    fields_accelerometer_mapValue_fields["y"]["integerValue"] = adxlAy;


    JsonObject fields_gps_mapValue_fields = fields["gps"]["mapValue"].createNestedObject("fields");
    fields_gps_mapValue_fields["longitude"]["doubleValue"] = gps.location.lat();
    fields_gps_mapValue_fields["latitude"]["doubleValue"] = gps.location.lng();
    fields_gps_mapValue_fields["altitude"]["doubleValue"] = gps.altitude.meters();



    fields["fsrResistance"]["doubleValue"] = getFSRResistance();

    Serial1.println(getTime());
    fields["timestamp"]["timestampValue"] = getTime(); //needs to be in following format: yyyy-MM-ddTHH:mm:ssZ
    fields["counter"]["integerValue"] = i;

    String output;
    serializeJson(doc, output);
    postSystemData(output);
    i++;
  }



}

void postSystemData(String jsonString) {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClientSecure client;
    client.setFingerprint(fingerprint);

    HTTPClient http;
    String url = "https://firestore.googleapis.com/v1/projects/" + firebaseID + "/databases/(default)/documents/hardware/deviceReadings/" + WiFi.macAddress() + "?key=" + firebaseAPIKey;
    // Your Domain name with URL path or IP address with path
    http.begin(client, url);


    // Specify content-type header
    http.addHeader("Content-Type", "application/json");
    // Data to send with HTTP POST
    Serial1.print("HTTP Response: ");
    Serial1.println(http.POST(jsonString));




    // Free resources
    http.end();
  } else {
    Serial1.println("WiFi has been disconnected");
  }

}

//MARK - GPS functions
void readGPSData() {
  while (Serial.available() > 0) {
    gps.encode(Serial.read());
  }

}

String getTime() {
  //returns gps time in yyyy-MM-ddTHH:mm:ssZ format
  String dateStr = "";
  dateStr += gps.date.year();
  dateStr += "-";
  if (gps.date.month() < 10 ) {
    dateStr += "0";
  }
  dateStr += gps.date.month();
  dateStr += "-";
  if (gps.date.day() < 10 ) {
    dateStr += "0";
  }
  dateStr += gps.date.day();
  dateStr += "T";

  if (gps.time.hour() < 10 ) {
    dateStr += "0";
  }
  dateStr += gps.time.hour();
  dateStr += ":";

  if (gps.time.minute() < 10 ) {
    dateStr += "0";
  }
  dateStr += gps.time.minute();
  dateStr += ":";

  if (gps.time.second() < 10 ) {
    dateStr += "0";
  }
  dateStr += gps.time.second();
  dateStr += "Z";
  return dateStr; 
}


//MARK - FSR functions
double getFSRResistance() {
  int analogReadVal = analogRead(FSRSQUARE_PIN_1); // read the FSR pin
  double vOut = analogToVout(analogReadVal); //convert analog read to voltage
  return vOutToResistance(vOut); //convert read voltage to FSR resistance
}
double analogToVout(int analogReadValue) {

  return analogReadValue * vcc / 1023; //convert analog read value parameter to volts, can also use map function
}

double vOutToResistance(double vOut) {
  int refResistorVal = 10000; //we are currently using a 10k resistor
  return (refResistorVal * vcc / vOut) - 1;
}


//MARK - configuration for the WiFi
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
