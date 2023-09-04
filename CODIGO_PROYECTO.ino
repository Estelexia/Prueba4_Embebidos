// Tested and compiled with no errors
// measuring AC current using ACS712 current sensor with ESP32 Microcontroller
// The ACS712 works with high voltage AC so be careful !
// source - /www.circuitschools.com
 //FIREBASE
 #include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
#include <Wire.h>

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "Jesús"
#define WIFI_PASSWORD "Ejesus19"

// Insert Firebase project API Key
#define API_KEY "AIzaSyD58wL4qY-ODzIjwWkrXDtktbwAOnhXD04"

// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "andresville01@hotmail.es"
#define USER_PASSWORD "123abc"

// Insert RTDB URLefine the RTDB URL
#define DATABASE_URL "pruebafireesp-default-rtdb.firebaseio.com"

// Define Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variable to save USER UID
String uid;

// Variables to save database paths
String databasePath;
String PRUEBAPath;
String WattPath;
String FPPath;
String KILOWHPath;


// Timer variables (send new readings every three minutes)
unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 5000;
//variable sensors
const int sensorIn = 39;      // pin where the OUT pin from sensor is connected on Arduino
int mVperAmp = 110;           // this the 5A version of the ACS712 -use 100 for 20A Module and 66 for 30A Module
int Watt = 0;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;
 double TimeFp=0;
 int WattH = 0;
 double WattsTotal=0;
 double WattsHTotal=0;
 double tstar=0;
 double VoltsRMS=0;
 double Voltage2=0;
 double timesr=0;
 double timest=0;
 double Fp=0.95;
 const byte interruptPin1 = 32;
 const byte interruptPin2 = 33;
/*****************************/

// Initialize WiFi
void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
  }
}

 
// Write float values to the database
void sendFloat(String path, float value){
  if (Firebase.RTDB.setFloat(&fbdo, path.c_str(), value)){
  }
  else {
  }
}


void setup() {
  Serial.begin (115200); 
  pinMode(interruptPin1, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), timerisestart, HIGH);
  pinMode(interruptPin2, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), timerisestop, HIGH);
  
  // Initialize BME280 sensor
  //initBME();
  initWiFi();

  // Assign the api key (required)
  config.api_key = API_KEY;

  // Assign the user sign in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  // Assign the RTDB URL (required)
  config.database_url = DATABASE_URL;

  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096);

  // Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  // Assign the maximum retry of token generation
  config.max_token_generation_retry = 5;

  // Initialize the library with the Firebase authen and config
  Firebase.begin(&config, &auth);

  while ((auth.token.uid) == "") {
  }
  // Print user UID
  uid = auth.token.uid.c_str();


  // Update database path
  databasePath = "/UsersData/" + uid;

  // Update database path for sensor readings
  PRUEBAPath = databasePath +"PruebaAPPEmbebidos" + "/hola"; // --> UsersData/<user_uid>/temperature
  FPPath = databasePath +"/PruebaAPPEmbebidos"+ "/FactorPotencia"; // --> UsersData/<user_uid>/humidity
  KILOWHPath = databasePath +"/PruebaAPPEmbebidos"+ "/KILOWATTHORA"; // --> UsersData/<user_uid>/pressure
  WattPath = databasePath +"/PruebaAPPEmbebidos" + "/WATT";

  
}
 
void loop() {
  /********************************************************************/
  getVPP(39,36);
  
  VRMS = (((Voltage/2.0) *(1/sqrt(2)))-0.05);   //root 2 is 0.707
  AmpsRMS = ((VRMS * 1000)/mVperAmp)+0.3; //0.3 is the error I got for my sensor
  AmpsRMS= constrain(AmpsRMS,0,1000);
  
  Voltage2 = constrain(Voltage2,0,1000);
  VoltsRMS = Voltage2*410; //0.3 is the error I got for my sensor
  Watt=Watt+VoltsRMS*AmpsRMS*Fp;
  WattH=Watt*5/36;
  mandardar();

  /******************************************************************/
  float S=1;
  // note: 1.2 is my own empirically established calibration factor
// as the voltage measured at D34 depends on the length of the OUT-to-D34 wire
// 240 is the main AC power voltage – this parameter changes locally
  
  // Send new readings to database
  if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    
    WattsTotal=WattsTotal+Watt;
    WattsHTotal=WattsHTotal+WattH;
    // Send readings to database:
    sendFloat(PRUEBAPath,S);
    sendFloat(WattPath, Watt);
    sendFloat(FPPath, Fp);
    sendFloat(KILOWHPath,WattsHTotal);
    Watt=0;
  }
  
//Here cursor is placed on first position (col: 0) of the second line (row: 1) 
}
 
// ***** function calls ******
void getVPP(int a,int b)
{
  float result;
  int readValue,readValue2;                // value read from the sensor
  int maxValue = 0;             // store max value here
  int minValue = 4095;          // store min value here ESP32 ADC resolution
  int maxValue2 = 0;             // store max value here
  int minValue2 = 4095;          // store min value here ESP32 ADC resolution

   uint32_t start_time = millis();
   while((millis()-start_time) < 1000) //sample for 1 Sec
   {
       readValue = analogRead(a);
       readValue2 = analogRead(b);
       delay(17);
       // see if you have a new maxValue
       if (readValue > maxValue) 
       {
           maxValue = readValue;
       }
       if (readValue < minValue) 
       {
           minValue = readValue;
       }
        //
       if (readValue2 > maxValue2) 
       {
           maxValue2 = readValue2;
       }
       if (readValue2 < minValue2) 
       {
           minValue2 = readValue2;
       }
   }

   
   // Subtract min from max
   result = ((maxValue - minValue) * 3.3)/4095.0; //ESP32 ADC resolution 4096
   Voltage =result;
   result = ((maxValue2 - minValue2) * 3.3)/4095.0;
   Voltage2 = (result/2-0.02);    
 }


 
void mandardar(){
  Serial.print("/");
  Serial.print(VoltsRMS);
  Serial.print("/");
  Serial.print(",");
  Serial.print(AmpsRMS);
  Serial.print(",");
  Serial.print("-");
  Serial.print(Fp);
  Serial.println("-");
  
}
void  timerisestart(){
  timesr=micros();
}
void  timerisestop(){
  timest=timesr-micros();
  if(cos(timest*3.14159/16666.667)<=0){
  Fp=1+cos(timest*3.14159/16666.667);
  
  }
  else{
     Fp=cos(timest*3.14159/16666.667);
    }
  
  
}
