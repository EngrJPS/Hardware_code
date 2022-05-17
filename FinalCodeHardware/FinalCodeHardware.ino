/////////////////////////////////////////////////////////////////////// THIS IS THE LIBRARY FOR FIREBASE ESP32 /////////////////////////////////////////////////////////////////////////////////////////////
#if defined(ESP32)
#include <WiFi.h>
#include <FirebaseESP32.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#endif

//Provide the token generation process info.
#include <addons/TokenHelper.h>

//Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>

/* 1. Define the WiFi credentials */
#define WIFI_SSID "dlink-B8EB"
#define WIFI_PASSWORD "rvcuf68934"
/*
 *Mobile SSID: Redmi 9
 *Mobile Pass: 8tq9syrtgmtpbhr
*/
//For the following credentials, see examples/Authentications/SignInAsUser/EmailPassword/EmailPassword.ino

/* 2. Define the API Key */
#define API_KEY "AIzaSyDicz8IHyIAu4xhmjzc-f3GNIfjnx4NY44"

/* 3. Define the RTDB URL */
#define DATABASE_URL "https://agribuhaydatabase-default-rtdb.asia-southeast1.firebasedatabase.app/" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app

/* 4. Define the user Email and password that alreadey registerd or added in your project */
#define USER_EMAIL "is.excalibur73@gmail.com"
#define USER_PASSWORD "123123"

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;

unsigned long count = 0;

/////////////////////////////////////////////////////////////////////// THIS IS THE LIBRARY FOR LCD /////////////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
//Set GPIO 21 for SDA and GPIO 22 for SCL
LiquidCrystal_I2C lcd(0x27, 16, 2);

/////////////////////////////////////////////////////////////////////// THIS IS THE LIBRARY FOR DHT11/////////////////////////////////////////////////////////////////////////////////////////////
#include "DHT.h"
#define DHTPIN 15     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
//DHT initialization
DHT dht(DHTPIN, DHTTYPE);

/////////////////////////////////////////////////////////////////////// THIS IS THE LIBRARY FOR CO2 SENSOR MQ135 /////////////////////////////////////////////////////////////////////////////////////////////
//Library for MQ135 sensor
#include <MQUnifiedsensor.h>

//Definitions
#define placa "ESP32"
#define Voltage_Resolution 5
#define pin 32 //Analog GPIO 32 of ESP32
#define type "MQ-135" //MQ135
#define ADC_Bit_Resolution 12 // For ESP32
//Declare Sensor
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  
//#define calibration_button 13 //Pin to calibrate your sensor

MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

/////////////////////////////////////////////////////////////////////// THIS IS THE LIBRARY FOR WEIGHT SENSOR HX711 /////////////////////////////////////////////////////////////////////////////////////////////
/*This is the library for the HX711*/
#include <HX711_ADC.h>

#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

//pins:
const int HX711_dout = 25; //mcu > HX711 dout pin
const int HX711_sck = 26; //mcu > HX711 sck pin

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
const int tareOffsetVal_eepromAdress = 4;
unsigned long t = 0;

//This is the pins for the button
const int buttonPin = 13; //put the button in the pin 13

const int outputPin = 12; //Uncomment this to check if the button isn't working

int buttonState = 0; //Output of the button

unsigned long previousTime = 0;

unsigned long previousTime1 = 0;

unsigned long previousTime2 = 0;

const long eventTime1 = 500;

const long eventTime2 = 5000;

double co2;

float i;

unsigned long timerWifi = 20000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //LCD initializations
  lcd.begin();
  lcd.backlight();

  //DHT 11 initialization
  dht.begin();

  //WiFi Initializations
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
    //If WiFi is not connected within 20sec, ESP32 restarts
    if(millis() - previousTime >= timerWifi){
      Serial.print("ESP restarting in: ");
      for(int x = 10; x < 1; x--){
        Serial.println(x);
      }
        ESP.restart();
        previousTime = millis();
    }
    
  }

  lcd.setCursor(0, 0);
  lcd.print("Connected with IP: ");
  lcd.setCursor(0, 1);
  lcd.println(WiFi.localIP());
  delay(1000);

  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the user sign in credentials */
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  //Or use legacy authenticate method
  //config.database_url = DATABASE_URL;
  //config.signer.tokens.legacy_token = "<database secret>";

  //To connect without auth in Test Mode, see Authentications/TestMode/TestMode.ino


  //////////////////////////////////////////////////////////////////////////////////////////////
  //Please make sure the device free Heap is not lower than 80 k for ESP32 and 10 k for ESP8266,
  //otherwise the SSL connection will fail.
  //////////////////////////////////////////////////////////////////////////////////////////////

  Firebase.begin(&config, &auth);

  //Comment or pass false value when WiFi reconnection will control by your code or third party library
  Firebase.reconnectWiFi(true);

  Firebase.setDoubleDigits(5);

  lcd.print("INITIALIZING....");

  Serial.println();
  Serial.println("Starting...");


  //Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to to calculate CO2 concentration

  /*
    Exponential regression:
    GAS      | a      | b
    CO       | 605.18 | -3.937
    Alcohol  | 77.255 | -3.18
    CO2      | 110.47 | -2.862
    Toluen  | 44.947 | -3.445
    NH4      | 102.2  | -2.473
    Aceton  | 34.668 | -3.369
  */

  /*****************************  MQ Init ********************************************/
  //Remarks: Configure the pin of arduino as input.
  /************************************************************************************/
  MQ135.init();
  /*
    //If the RL value is different from 10K please assign your RL value with the following method:
    MQ135.setRL(10);
  */
  /*****************************  MQ CAlibration ********************************************/
  // Explanation:
  // In this routine the sensor will measure the resistance of the sensor supposedly before being pre-heated
  // and on clean air (Calibration conditions), setting up R0 value.
  // We recomend executing this routine only on setup in laboratory conditions.
  // This routine does not need to be executed on each restart, you can load your R0 value from eeprom.
  // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(43.898);
  Serial.println("  done!.");

  if (isinf(calcR0)) {
    Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while (1);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while (1);
  }
  /*****************************  MQ CAlibration ********************************************/
  MQ135.serialDebug(true);

  //  Loadcell initialization
  LoadCell.begin();
  //LoadCell.setReverseOutput();
  float calibrationValue; // calibration value (see example file "Calibration.ino")
  calibrationValue = 696.0; // uncomment this if you want to set the calibration value in the sketch

#if defined(ESP8266)|| defined(ESP32)
  EEPROM.begin(512);
#endif

  EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom

  //restore the zero offset value from eeprom:
  long tare_offset = 0;
  EEPROM.get(tareOffsetVal_eepromAdress, tare_offset);
  LoadCell.setTareOffset(tare_offset);
  boolean _tare = false; //set this to false as the value has been resored from eeprom

  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }

  lcd.clear();
  delay(2000);
  
  lcd.print("----------------");
  lcd.setCursor(2, 1);
  lcd.print("Connected...");

  delay(1000);

  lcd.clear();

  pinMode(outputPin, OUTPUT);
  pinMode(buttonPin, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentTime = millis();

  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    if (currentTime - previousTime1 >= eventTime1) {
      getWeight();
      previousTime1 = currentTime;
    }
  } else {
    if (currentTime - previousTime2 >= eventTime2) {
      getCO2Sensor();
      getTempHumid();
      lcd.clear();
      previousTime2 = currentTime;
    }
  }
}

void getCO2Sensor() {
  const int numReadings = 10;

  int readings[numReadings];
  int readIndex = 0;
  int total = 0;
  int avg = 0;

  MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
  co2 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  
  Serial.print(co2);
  Serial.println(" CO2 PPM");

  lcd.setCursor(0, 0);
  lcd.print("CO2 PPM value: ");
  lcd.setCursor(0, 1);
  lcd.print(co2);
  delay(3000);

  Firebase.setFloat(fbdo, F("/Sensors/CO2_PPM"), co2);
}

void getWeight() {

  static boolean newDataReady = 0;
  const int serialPrintInterval = 500; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      i = LoadCell.getData();
      Serial.print("Load_cell output val: ");
      Serial.println(i);
      lcd.setCursor(0,0);
      lcd.print("Net weight value: ");
      lcd.setCursor(0, 1);
      lcd.print(abs(i / 1000));
      lcd.setCursor(4, 1);
      lcd.print(" kg");
      Firebase.setDouble(fbdo, F("/Sensors/Weight_Value"), abs(i / 1000));
      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') refreshOffsetValueAndSaveToEEprom();
  }
}

void getTempHumid() {
  //  delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  Serial.print(F("Humidity: "));
  Serial.println(h);
  Serial.print(F("%  Temperature: "));
  Serial.println(t);

  lcd.setCursor(0, 0);
  lcd.print("Temp value: ");
  lcd.setCursor(0, 1);
  lcd.print(t);
  Firebase.setFloat(fbdo, F("/Sensors/Temp_Value"), t);
  delay(3000);

  lcd.setCursor(0, 0);
  lcd.print("Humidity value: ");
  lcd.setCursor(0, 1);
  lcd.print(h);
  Firebase.setFloat(fbdo, F("/Sensors/Humid_Value"), h);
  delay(3000);
}

void refreshOffsetValueAndSaveToEEprom() {
  long _offset = 0;
  Serial.println("Calculating tare offset value...");
  LoadCell.tare(); // calculate the new tare / zero offset value (blocking)
  _offset = LoadCell.getTareOffset(); // get the new tare / zero offset value
  EEPROM.put(tareOffsetVal_eepromAdress, _offset); // save the new tare / zero offset value to EEprom
#if defined(ESP8266) || defined(ESP32)
  EEPROM.commit();
#endif
  LoadCell.setTareOffset(_offset); // set value as library parameter (next restart it will be read from EEprom)
  Serial.print("New tare offset value:");
  Serial.print(_offset);
  Serial.print(", saved to EEprom adr:");
  Serial.println(tareOffsetVal_eepromAdress);
}
