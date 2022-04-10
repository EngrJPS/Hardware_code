//The library for FIREBASE ESP32
#include<WiFi.h>
#include<FirebaseESP32.h>


FirebaseData firebaseData;
FirebaseJson json;

//This library is for LCD
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
//Set GPIO 21 for SDA and GPIO 22 for SCL
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define FIREBASE_HOST "https://agribuhaydatabase-default-rtdb.asia-southeast1.firebasedatabase.app/" //RTDB of the project
#define FIREBASE_AUTH "8aNXNPagtqzIRlEVN9r2V6pilHWcvf9LPWMQKjgr" //Server key of the cloud server

#define WIFI_SSID "dlink-guest" //Wifi-name
#define WIFI_PASSWORD "rvcuf68934" //Wifi password

/*This is the library for the DHT 11 sensor
 */
#include "DHT.h"
#define DHTPIN 15     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
//DHT initialization
DHT dht(DHTPIN, DHTTYPE);

//Library for MQ135 sensor
#include <MQUnifiedsensor.h>

//Definitions
#define placa "ESP32"
#define Voltage_Resolution 5
#define pin 32 //Analog input 0 of your arduino
#define type "MQ-135" //MQ135
#define ADC_Bit_Resolution 12 // For arduino UNO/MEGA/NANO
//Declare Sensor
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  
//#define calibration_button 13 //Pin to calibrate your sensor

MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

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

unsigned long previousTime1 = 0;

unsigned long previousTime2 = 0;

const long eventTime1= 500;

const long eventTime2 = 5000;

double co2;

float i;

void setup() {
  //LCD initializations
  lcd.begin();
  lcd.backlight();

  lcd.print("INITIALIZING....");

  Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Starting...");

  //WiFi Initialization
//  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
//  Serial.print("Connecting to Wi-Fi");
//  while(WiFi.status() != WL_CONNECTED){
//    Serial.print(".");
//    delay(300);
//  }
  
  // put your setup code here, to run once:
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
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 
  MQ135.serialDebug(true);

//
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

  //DHT 11 initialization
  dht.begin();
  
  delay(2000);

//  Serial.println();
//  Serial.print("Connected with IP: ");
//  Serial.println(WiFi.localIP());
//  Serial.println();
//  
//  lcd.clear();
//  lcd.print("Connected with IP: ");
//  lcd.setCursor(0,1);
//  lcd.print(WiFi.localIP());
//  lcd.println();
  
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  Firebase.setReadTimeout(firebaseData, 1000*60);

  Firebase.setwriteSizeLimit(firebaseData, "tiny");

  lcd.clear();
  delay(2000);

//  Serial.print("------------------------------------------------");
//  Serial.println("Connected...");
  
  lcd.print("----------------");
  lcd.setCursor(2,1);
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
  if(buttonState == HIGH){
     if(currentTime - previousTime1 >= eventTime1){
      getWeight();
//      Serial.println(buttonState);
      previousTime1 = currentTime;
      }
  }else{
    if(currentTime - previousTime2 >= eventTime2){
      getCO2Sensor();
      getTempHumid();
      previousTime2 = currentTime;
    }
  }
}

void getCO2Sensor(){
  MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
  co2 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
//  MQ135.serialDebug(); // Will print the table on the serial port
//  delay(500); //Sampling frequency
  Serial.print(co2);
  Serial.println(" CO2 PPM");
//  lcd.cursor(0,0);
// lcd.print(co2);
//  json.set("/CO2_PPM",co2);

  lcd.print("CO2 PPM value: ");
  lcd.setCursor(0,1);
  lcd.print(co2);
  delay(3000);
  lcd.clear();

//  Firebase.updateNode(firebaseData, "/Sensors", json);
//
//  if(Firebase.getDouble(firebaseData, "/Sensors/CO2_PPM")){
//    if(firebaseData.dataType() == "double"){
//      lcd.print("CO2 PPM value: ");
//      lcd.setCursor(0,1);
//      lcd.print(firebaseData.doubleData());
//      delay(3000);
//      lcd.clear();
//    }
//  }
}

void getWeight(){

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
      lcd.print("Net weight value: ");
      lcd.setCursor(0,1);
      lcd.print(i/1000);
      lcd.setCursor(4,1);
      lcd.print(" kg");
      delay(1000);
      lcd.clear();
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

void getTempHumid(){
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

//  json.set("/humidity",h);
  
//  json.set("/temperature", t);

//  Firebase.updateNode(firebaseData, "/Sensors", json);

  lcd.print("Temp value: ");
  lcd.setCursor(0,1);
  lcd.print(t);
  delay(3000);
  lcd.clear();

  
  if(Firebase.getFloat(firebaseData, "/Sensors/temperature")){
    if(firebaseData.dataType() == "float"){
      lcd.print("Temp value: ");
      lcd.setCursor(0,1);
      lcd.print(firebaseData.floatData());
      delay(3000);
      lcd.clear();
    }
  }

  lcd.print("Humidity value: ");
  lcd.setCursor(0,1);
  lcd.print(h);
  delay(3000);
  lcd.clear();

//  if(Firebase.getInt(firebaseData, "/Sensors/humidity")){
//      if(firebaseData.dataType() == "int"){
//        lcd.print("Humidity value: ");
//        lcd.setCursor(0,1);
//        lcd.print(firebaseData.floatData());
//        delay(3000);
//        lcd.clear();
//    }
//  }
  
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
