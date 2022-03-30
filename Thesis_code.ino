/*
 * This is the code for our hardware
*/

//This is the library for the hx711 and loadcell
#include<HX711_ADC.h> 
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

//pins:
const int HX711_dout = 25; //mcu > HX711 dout GPIO 25
const int HX711_sck = 26; //mcu > HX711 sck GPIO 26

HX711_ADC LoadCell(HX711_dout, HX711_sck); //HX711 constructor

const int calVal_eepromAdress = 0; //This variable is for calibration
unsigned long t = 0;
float i; //This variable is for the output of load cell

//The library for FIREBASEESP32
#include<WiFi.h>
#include<FirebaseESP32.h>

//The library for DHT11
#include "DHT.h"

#define DHTTYPE DHT11   // DHT 11

#define DHTPIN 15     // GPIO 15 connected to the DHT sensor 

DHT dht(DHTPIN, DHTTYPE);

//This library is for LCD
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
//Set GPIO 21 for SDA and GPIO 22 for SCL
LiquidCrystal_I2C lcd(0x27, 16, 2);


#define FIREBASE_HOST "https://agribuhaydatabase-default-rtdb.asia-southeast1.firebasedatabase.app/" //RTDB of the project
#define FIREBASE_AUTH "8aNXNPagtqzIRlEVN9r2V6pilHWcvf9LPWMQKjgr" //Server key of the cloud server
#define WIFI_SSID "dlink-B8EB" //Wifi-name
#define WIFI_PASSWORD "rvcuf68934" //Wifi password

FirebaseData firebaseData;
FirebaseJson json;

uint32_t delayMS;

//analog interface of the MHZ-14A
const int analogPin = 4;

//PWM interface of the MHZ-14A
const int pwmPin = 0;

void setup(){

  //Initializes the lcd with begin function
  lcd.begin();
  
  //Initializes the lcd backlight
  lcd.backlight();

  lcd.print("Starting...");

  delay(2000);

  lcd.clear();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lcd.print("Connecting to Wi-Fi");
  while(WiFi.status() != WL_CONNECTED){
    lcd.setCursor(0,1);
    lcd.print(".");
    delay(300);
  }

  delay(2000);
  lcd.clear();

  lcd.print("Connected with IP: ");
  lcd.setCursor(0,1);
  lcd.print(WiFi.localIP());
  Serial.println();

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  Firebase.setReadTimeout(firebaseData, 1000*60);

  Firebase.setwriteSizeLimit(firebaseData, "tiny");

  lcd.clear();
  delay(2000);

  lcd.print("----------------");
  lcd.setCursor(2,1);
  lcd.print("Connected...");

  lcd.clear();
  delay(1000);

  lcd.setCursor(2,0);
  lcd.print("Initializing");
  lcd.setCursor(4,1);
  lcd.print("load cell");

  Serial.begin(57600); 
  delay(10);
  Serial.println();
  Serial.println("Starting...");

  //Initializes the load cell using the begin function from the library
  LoadCell.begin();
  
  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(1.0); // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  calibrate(); //start calibration procedure

  //MHZ-14A INTIATION
  pinMode(pwmPin, INPUT_PULLUP);
  
  lcd.print("Preheating CO2")

  delay(180000); // preheat the CO2 sensor for 3 minutes

}


void loop(){

  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      i = LoadCell.getData();//Value of the load cell
      Serial.print("Load_cell output val: ");
      Serial.println(i);
      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay(); //tare
    else if (inByte == 'r') calibrate(); //calibrate
    else if (inByte == 'c') changeSavedCalFactor(); //edit calibration value manually
  }

  // check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
  
  delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  int h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);
  // Read CO2 Analog
  int ppm_analog = get_analog();
  // Read CO2 Concetration
  int ppm_PWM = gas_concentration_PWM();

  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  if(isnan(ppm_analog) || isnan(ppm_PWM)){
    Serial.println(F("Failed to read from the MHZ-14A sensor!"));
    return;
  }
  
  //Saving the weight value of i to the weight RTDB
  json.set("/weight",i);
  //Saving the temperature value of t to the temperature RTDB
  json.set("/temperature", t);
  //Saving the humidity value of h to the humidity RTDB
  json.set("/humidity", h);
  //Saving the CO2 analog value of ppm_analog to the CO2_analog RTDB
  json.set("/CO2_analog", ppm_analog);
  //Saving the CO2 PWM value of ppm_analog to the CO2_PWM RTDB
  json.set("/CO2_PWM", ppm_PWM);
  //Updating the node with the name Sensors to the RTDB
  Firebase.updateNode(firebaseData, "/Sensors", json);

  //Getting the value of temperature from the Firebase RTDB
  if(Firebase.getFloat(firebaseData, "/Sensors/temperature")){
    if(firebaseData.dataType() == "float"){
      lcd.print("Temperature value: ");
      lcd.setCursor(0,1);
      lcd.print(firebaseData.floatData());
      delay(10000);
      lcd.clear();
    }
  }
  
  //Getting the value humidity from the Firebase RTDB
  if(Firebase.getInt(firebaseData, "/Sensors/humidity")){
    if(firebaseData.dataType() == "int"){
      lcd.print("Humidity value: ");
      lcd.setCursor(0,1);
      lcd.print(firebaseData.intData());
      delay(10000);
      lcd.clear();
    }
  }
  
  //Getting the value of weight from the Firebase RTDB
  if(Firebase.getFloat(firebaseData, "/Sensors/weight")){
    if(firebaseData.dataType() == "float"){
      lcd.print("Load cell val: ");
      lcd.setCursor(0,1);
      lcd.print(firebaseData.floatData());
      delay(10000);
      lcd.clear();
    }
  }

  //Getting the value humidity from the Firebase RTDB
  if(Firebase.getInt(firebaseData, "/Sensors/CO2_analog")){
    if(firebaseData.dataType() == "int"){
      lcd.print("CO2_analog value: ");
      lcd.setCursor(0,1);
      lcd.print(firebaseData.intData());
      delay(10000);
      lcd.clear();
    }
  }

  //Getting the value humidity from the Firebase RTDB
  if(Firebase.getInt(firebaseData, "/Sensors/CO2_PWM")){
    if(firebaseData.dataType() == "int"){
      lcd.print("CO2_PWM value: ");
      lcd.setCursor(0,1);
      lcd.print(firebaseData.intData());
      delay(10000);
      lcd.clear();
    }
  }
}

void calibrate() {
  Serial.println("***");
  Serial.println("Start calibration:");
  Serial.println("Place the load cell an a level stable surface.");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("Send 't' from serial monitor to set the tare offset.");

  boolean _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      if (Serial.available() > 0) {
        char inByte = Serial.read();
        if (inByte == 't') LoadCell.tareNoDelay();
      }
    }
    if (LoadCell.getTareStatus() == true) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

  Serial.println("Now, place your known mass on the loadcell.");
  Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass != 0) {
        Serial.print("Known mass is: ");
        Serial.println(known_mass);
        _resume = true;
      }
    }
  }

  LoadCell.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass); //get the new calibration value

  Serial.print("New calibration value has been set to: ");
  Serial.print(newCalibrationValue);
  Serial.println(", use this as calibration value (calFactor) in your project sketch.");
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? y/n");

  _resume = false;
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        _resume = true;

      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }

  Serial.println("End calibration");
  Serial.println("***");
  Serial.println("To re-calibrate, send 'r' from serial monitor.");
  Serial.println("For manual edit of the calibration value, send 'c' from serial monitor.");
  Serial.println("***");
}

void changeSavedCalFactor() {
  float oldCalibrationValue = LoadCell.getCalFactor();
  boolean _resume = false;
  Serial.println("***");
  Serial.print("Current value is: ");
  Serial.println(oldCalibrationValue);
  Serial.println("Now, send the new value from serial monitor, i.e. 696.0");
  float newCalibrationValue;
  while (_resume == false) {
    if (Serial.available() > 0) {
      newCalibrationValue = Serial.parseFloat();
      if (newCalibrationValue != 0) {
        Serial.print("New calibration value is: ");
        Serial.println(newCalibrationValue);
        LoadCell.setCalFactor(newCalibrationValue);
        _resume = true;
      }
    }
  }
  _resume = false;
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? y/n");
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        _resume = true;
      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }
  Serial.println("End change calibration value");
  Serial.println("***");
}

int get_analog(){
  float v = analogRead(analogPin) * 5 / 4095.0;
  int gas_concentration = int((v) * (5000/2));

  return gas_concentration;
}

int gas_concentration_PWM(){
  
  while (digitalRead(pwmPin) == LOW) {};
  long t0 = millis();
  
  while (digitalRead(pwmPin) == HIGH) {};
  long t1 = millis();
  
  while (digitalRead(pwmPin) == LOW) {};
  long t2 = millis();
  
  long th = t1-t0;
  long tl = t2-t1;
  long ppm = 5000L * (th - 2) / (th + tl - 4);
  
  while (digitalRead(pwmPin) == HIGH) {};
  delay(10);
  
  return int(ppm);
}
