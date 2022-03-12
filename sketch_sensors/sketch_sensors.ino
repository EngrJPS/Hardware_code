esp32_hx711
  // put your setup code here, to run once:
<span style="font-size: 14pt; font-family: 'times new roman', times, serif;">#include "HX711.h"
 sensors #include "soc/rtc.h" 
#include <stdlib.h>
#include <Wire.h>
sensors#include <Adafruit_GFX.h>
 #include <Adafruit_SSD1306.h>
#include <SimpleTimer.h>
 
#define DOUT  23
#define CLK  19
HX711 scale(DOUT, CLK);
}
int rbutton = 18; // this button will be used to reset the scale to 0. 
String myString; 
String cmessage; // complete message
char buff[10];
float weight; 
float calibration_factor = 206140; 
void loop() {
  SimpleTimer Timer;
  // // for the OLED display
 
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
 
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
 
 
// You should get Auth Token in the Blynk App.
char auth[] = "SQPCAqTecEFGUs5UO7_wnDLQPrtCqx-h";
 
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "fawadkhan";
char pass[] = "computer12ec";
 
void setup() {
  
  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass);
  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
  pinMode(rbutton, INPUT_PULLUP); 
  scale.set_scale();
  scale.tare(); //Reset the scale to 0
  long zero_factor = scale.read_average(); //Get a baseline reading
 
 
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  timer.setInterval(1500L, getSendData);
  display.clearDisplay();
  display.setTextColor(WHITE);
 
}
 
 
void loop() {
 
Blynk.run(); 
timer.run(); // Initiates SimpleTimer
 
}
 
 
void getSendData()
{
 
  scale.set_scale(calibration_factor); //Adjust to this calibration factor
 
weight = scale.get_units(5); //5
myString = dtostrf(weight, 3, 3, buff);
cmessage = cmessage + "Weight" + ":" + myString + "Kg"+","; 
 
 
  if ( digitalRead(rbutton) == LOW)
  {
     scale.set_scale();
  scale.tare(); //Reset the scale to 0
  }
 
  
Blynk.virtualWrite(V2,"Weight:");
Blynk.virtualWrite(V3,myString);
        
// Oled display
  display.clearDisplay();
  display.setTextSize(3);
  display.setCursor(0,0); // column row
  display.print("Weight:");
 
  display.setTextSize(4);
  display.setCursor(0,30);
  display.print(myString);
 
 display.display(); 
}
</span>

}