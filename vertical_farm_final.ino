#include <DFRobot_PH.h>
#include "DFRobot_PH.h"
#include <EEPROM.h>
#include <DateTime.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>

#define DHT_PIN A0
#define GAS_PIN A1
#define PH_PIN A2
#define HEAT_PIN 7
#define LIGHT_PIN 8
#define FAN_PIN 9
#define HUMID_PIN 10
#define PUMP_PIN 11

#define TIME_MSG_LEN  11
#define TIME_HEADER  255 

float PH_voltage = 20;
float PH_value = 20;

float temperature, humidity;
int gas_value;

float read_values();

DFRobot_PH ph;
DHT dht = DHT(DHT_PIN, DHT22);

LiquidCrystal_I2C lcd(0x27, A5, 2);

void setup() {
  Serial.begin(115200);

  pinMode(DHT_PIN, INPUT);
  pinMode(GAS_PIN, INPUT);
  pinMode(PH_PIN, INPUT);
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(HUMID_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  
  dht.begin();
  ph.begin();

  lcd.init();
  lcd.backlight();
}

void loop() {
  getPCtime(); //Update time
  
  float temperature = dht.readTemperature(); //Read temperature and humidity values
  float humidity = dht.readHumidity();

  PH_voltage = analogRead(PH_PIN)/1024.0*5000; //Read PH value and calibrate
  PH_value = ph.readPH(PH_voltage, 20);
  ph.calibration(PH_voltage, 20);

  int gas_value = analogRead(GAS_PIN); //Read gas sensor value

  char str1[30];
  char str2[30];
  char t[15];
  char h[15];
  char p[15];
  
  dtostrf(temperature,4,1,t); //Convert float values to string
  dtostrf(humidity,2,0,h);
  dtostrf(PH_value,2,1,p);
  
  sprintf(str1, "T:%s C, H:%s%%", t, h); //Stitch strings for display
  sprintf(str2, "G:%d, PH:%s",gas_value, p);
  
  lcd.setCursor(0,0); //Update LCD dispay
  lcd.print(ok);
  lcd.setCursor(0,1);
  lcd.print(oke);

  //Temperature control
  if (temperature < 19) {
    digitalWrite(HEAT_PIN, HIGH);
  }
  else {
    digitalWrite(HEAT_PIN, LOW);
  }
  
  if (temperature > 21) {
    digitalWrite(FAN_PIN, HIGH);
  }
  else {
    digitalWrite(FAN_PIN, LOW);
  }

  //Humidity control
  if (humidity < 50) {
    digitalWrite(HUMID_PIN, HIGH);
  }
  else {
    digitalWrite(HUMID_PIN, LOW);
  }
  if (humidity > 70) {
    digitalWrite(FAN_PIN, HIGH);
  }
  else {
    digitalWrite(FAN_PIN, LOW);
  }

  //Print warnings to serial when applicable
  if (gas_value > 450) {
    Serial.println("WARNING: CO2 LEVELS HIGH");
  }
  if (PH_value < 5) {
    Serial.println("WARNING: PH TOO LOW");
  }
  if (PH_value > 8) {
    Serial.println("WARNING: PH TOO HIGH");
  }

  //Light cycle
  if (23 > DateTime.Hour > 5) {
    digitalWrite(LIGHT_PIN, HIGH);
  }
  else{
    digitalWrite(LIGHT_PIN, LOW);
  }

  //Pump cycle
  if (DateTime.Hour == 8 & DateTime.Minute < 11) {
    digitalWrite(PUMP_PIN, HIGH);
  }
  else {
    digitalWrite(PUMP_PIN, LOW);
  }
  
  delay(1000);
}

void getPCtime() {
  while(Serial.available() >=  TIME_MSG_LEN ){
    if( Serial.read() == TIME_HEADER ) {        
      time_t pctime = 0;
      for(int i=0; i < TIME_MSG_LEN -1; i++){  
        char c= Serial.read();          
        if( c >= '0' && c <= '9')  
          pctime = (10 * pctime) + (c - '0') ;           
      }  
      DateTime.sync(pctime);
    }  
  }
}
