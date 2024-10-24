#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <avr/io.h>
#include <avr/interrupt.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows

// Pins definitions
#define TEMP_SENSOR_PIN A0  // "Analog" pins
#define CO2_ANALOG_VALUE_PIN A1
#define SOIL_ANALOG_VALUE_SENSOR_PIN A2
#define BUZZER_PIN 5  // PWM
#define WATER_PUMP_PIN 6
#define SOIL_SENSOR_PIN 8
#define CO2_SENSOR_PIN 2                 //INT0
#define BUTTON_PIN 3                    //INT1

//DEBOUNCER
volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Global vaiables
volatile bool buttonState = HIGH;
volatile bool co2Alarm = HIGH;
volatile bool waterState;// = HIGH; // HIGH = plant has enough water

bool prevButtonState;
int pevTempVal = 0;
int prevCo2Val = 0;

DHT dht11(TEMP_SENSOR_PIN, DHT11);

void setup()
{
  // Pins setup
  pinMode(CO2_ANALOG_VALUE_PIN, INPUT); // Inputs
  pinMode(SOIL_ANALOG_VALUE_SENSOR_PIN, INPUT);
  pinMode(CO2_SENSOR_PIN, INPUT_PULLUP); // INT0
  pinMode(SOIL_SENSOR_PIN, INPUT_PULLUP);  // INT1
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(WATER_PUMP_PIN, OUTPUT);  // Outputs
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initialization
  lcd.init(); // initialize the lcd
  lcd.backlight();
  dht11.begin(); // initialize the sensor
  Serial.begin(9600); // initialize the serial communication

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, RISING);
  attachInterrupt(digitalPinToInterrupt(CO2_SENSOR_PIN), co2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CO2_SENSOR_PIN), co2ISR, CHANGE);
}
  
void loop(){
  if(co2Alarm){ // Check if buzzer ande red diode should be turned on
      analogWrite(BUZZER_PIN, 0);
  } else {
      analogWrite(BUZZER_PIN, 191); // After interrupt turning on buzzer and red diode
  }

float temp = dht11.readTemperature();
float hum = dht11.readHumidity();
float co2Read = analogRead(CO2_ANALOG_VALUE_PIN);

  if(buttonState){  // Check what should be displayed on LCD
    tempLCD(temp, hum);
  } else{
    co2LCD(co2Read);
  }
  prevButtonState = buttonState;

  waterState = digitalRead(SOIL_SENSOR_PIN);
  Serial.print(waterState);
  watering(waterState);
}

// Functions
float co2PpmConversion(float co2Value){
  //???
}

void tempLCD(float temperature, float humidity){

  if (temperature != pevTempVal) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: " + String(temperature));
    lcd.setCursor(0, 1);
    lcd.print("Humidity: " + String(humidity) + "%  ");
    pevTempVal = temperature;
  }
}

void co2LCD(float co2Concentration){
  if (co2Concentration != prevCo2Val){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CO2: " + String(co2Concentration) + "ppm");

  if(co2Concentration > 1000){
    lcd.setCursor(0, 1);
    lcd.print("Extremly High!  ");
  } else if (co2Concentration > 500){
      lcd.setCursor(0, 1);          // move cursor to   (0, 1)
      lcd.print("High CO2        "); // print message at (0, 1)
  } else {
      lcd.setCursor(0, 1);          // move cursor to   (0, 1)
      lcd.print("Good CO2        "); // print message at (0, 1)
    }
    prevCo2Val = co2Concentration;
  }
}

void watering(bool waterState){
  if(waterState){
    analogWrite(WATER_PUMP_PIN, 0); // Giving 5V on base of pnp transistor, transistor closed
  } else {
    analogWrite(WATER_PUMP_PIN, 255); // 0V on base, transistor conducts
  }
}

// ISR functions
void co2ISR(){
  co2Alarm = !co2Alarm;
}

void buttonISR(){
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > debounceDelay) {
    buttonState = !buttonState;
    lastDebounceTime = currentTime;
    pevTempVal = 0;
    prevCo2Val = 0;
  }
}