#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <avr/io.h>
#include <avr/interrupt.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows
#define DHT11_PIN A2
// #define MQ_9
// #define OKY
#define DEBOUNCER 10

DHT dht11(DHT11_PIN, DHT11);
const int INTERRUPT_BUTTON_PIN = 2; // 2&3 pins can be interrupt pins for uno&nano
const int INTERRUPT_WATER_SENSOR_PIN = 3;

const int WATER_PUMP_PIN = 8;

volatile bool buttonState = LOW;
//volatile bool previousButtonState = LOW;
volatile bool refresh = HIGH;
bool refreshed = HIGH;
unsigned int reload = 0xF424;

ISR(TIMER1_COMPA_vect){
  refresh = !refresh;
}

void setup()
{
  lcd.init(); // initialize the lcd
  lcd.backlight();
  Serial.begin(9600);
  dht11.begin(); // initialize the sensor

  pinMode(INTERRUPT_BUTTON_PIN, INPUT); // might be input_pullup cos atmega has built-in pull-up ressistor but my design is with pull-down resistor so i don't want to change it cos of lazyness
  pinMode(INTERRUPT_WATER_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_BUTTON_PIN), myISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_WATER_SENSOR_PIN), waterISR, CHANGE);

  pinMode(WATER_PUMP_PIN, OUTPUT);

//Timer Interrupt
cli();
TCCR1A = 0;
TCCR1B = 0; 
OCR1A = reload;
TCCR1B = (1<<WGM12) | (1<<CS12) | (1 << CS10); 
TIMSK1 = (1<<OCIE1A); 
sei(); 
////////////////////
}

void loop()
{

  if(buttonState == LOW){
  int temp = dht11.readTemperature();
  int hum = dht11.readHumidity();
  if(refresh != refreshed){
    lcd.clear();
    refreshed = refresh;
  }
  lcd.setCursor(0, 0);
  lcd.print("Temp: " + String(temp));
  lcd.setCursor(0, 1);
  lcd.print("Humidity: " + String(hum) + "%  ");
  }else {
    float CO2 = analogRead(A1);
  if(refresh != refreshed){
    lcd.clear();
    refreshed = refresh;
  }
    lcd.setCursor(0, 0);
    lcd.print("CO2: " + String(CO2));
  if(CO2 > 1000){
    lcd.setCursor(0, 1);
    lcd.print("Extremly High!  ");
  } else if (CO2 > 500){
      lcd.setCursor(0, 1);          // move cursor to   (0, 1)
      lcd.print("High CO2        "); // print message at (0, 1)
  } else {
      lcd.setCursor(0, 1);          // move cursor to   (0, 1)
      lcd.print("Good CO2        "); // print message at (0, 1)
  }
  //delay(5000);
}

}

void myISR() {
  static bool switching_pending = false;
  static long int elapse_timer;
  int button_reading = digitalRead(INTERRUPT_BUTTON_PIN);

  if(button_reading == HIGH){
    switching_pending = true;
    elapse_timer  = millis();
  }
  if(switching_pending && button_reading == LOW){
    if(millis() - elapse_timer > DEBOUNCER){
      switching_pending = false;
      buttonState = !buttonState;
      refreshed = !refresh;
    }
  }
}

void waterISR(){
  bool sensorReading = digitalRead(INTERRUPT_WATER_SENSOR_PIN);
  if(sensorReading == HIGH){
    digitalWrite(WATER_PUMP_PIN, HIGH);
    //pumpWater = false;
  } else digitalWrite(WATER_PUMP_PIN, LOW);
}

//TO DO
//1. Interrupt for soil sensor -> turnig on the water pump, probably with use of transistor -> to give Vcc from usb and not from uC pin
//2. conect buzzer to high co2
//3. 