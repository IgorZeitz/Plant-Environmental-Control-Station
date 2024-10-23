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
#define CO2_SENSOR_PIN 8  // Interrupts,  PCINT0
#define SOIL_SENSOR_PIN 3               //INT1
#define BUTTON_PIN 2                    //INT0
//!!!!!!!!!!!!!!!!!!! PWM - 3, 5, 6, 9, 10, 11 

//#define DEBOUNCER 10  ??????
volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
/*
  PB0-PB7 pins used for PCINT (PCINT0-PCINT7)
  it's better not to use D8 to D13 since those
  pins are configured for pin change interrupts
*/

// Global vaiables
volatile bool buttonState = HIGH;
volatile bool co2Alarm = HIGH;
volatile bool watering = HIGH;
bool prevButtonState;
bool waterState = 0;
bool co2State;
//volatile bool previousButtonState = LOW;
//volatile bool refresh = HIGH;
//bool refreshed = HIGH;
//unsigned int reload = 0xF424;

/*
ISR(TIMER1_COMPA_vect){
  refresh = !refresh;
}
*/

void setup()
{
  // Pins setup
  DHT dht11(TEMP_SENSOR_PIN, DHT11);  // Inputs
  pinMode(CO2_ANALOG_VALUE_PIN, INPUT);
  pinMode(SOIL_ANALOG_VALUE_SENSOR_PIN, INPUT);
  pinMode(CO2_SENSOR_PIN, INPUT_PULLUP); // INT0
  pinMode(SOIL_SENSOR_PIN, INPUT_PULLUP);  // INT1
  pinMode(BUTTON_PIN, INPUT_PULLUP); // PCINT0
  pinMode(WATER_PUMP_PIN, OUTPUT);  // Outputs
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initialization
  lcd.init(); // initialize the lcd
  lcd.backlight();
  dht11.begin(); // initialize the sensor
  Serial.begin(9600); // initialize the serial communication

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, RISING);
  attachInterrupt(digitalPinToInterrupt(SOIL_SENSOR_PIN), soilISR, CHANGE);

  PCICR |= (1<<PCIE0);  // Enabling PCINT0, change on any enabled PCINT7-0 pin will cause an interrupt
  PCMSK0 |= (1<<PCINT0); // register that controls which pins contribute to the pin change interrupts - only PCINT0 - D8. Enabling individual pin
  sei();
}
  
//Timer Interrupt
/*
cli();
TCCR1A = 0;
TCCR1B = 0; 
OCR1A = reload;
TCCR1B = (1<<WGM12) | (1<<CS12) | (1 << CS10); 
TIMSK1 = (1<<OCIE1A); 
sei(); 
*/
////////////////////
//bool alarm = 0;
void loop(){
  if(co2Alarm != waterState){
  if(co2Alarm){ // Check if buzzer ande red diode should be turned on
      analogWrite(BUZZER_PIN, 0);
  } else {
      analogWrite(BUZZER_PIN, 191); // After interrupt turning on buzzer and red diode
  }
  //waterState = co2Alarm;
  }

  if(watering){ // Check if plant should be watered
    analogWrite(WATER_PUMP_PIN, 255); // Giving 5V on base of pnp transistor
  } else {
    analogWrite(WATER_PUMP_PIN, 0); // 0V on base, transistor conducts
  }

  if(buttonState){  // Check what should be displayed on LCD
    tempLCD(10, 20);
  } else{
    co2LCD(222);
  }

  prevButtonState = buttonState;
  waterState = co2Alarm;
  //co2State = co2Alarm;


/*
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
    float CO2_read = analogRead(A1);
    
    alarm((bool)CO2_read);
  if(refresh != refreshed){
    lcd.clear();
    refreshed = refresh;
  }
    lcd.setCursor(0, 0);
    lcd.print("CO2: " + String(CO2_read));
  if(CO2_read > 1000){
    lcd.setCursor(0, 1);
    lcd.print("Extremly High!  ");
  } else if (CO2_read > 500){
      lcd.setCursor(0, 1);          // move cursor to   (0, 1)
      lcd.print("High CO2        "); // print message at (0, 1)
  } else {
      lcd.setCursor(0, 1);          // move cursor to   (0, 1)
      lcd.print("Good CO2        "); // print message at (0, 1)
    }
  //delay(5000);
  }
*/
}

// Functions
float co2PpmConversion(float co2Value){
  //???
}

int poprzednia_wartoscTemp = 0;
void tempLCD(int temperature, int humidity){

  if (temperature != poprzednia_wartoscTemp) {
    lcd.clear();
   lcd.setCursor(0, 0);
   lcd.print("Temp: " + String(temperature));
   lcd.setCursor(0, 1);
   lcd.print("Humidity: " + String(humidity) + "%  ");
  poprzednia_wartoscTemp = temperature;
}


}

int poprzednia_wartoscCo2 = 0;
void co2LCD(float co2Concentration){
  if (co2Concentration != poprzednia_wartoscCo2){
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
    poprzednia_wartoscCo2 = co2Concentration;
}
}

/*
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
*/

// ISR functions

void soilISR(){
  //if(waterState == watering){
  watering = !watering;
  waterState = 0;
  //}
}

void soil2ISR(){
  //if(waterState == watering){
  watering = !watering;
  //}
}

  void buttonISR(){
      unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > debounceDelay) {
    buttonState = !buttonState;
    lastDebounceTime = currentTime;
    poprzednia_wartoscTemp = 0;
    poprzednia_wartoscCo2 = 0;
  }
    //if(prevButtonState == buttonState){
    //buttonState = !buttonState;
    //}
  }

  ISR(PCINT0_vect) {
    //if(co2State == co2Alarm){
    co2Alarm = !co2Alarm;
    //}
}