#include <Wire.h>

#include "rgb_lcd.h"

/// SENSORS

#define MAX_CURRENT_SENSOR A0
#define LOAD_CURRENT_SENSOR A1
#define VOLTAGE_SENSOR A2
#define MAX_CURRENT_RELAY 2
#define LOAD_RELAY 3

// resistance in Ohm
#define MAX_CURRENT_RESISTOR 10
#define MAX_VOLT_DIVIDER_RATIO 2
#define LOAD_CURRENT_RESISTOR 10

#define DEBUG

/// PERIPHERALS
rgb_lcd lcd;

void sensorSetup() {
  pinMode(MAX_CURRENT_RELAY, OUTPUT);
  pinMode(MAX_CURRENT_SENSOR, INPUT);
  pinMode(LOAD_RELAY, OUTPUT);

  pinMode(LOAD_CURRENT_SENSOR, INPUT);

  pinMode(VOLTAGE_SENSOR, INPUT);

  digitalWrite(LOAD_RELAY, HIGH); // TODO do not
}

// in mA
unsigned long measureLoadCurrent() {
	int rawVoltage = analogRead(LOAD_CURRENT_SENSOR);

	// in mV
	unsigned long voltage = (unsigned long) (rawVoltage+1) * 5000 / 1024;

	// in mA
	unsigned long current = (unsigned long) voltage / LOAD_CURRENT_RESISTOR;
	
	return current;
}

// returns power in mW
unsigned long measureMaxPower(int halfMeasurementDuration = 10) {
  digitalWrite(LOAD_RELAY, LOW);
  digitalWrite(MAX_CURRENT_RELAY, HIGH);
  delay(halfMeasurementDuration);
  int rawVoltage = analogRead(MAX_CURRENT_SENSOR); // in 0-1023
  delay(halfMeasurementDuration);
  digitalWrite(MAX_CURRENT_RELAY, LOW);
  digitalWrite(LOAD_RELAY, HIGH);

  // 0-1023 to mV
  unsigned long voltage = (unsigned long) (rawVoltage+1) * 5000 * MAX_VOLT_DIVIDER_RATIO / 1024;

  // in mW
  unsigned long wattage = (unsigned long) (voltage * voltage) / MAX_CURRENT_RESISTOR / 1000;

  return wattage;
}

// in mV
unsigned long measureVoltage() {
  int rawVoltage = analogRead(MAX_CURRENT_SENSOR); // in 0-1023

  // 0-1023 to mV
  return (unsigned long) (rawVoltage+1) * 5000 * MAX_VOLT_DIVIDER_RATIO / 1024;
}


/// MAIN

char buff[10];

void setup() {
  lcd.begin(16, 2);

  lcd.setRGB(0xFF, 0xFF, 0xFF);

  sensorSetup();
}

void loop() {
  unsigned long sumVolt = 0;
  unsigned long sumCur = 0;
  for(int i = 0; i<5; i++) {
    sumVolt += measureVoltage();
    sumCur += measureLoadCurrent();
    delay(200);
  }
  
  lcd.clear();
  lcd.setCursor(0,0);
  sprintf(buff, "%04d", sumVolt/5);
  lcd.print("U="+(String) buff+"mV");
  lcd.setCursor(8,0);
  sprintf(buff, "%04d", sumCur/5);
  lcd.print("I="+(String)buff+"mA");
  lcd.setCursor(0,1);
  sprintf(buff, "%04d", measureMaxPower());
  lcd.print("P="+ (String) buff+"mW");
}
