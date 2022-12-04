#include <Wire.h>
#include "rgb_lcd.h"

#define DEBUG

/// SENSORS

#define LOAD_CURRENT           A1
#define SOLAR_VOLTAGE          A2
#define SOLAR_CURRENT          A0
#define GENERATOR_VOLTAGE      A3
#define LOAD_VOLTAGE           A5

#define LOAD_DIVIDER_RATIO      2
#define SOLAR_DIVIDER_RATIO     2
#define GENERATOR_DIVIDER_RATIO 2

#define SHORT_CURRENT          A4
#define SHORT_RELAY             2
#define LOAD_RELAY              3

#define SHORT_DIVIDER_RATIO     2
#define SHORT_RESISTOR         10

#define SOLAR_RESISTOR 10
#define LOAD_RESISTOR  20

unsigned long rawToMilliVolts(int raw) {
	return (unsigned long) ((raw * 5000) / 1024);
}

unsigned long calculateMilliAmpere(unsigned long voltage, unsigned long resistance) {
	return (unsigned long) (voltage / resistance);
}

unsigned long calculateMilliWatts(unsigned long voltage, unsigned long current) {
	return (unsigned long) (voltage * current);
}

unsigned long measureLoadVoltage() {
	return rawToMilliVolts(analogRead(LOAD_VOLTAGE) * LOAD_DIVIDER_RATIO);
}

unsigned long measureGeneratorVoltage() {
  return rawToMilliVolts(analogRead(GENERATOR_VOLTAGE)*GENERATOR_DIVIDER_RATIO);
}

unsigned long measureLoadCurrent() {
	return calculateMilliAmpere(
		rawToMilliVolts(analogRead(LOAD_CURRENT)),
		LOAD_RESISTOR
	);
}

unsigned long measureSolarVoltage() {
  return rawToMilliVolts(analogRead(SOLAR_VOLTAGE) * SOLAR_DIVIDER_RATIO);
}

unsigned long measureSolarCurrent() {
  return calculateMilliAmpere(
    rawToMilliVolts(analogRead(SOLAR_CURRENT)),
    SOLAR_RESISTOR
  );
}

unsigned long measureShortVoltage(int halfMeasurementDuration = 10) {
	digitalWrite(LOAD_RELAY, LOW);
	digitalWrite(SHORT_RELAY, HIGH);

	delay(halfMeasurementDuration);

	unsigned long shortVoltage = measureLoadVoltage();
	
	delay(halfMeasurementDuration);
	digitalWrite(SHORT_RELAY, LOW);
	digitalWrite(LOAD_RELAY, HIGH);

	return shortVoltage;
}

unsigned long calculateMaxSolarPower(int halfMeasurementDuration = 10) {
	unsigned long voltage = measureShortVoltage(halfMeasurementDuration);
	return calculateMilliWatts(
		voltage,
		calculateMilliAmpere(voltage, SHORT_RESISTOR)
	);
}

unsigned long calculateSolarPower() {
	return calculateMilliWatts(
		measureSolarVoltage(),
		measureSolarCurrent()
	);
}

unsigned long calculateLoadPower() {
	return calculateMilliWatts(
		measureLoadVoltage(),
		calculateMilliAmpere(measureLoadCurrent(), LOAD_RESISTOR)
	);
}

unsigned long calculateGeneratorCurrent() {
	return measureLoadCurrent() - measureSolarCurrent();
}

unsigned long calculateGeneratorPower() {
	return calculateMilliWatts(
		measureGeneratorVoltage(),
		calculateGeneratorCurrent()
	);
}

void sensorSetup() {
	pinMode(LOAD_CURRENT, INPUT);
	pinMode(LOAD_VOLTAGE, INPUT);
	pinMode(SOLAR_VOLTAGE, INPUT);
	pinMode(SOLAR_CURRENT, INPUT);
	pinMode(GENERATOR_VOLTAGE, INPUT);
	
	pinMode(SHORT_CURRENT, INPUT);
	pinMode(SHORT_RELAY, OUTPUT);
}


char buff[10];
rgb_lcd lcd;

void setup() {
  Serial.begin(9600);

	lcd.begin(16,2);
	lcd.setRGB(0xFF, 0xFF, 0xFF);

  lcd.print("DES");
  Serial.println("DES");

	pinMode(LOAD_RELAY, OUTPUT);
	
	sensorSetup();

	digitalWrite(LOAD_RELAY, HIGH);
 Serial.println("done with setup");
}

void loop() {
	// unsigned long sumSolarVolt = 0;
	// unsigned long sumSolarCurr = 0;

//  lcd.clear();
//  lcd.setCursor(0,1);
// lcd.print("aaaaa");
 // delay(500);

	// for(int i = 0; i < 10; i++) {
//		sumSolarVolt += measureSolarVoltage();
//		sumSolarCurr += measureSolarCurrent();
//		delay(100);
//	}

//	lcd.clear();

//	lcd.setCursor(0,0);
//	sprintf(buff, "%04d", sumSolarVolt/10);
//  Serial.println("U=" + (String) buff + "mV");
	
//	lcd.setCursor(8,0);
//	sprintf(buff, "%04d", sumSolarCurr/10);
//  Serial.println("I=" + (String) buff + "mA");

//	lcd.setCursor(0,1);
//	sprintf(buff, "%04d", sumSolarVolt * sumSolarCurr / 100);
//	Serial.println("P=" + (String) buff + "mW");

//	lcd.setCursor(8,1);
//	sprintf(buff, "%04d", calculatenarPower());
//  Serial.println("P=" + (String) buff + "mW");
Serial.println("Load      U="+(String)measureLoadVoltage()+"; I="+(String)measureLoadCurrent()+"; P="+(String)calculateLoadPower());
Serial.println("Solar     U="+(String)measureSolarVoltage()+"; I="+(String)measureSolarCurrent()+"; P="+(String)calculateSolarPower());
Serial.println("Generator U="+(String)measureGeneratorVoltage()+"; I="+(String)calculateGeneratorCurrent()+"; P="+(String)calculateGeneratorPower());
Serial.println("Max Power P="+(String)measureMaxSolarPower());
delay(1000);
}
