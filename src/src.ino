
/// SENSORS

#define MAX_CURRENT_SENSOR A0
#define LOAD_CURRENT_SENSOR A1
#define VOLTAGE_SENSOR A2
#define MAX_CURRENT_RELAY 2
#define LOAD_RELAY 3

// resistance in Ohm
#define MAX_CURRENT_RESISTOR 10
#define MAX_VOLT_DIVIDER_RATIO 2


// #define DEBUG

void sensorSetup() {
  pinMode(MAX_CURRENT_RELAY, OUTPUT);
  pinMode(MAX_CURRENT_SENSOR, INPUT);
  pinMode(LOAD_RELAY, OUTPUT);

  pinMode(LOAD_CURRENT_SENSOR, INPUT);

  pinMode(VOLTAGE_SENSOR, INPUT);

  digitalWrite(LOAD_RELAY, HIGH); // TODO do not
}

// returns power in milliwatt
unsigned long measureMaxPower(int halfMeasurementDuration = 1) {
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

  #ifdef DEBUG
    Serial.print("Raw: ");
    Serial.print(rawVoltage);
    Serial.print("; mV: ");
    Serial.print(voltage);
    Serial.print("; mW: ");
    Serial.print(wattage);
    Serial.println();
  #endif

  return wattage;
}


/// MAIN

void setup() {
  Serial.begin(115200);

  sensorSetup();
}

void loop() {
  Serial.println(measureMaxPower(10));
  
  delay(500);
}
