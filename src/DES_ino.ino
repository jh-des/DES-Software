#include <Wire.h>
#include <Servo.h>
#include <Grove_I2C_Motor_Driver>
#include "rgb_lcd.h"

/// SENSORS

#define MAX_CURRENT_SENSOR A0
#define LOAD_CURRENT_SENSOR A1
#define VOLTAGE_SENSOR A2
#define MAX_CURRENT_RELAY 2
#define LOAD_RELAY 3

#define MOTOR_DRIVER_START_RELAY 4
#define PUMP_RATE_PER_10_PERCENT_DUTY 10  //ml/s/10° duty
#define TOP_POOL_VOLUME 100
#define VALVE 9 //requires PWM port
#define VALVE_SERVO_CLOSED 100
#define VALVE_SERVO_OPEN -100
#define R_PUMP_MOTOR 200
#define PIN9_PWM_FREQUENCY 500
#define UPPER_POOL_VOLUME
#define FLOW_RATE_PER_10_PERCENT_DUTY 14

// resistance in Ohm
#define MAX_CURRENT_RESISTOR 10
#define MAX_VOLT_DIVIDER_RATIO 2
#define LOAD_CURRENT_RESISTOR 10

#define DEBUG
int Upper_pool_fill_level = 0; //percentual

/// PERIPHERALS
rgb_lcd lcd;
// ------
void sensorSetup() {
  pinMode(MAX_CURRENT_RELAY, OUTPUT);
  pinMode(MAX_CURRENT_SENSOR, INPUT);
  pinMode(LOAD_RELAY, OUTPUT);

  pinMode(LOAD_CURRENT_SENSOR, INPUT);

  pinMode(VOLTAGE_SENSOR, INPUT);

  digitalWrite(LOAD_RELAY, HIGH); // TODO do not
}

void pumped_storage_init()
{
  pinMode(MOTOR_DRIVER_START_RELAY, OUTPUT);
  pinMode(VALVE, OUTPUT);
  digitalWrite(MOTOR_DRIVER_START_RELAY, HIGH);
  Servo valve;
  valve.attach(9);
  valve.write(VALVE_SERVO_CLOSED);
  Motor.begin();
  Motor.speed(MOTOR1, 0);
}

  void PS_pump(int pump_input_power_mW) //power in mW
  {
    if(flow_rate < VALVE_SERVO_OPEN && flow_rate > 0)
    {
       Motor.speed(MOTOR1, sqrt((R_PUMP_MOTOR * pump_input_power / 1000)/(pow(PIN9_PWM_FREQUENCY,2) * 25)));   
    }
  }
    
  void PS_gen(int flow_rate)
  {
    if(flow_rate < VALVE_SERVO_OPEN && flow_rate > 0)
    {
      valve.write(flow_rate);
    }
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
  Pumped_storage_init();
}

void loadprediction(int t)
{
    pow(((t % 24)-12)/2.5,2); // TODO what does this do
}

int generationprediction(int t)
{
  return 9 * exp( -(pow((((t % 24)-12)/2.5),2)));
}

int predicted_PS_demand(int t)
{
  return 2 - (9 * exp( -(pow((((t % 24)-12)/2.5),2))));
}

int max_permissible_power_draw_t(int t)
{
  
}

void loop() {
  unsigned long sumVolt = 0;
  int counter;
  int i;
  int j;
  int deltatime;
  unsigned long sumCur = 0;
  for(int i = 0; i<5; i++) {
    sumVolt += measureVoltage();
    sumCur += measureLoadCurrent();
    counter ++;
    if(counter > 100)
    {
      int i = measureMaxPower();
      counter = 0;
    }
      if(i - measure_load_current() > 0 || pump_mode == true)
      {
        if(pump_mode == false && gen_mode == false) {pump_mode = true;}
        PS_pump(i - measure_load_current);
      }
      else if(i - measure_load_current == 0 || gen_mode == true)
      {
         if(pump_mode == false && gen_mode == false) {gen_mode = true;}
        PS_gen(30);
      }
    delay(50);
    if(pump_mode == true)
    {
      if(deltatime % 1000 == 0)
      {
        j++;
        upper_pool_fill_level = upper_pool_fill_level + floor(j * (i - measure_load_current) * PUMP_RATE_PER_10_PERCENT_DUTY / UPPER_POOL_VOLUME)
      }
    }
    if(gen_mode == true)
    {
      if(deltatime % 1000 == 0)
      {
        j++;
        upper_pool_fill_level = upper_pool_fill_level - floor(j * (i - measure_load_current) * FLOW_RATE_PER_10_PERCENT_DUTY / UPPER_POOL_VOLUME)
      }
    }
    if(gen_mode == true || pump_mode == true))
    {
      deltatime += 50;
    }
    if(deltatime > 15000)
    {
      pump_mode = 0;
      gen_mode = 0;
      deltatime = 0:
      j = 0;
    }
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
