// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>
#include "MS5837.h"
MS5837 sensor;
int pulse_ready;
int latest_depth;
int event;

void setup()
{
  d1_flag=0;
  d2_flag=0;
  pulse_ready=1;
  event=0;//initial
  latest_depth=0;
  Wire.begin(42);                // join i2c bus with address #9
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  Serial.begin(9600);           // start serial for output
  Wire.begin();

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  
  sensor.setModel(MS5837::MS5837_02BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void loop() {
  // Update pressure and temperature readings
  if(pulse_ready==1) {
    sensor.read();
    pulse_ready=0;
  }
  if(d1_flag==1) {
    sensor.calculate_D1();
    d1_flag=0;
  }
  if(d2_flag==1) {
    sensor.calculate_D2();
    d2_flag=0;
  }
  if(sensor.read_ready==1) {
    /*Serial.print("Pressure: "); 
    Serial.print(sensor.pressure()); 
    Serial.println(" mbar");*/
    
    Serial.print("Temperature: "); 
    Serial.print(sensor.temperature()); 
    Serial.println(" deg C");
    
    Serial.print("Depth: "); 
    Serial.print(sensor.depth()); 
    Serial.println(" m");
    
    /*Serial.print("Altitude: "); 
    Serial.print(sensor.altitude()); 
    Serial.println(" m above mean sea level");*/
    latest_depth=sensor.depth();
    pulse_ready=1;
    sensor.read_ready=0;
  }
  //delay(1000);
  
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  event=1; //event for depth 
}

void requestEvent()
{
  switch(event) {
    case(1) :
      Wire.write(latest_depth);
    default :
      Serial.println("Error sending");
  }
}
