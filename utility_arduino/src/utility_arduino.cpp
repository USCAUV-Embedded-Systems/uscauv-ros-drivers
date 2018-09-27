#include <i2c_cpp/I2C.h>
#include <ros/ros.h>
#include <thread>
#include <vector>
#include <iostream>

const int arduinoI2CAddress = 0x08;
const char eventPressure = 'a';
const char eventButton = 'b';
const char eventC = 'c';
const char eventD = 'd';

int main(int argc, char** argv)
{
  I2C i2c(1, arduinoI2CAddress);

  double pressure = 0;
  int buttonState = 1;
  double c = 0;
  double d = 0;

  while (true)
  {
    i2c.write_byte(arduinoI2CAddress, eventPressure); // send event for pressure value
    i2c.write_byte(arduinoI2CAddress, eventButton);
    i2c.write_byte(arduinoI2CAddress, eventC);
    i2c.write_byte(arduinoI2CAddress, eventD);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    try
    {
      std::vector<uint8_t> pressureBytes = i2c.read_bytes(2);
      pressure = (pressureBytes[0] << 8) | pressureBytes[1];
    }
    catch(I2CException & ex)
    {

    }

    try
    {
      std::vector<uint8_t> buttonBytes = i2c.read_bytes(2);
      buttonState = (buttonBytes[0] << 8) | buttonBytes[1];
    }
    catch(I2CException & ex)
    {

    }

    try
    {
      std::vector<uint8_t> cBytes = i2c.read_bytes(2);
      c = (cBytes[0] << 8) | cBytes[1];
    }
    catch(I2CException & ex)
    {

    }

    try
    {
      std::vector<uint8_t> dBytes = i2c.read_bytes(2);
      d = (dBytes[0] << 8) | dBytes[1];
    }
    catch(I2CException & ex)
    {

    }
  }
}
