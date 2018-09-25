<<<<<<< HEAD
#include <Wire.h>

const int PIN_READ_PRESSURE = A0;
const int PIN_READ_SWITCH1 = D1;
const int PIN_READ_SWITCH2 = D2;
const int PIN_READ_SWITCH3 = D3;
const int PIN_READ_SWITCH4 = D4;
//READ_PIN#

const int I2C_ADDRESS = 0x08; 
//I2C_ADDRESS

const int BUFFER_LEN = 4;

void setup()
{
  // put your setup code here, to run once:
  
  int val = 0;
  int pressure = 0;
  int 
  //declare variables

  unsigned char i2c_buffer[BUFFER_LEN] = {'\0'};
  //declare the buffer

  pinMode(PIN_READ_PRESSURE, INPUT);
  pinMode(PIN_READ_SWITCH1, INPUT);
  pinMode(PIN_READ_SWITCH3, INPUT);
  pinMode(PIN_READ_SWITCH3, INPUT);
  //set read pin
  
  Wire.begin(I2C_ADDRESS); //join I2C as slave with I2C_ADDRESS
  Wire.onRequest(requestEvent); // register event executed when requested
  Wire.onReceive(receiveEvent); // register event executed when receive information
    
  Serial.begin(9600);
}

void loop() 
{  
    val = analogRead(READ_PIN_PRESSURE);
    // read raw analogue value
    //Serial.println(val);

    pressure = map(val, 95, 1023, 0, 103.42);
    // project raw analogue value into kPa
    //Serial.print(pressure);
    //Serial.println(" kPa");

    

}

void requestEvent()
{
  // executes when mentioned by master

  Wire.write(0x06);
}

void receiveEvent(int numBytes)
{
  // executes when receives message sent by master
 
  char id = Wire.read();
  // receive the bite

  if(id == 'a')
  {
    for(int i = 0; i < BUFFER_LEN; i++)
    {
      i2c_buffer[3 - i] = (pressure >> (i * 8));
    }
    // fill the buffer with pressure value

    Wire.write(i2c_buffer, BUFFER_LEN);
    // respond with pressure of BUFFER_LEN bytes (int) and '\0' as sentinel
  } 
  else if (id == 'b')
  {
    for(int i = 0; i < BUFFER_LEN; i++)
    {
      i2c_buffer[3 - i] = (temperature >> (i * 8));
    }
    // fill the buffer with temperature value

    Wire.write(i2c_buffer, BUFFER_LEN);
    // respond with temperature of BUFFER_LEN bytes (int) and '\0' as sentinel
  }
}
=======
#include <Wire.h>

const int PIN_READ_PRESSURE = A0;
const int SWITCH1 = D1;
const int SWITCH2 = D2;
const int SWITCH3 = D3;
const int SWITCH4 = D4;
int READ_SWITCH1;
int READ_SWITCH2;
int READ_SWITCH3;
int READ_SWITCH4;

//READ_PIN#

const int I2C_ADDRESS = 0x08; 
//I2C_ADDRESS

const int BUFFER_LEN = 4;

void setup()
{
  // put your setup code here, to run once:
  
  int val = 0;
  int pressure = 0;
  
  //declare variables

  unsigned char i2c_buffer[BUFFER_LEN] = {'\0'};
  //declare the buffer

  pinMode(PIN_READ_PRESSURE, INPUT);
  pinMode(SWITCH1, INPUT);
  pinMode(SWITCH3, INPUT);
  pinMode(SWITCH3, INPUT);
  pinMode(SWITCH4, INPUT);
  //set read pin
  
  Wire.begin(I2C_ADDRESS); //join I2C as slave with I2C_ADDRESS
  Wire.onRequest(requestEvent); // register event executed when requested
  Wire.onReceive(receiveEvent); // register event executed when receive information
    
  Serial.begin(9600);
}

void loop() 
{  
    val = analogRead(READ_PIN_PRESSURE);
    // read raw analogue value
    //Serial.println(val);

    pressure = map(val, 95, 1023, 0, 103.42);
    // project raw analogue value into kPa
    //Serial.print(pressure);
    //Serial.println(" kPa");

    READ_SWITCH1 = digitalRead(SWITCH1);
    READ_SWITCH2 = digitalRead(SWITCH2);
    READ_SWITCH3 = digitalRead(SWITCH3);
    READ_SWITCH4 = digitalRead(SWITCH4);

    

    
    

}

void requestEvent()
{
  // executes when mentioned by master

  Wire.write(0x06);
}

void receiveEvent(int numBytes)
{
  // executes when receives message sent by master
 
  char id = Wire.read();
  // receive the bite

  if(id == 'a')
  {
    for(int i = 0; i < BUFFER_LEN; i++)
    {
      i2c_buffer[3 - i] = (pressure >> (i * 8));
    }
    // fill the buffer with pressure value

    Wire.write(i2c_buffer, BUFFER_LEN);
    // respond with pressure of BUFFER_LEN bytes (int) and '\0' as sentinel
  } 
  else if (id == 'b')
  {
    for(int i = 0; i < BUFFER_LEN; i++)
    {
      i2c_buffer[3 - i] = (temperature >> (i * 8));
    }
    // fill the buffer with temperature value

    Wire.write(i2c_buffer, BUFFER_LEN);
    // respond with temperature of BUFFER_LEN bytes (int) and '\0' as sentinel
  }
}
>>>>>>> 724aaaecf9ff4e72e3a67b3c88047fd46ad24153
