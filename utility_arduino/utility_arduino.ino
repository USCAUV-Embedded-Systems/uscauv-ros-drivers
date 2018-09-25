#include <Wire.h>

const int PIN_READ_PRESSURE = A0;
const int SWITCH2 = 2;
const int SWITCH3 = 3;
const int SWITCH4 = 4;
const int SWITCH5 = 5;
//declare the pin

int READ_SWITCH1;
int READ_SWITCH2;
int READ_SWITCH3;
int READ_SWITCH4;
int val = 0;
int pressure = 0;
//declare variables

const int BUFFER_LEN = 4;
unsigned char i2c_buffer[BUFFER_LEN] = {'\0'};
//declare the buffer

const int I2C_ADDRESS = 0x08; 
//I2C_ADDRESS

void setup()
{
  // put your setup code here, to run once:
  
  pinMode(PIN_READ_PRESSURE, INPUT);
  pinMode(SWITCH1, INPUT_PULLUP);
  pinMode(SWITCH3, INPUT_PULLUP);
  pinMode(SWITCH3, INPUT_PULLUP);
  pinMode(SWITCH4, INPUT_PULLUP);
  //set read pin
  
  Wire.begin(I2C_ADDRESS); //join I2C as slave with I2C_ADDRESS
  Wire.onRequest(requestEvent); // register event executed when requested
  Wire.onReceive(receiveEvent); // register event executed when receive information
    
  Serial.begin(9600);
}

void loop() 
{  
    val = analogRead(PIN_READ_PRESSURE);
    // read raw analogue value
    //Serial.println(val);

    pressure = map(val, 95, 1023, 0, 103.42);
    // project raw analogue value into kPa
    Serial.print(pressure);
    //Serial.println(" kPa");

    READ_SWITCH1 = digitalRead(SWITCH1);
    READ_SWITCH2 = digitalRead(SWITCH2);
    READ_SWITCH3 = digitalRead(SWITCH3);
    READ_SWITCH4 = digitalRead(SWITCH4);
    
 
    Serial.println(READ_SWITCH1);
    Serial.println(READ_SWITCH2);
    Serial.println(READ_SWITCH3);
    Serial.println(READ_SWITCH4);

    delay(100);  

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
    /*for(int i = 0; i < BUFFER_LEN; i++)
    {
      i2c_buffer[3 - i] = (temperature >> (i * 8));
    }
    // fill the buffer with temperature value

    Wire.write(i2c_buffer, BUFFER_LEN);
    // respond with temperature of BUFFER_LEN bytes (int) and '\0' as sentinel*/
  }
}
