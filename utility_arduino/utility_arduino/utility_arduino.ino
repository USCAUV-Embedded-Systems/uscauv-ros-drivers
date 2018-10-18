#include <Wire.h>

const int PIN_READ_PRESSURE = A0;
const int PIN_READ_MOTORBAT = A1;
const int PIN_READ_LOGICBAT = A2;
const int SWITCH1 = 2;
const int SWITCH2 = 3;
const int SWITCH3 = 4;
const int SWITCH4 = 5;
//declare the pin

int READ_SWITCH1;
int READ_SWITCH2;
int READ_SWITCH3;
int READ_SWITCH4;
int val_pres = 0;
int val_motor_batt = 0;
int val_logic_batt = 0;

//declare variables

const int BUFFER_LEN = 8;
unsigned char i2c_buffer[BUFFER_LEN] = {'\0'};
unsigned char button_state;
//declare the buffer

const int I2C_ADDRESS = 0x08; 
//I2C_ADDRESS

void setup()
{
  // put your setup code here, to run once:
  
  pinMode(PIN_READ_PRESSURE, INPUT);
  pinMode(PIN_READ_MOTORBAT, INPUT);
  pinMode(PIN_READ_LOGICBAT, INPUT);
  pinMode(SWITCH1, INPUT_PULLUP);
  pinMode(SWITCH2, INPUT_PULLUP);
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
    val_pres = analogRead(PIN_READ_PRESSURE);
    // read raw analogue value
    //Serial.println(val_pres);

    // pressure = map(val_pres, 95, 1023, 0, 103.42);
    // project raw analogue value into kPa


    val_motor_batt = analogRead(PIN_READ_MOTORBAT);

    //motor_battery = map(val_motor_batt, );        //To be finished

    val_logic_batt = analogRead(PIN_READ_LOGICBAT);

    //logic_battery = map(val_logic_batt, );        //To be finished

    READ_SWITCH1 = digitalRead(SWITCH1);
    READ_SWITCH2 = digitalRead(SWITCH2);
    READ_SWITCH3 = digitalRead(SWITCH3);
    READ_SWITCH4 = digitalRead(SWITCH4);
    
    // read switch switch status
    //Serial.println(READ_SWITCH2);
    //Serial.println(READ_SWITCH3);
    //Serial.println(READ_SWITCH4);
    //Serial.println(READ_SWITCH5);

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
  // receive the bite

  int id = Wire.read();
  
  Wire.write(i2c_buffer, BUFFER_LEN);
  
  /*
  if(id == 97)
  {
    for(int i = 0; i < BUFFER_LEN; i++)
    {
      i2c_buffer[BUFFER_LEN - 1 - i] = (val_pres >> (i * 8));
    }
    // fill the buffer with pressure value

    Wire.write(i2c_buffer, BUFFER_LEN);
    // respond with pressure of BUFFER_LEN bytes (double)
  } 
  else if (id == 'b')
  {
    button_state = READ_SWITCH1 | (READ_SWITCH2 << 1) | (READ_SWITCH3 << 2) | (READ_SWITCH4 << 3);
    // assemble the button state

    Wire.write(button_state);
    // respond with button state
  }
  else if (id == 'c')
  {
    for(int i = 0; i < BUFFER_LEN; i++)
    {
      i2c_buffer[BUFFER_LEN - 1 - i] = (val_motor_batt >> (i * 8));
    }
    // fill the buffer with motor battery value

    Wire.write(i2c_buffer, BUFFER_LEN);
    // respond with motor battery voltage of BUFFER_LEN bytes (double)  
  }
  else if (id == 'd')
  {
    for(int i = 0; i < BUFFER_LEN; i++)
    {
      i2c_buffer[BUFFER_LEN - 1 - i] = (val_logic_batt >> (i * 8));
    }
    // fill the buffer with logic battery value

    Wire.write(i2c_buffer, BUFFER_LEN);
    // respond with logic battery voltage of BUFFER_LEN bytes (double) 
    
  }*/
}
