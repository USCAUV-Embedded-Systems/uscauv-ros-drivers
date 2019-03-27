#include "I2C_Ball_Drop.h"
#include <sys/ioctl.h>
#include <errno.h>
#include <stdio.h>      /* Standard I/O functions */
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sstream>

int main(int argc, char* argv[]){
    I2C* i2c = new I2C(MODE1,  ARDUINO_ADDR);
    //should just write 0x42 to the arduino
    i2c->write_byte(ARDUINO_ADDR, 0x42);
    //I don't know if this is enough, but we can dream    
}
