#include "marker_dropper.h"
#include <i2c_cpp/I2C.h>
#include <ros/ros.h>
#include "ros_marker_dropper/DropMarker.h"
#include <std_srvs/Empty.h>
#include <stdio.h>

bool drop_marker(std_srvs::Empty::Request & request, std_srvs::Empty::Response & response){
    //I2C* i2c = new I2C(MODE1,  ARDUINO_ADDR);
    I2C i2c(1, 0x08);
    //should just write 0x42 to the arduino
    //i2c->write_byte(ARDUINO_ADDR, 0x42);
    //I don't know if this is enough, but we can dream 
    
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "marker_dropper_server");
    ros::NodeHandle n;
    
    ros::ServiceServer service = n.advertiseService("marker_dropper", drop_marker);
    ROS_INFO("Ready to drop marker");
    std::cout << "Testing" << std::endl;
    ros::spin();
    return 0;
}
