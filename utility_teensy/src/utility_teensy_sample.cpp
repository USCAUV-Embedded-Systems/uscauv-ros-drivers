#include "utility_teensy_sample.h"
//if you want to send I2C messages, then you need to include this header.
#include <ros/ros.h>

// Ros will automatically make header files for each of your service files
// make sure to include them in order to be able to use the service
#include "ros_marker_dropper/DropMarker.h"

//If you don't need to send or recieve anything, you can use std_srvs::Empty
#include <std_srvs/Empty.h>
#include <stdio.h>

//Add new services here
/*Below are two sample services, one uses the DropMarker serivce, the other
use the Empty standard service.
*/
bool drop(ros_marker_dropper::DropMarker::Request & request, ros_marker_dropper::DropMarker:: & response){
    //Insert Code Here
}
bool drop_marker(std_srvs::Empty::Request & request, std_srvs::Empty::Response & response){
    // Declare an i2c object in your header. I2C objects are declared in I2C.h, check the
    // header file for more info.

    //should just write 0x42 to the arduino
    i2c->write_byte(ARDUINO_ADDR, 0x42);
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
