#include <ros/ros.h>
#include "ros_marker_dropper/DropMarker.h"

bool drop_marker(ros_marker_dropper::DropMarker::Request & request,
                 ros_marker_dropper::DropMarker::Response & response){
    I2C* i2c = new I2C(MODE1,  ARDUINO_ADDR);
    //should just write 0x42 to the arduino
    i2c->write_byte(ARDUINO_ADDR, 0x42);
    //I don't know if this is enough, but we can dream 

}

int main(int argc, char **argv){
    ros::init(argc, argv, "marker_dropper_server");
    ros::NodeHandle n;
    
    ros::ServiceServer service = n.advertiseService("marker_dropper", drop_marker);
    ROS_INFO("Ready to drop marker");
    ros::spin();
    return 0;
}

