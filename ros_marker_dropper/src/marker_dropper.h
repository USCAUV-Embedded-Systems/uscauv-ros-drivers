#ifndef MARKER_DROPPER_H
#define MARKER_DROPPER_H

#include <ros/ros.h>
#include <i2c_cpp/I2C.h>

#define MODE1 0x00
#define ARDUINO_ADDR 0x08

bool drop_marker(ros_marker_dropper::DropMarker::Request & request,
 ros_marker_dropper::DropMarker::Response & response);
 
#endif
