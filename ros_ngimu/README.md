## ROS NGIMU
This ROS node implements a driver for the X-IO NGIMU. It supports reading earth acceleration, Euler angles, and quaternion data, and has proven able to deal with corrupt serial data.  

It uses the OSCPack library to decode the OSC binary data, and Asio to read the serial port (so it should work on all major OSs).  Both libraries are bundled in the source code.

To configure options (serial port, node name, etc.), check `ros_ngimu.cpp`.  To change the refresh rate or disable certain data redaouts, just disable them in the NGIMU configuration tool.