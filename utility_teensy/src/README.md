All of the cpp and header files should go in here.

If you want to send I2C messages, you need to include the I2C library.
This is already done in the sample header.
Just in case here is the I2C header file:
//I2C Definitions for reference:
// I2C(int bus, int address);
// virtual ~I2C();

// read a byte from an address on the device
// uint8_t read_byte(uint8_t deviceAddress);

// read a number of bytes from the device
// std::vector<uint8_t> read_bytes(size_t numBytes);

// write one byte to a specific address on the device
// void write_byte(uint8_t deviceAddress, uint8_t byte);

// write a number of bytes to the device
// void write_bytes(std::vector<uint8_t> bytes);

Any dependencies that you may need should be added to CMakeLists.txt

utility_teensy_sample.cpp is a sample of how to set up a new ROS node.
Below is the main function annotated:

Use this to declare the name of the node:
ros::init(argc, argv, "marker_dropper_server");

This lets you interact with the node in different ways:
ros::NodeHandle n;

Reference this for more info:
http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown

This line declares a service. The second argument is a function that the service
will call (More info on services: http://wiki.ros.org/srv):
ros::ServiceServer service = n.advertiseService("marker_dropper", drop_marker);

This prints to ROS_INFO, then std;:cout:
ROS_INFO("Ready to drop marker");
std::cout << "Testing" << std::endl;

Ros::spin will keep the node running forver:
ros::spin();
return 0;
