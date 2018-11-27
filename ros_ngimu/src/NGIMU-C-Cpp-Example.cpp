/**
 * @file NGIMU-Teensy-IO-Expansion-Example.ino
 * @author Seb Madgwick
 * @brief Example for receiving data from an NGIMU on a Teensy via the NGIMU
 * serial interface.
 *
 * Device:
 * Teensy 3.2
 *
 * IDE/compiler:
 * Arduino 1.8.5 and Teensy Loader 1.4
 *
 * The OSC99 source files (i.e. the "Osc99" directory) must be added to the
 * Arduino libraries folder.  See: https://www.arduino.cc/en/guide/libraries
 *
 * Lower performance devices such as the Arduino MEGA do not have enough memory
 * to use this example 'as is'.  The value of MAX_TRANSPORT_SIZE must be reduced
 * to 150 in OscCommon.h if this example is used on such devices.
 */

//------------------------------------------------------------------------------
// Includes

#include "NgimuReceive.h"

#include <asio.hpp>

#include <memory>
#include <iostream>

asio::io_service io_service;
std::shared_ptr<asio::serial_port> serial_port;
uint8_t byteBuffer;

void ngimuReceiveErrorCallback(const char* const errorMessage);
void ngimuSensorsCallback(const NgimuSensors ngimuSensors);
void ngimuQuaternionCallback(const NgimuQuaternion ngimuQuaternion);
void ngimuEulerCallback(const NgimuEuler ngimuEuler);

void onSerialRead(const asio::error_code &errorCode, size_t bytes_transferred)
{
	assert(bytes_transferred == 1);

	NgimuReceiveProcessSerialByte(byteBuffer);

	// kick off another read
	asio::async_read(*serial_port, asio::buffer(&byteBuffer, 1), std::bind(&onSerialRead, std::placeholders::_1, std::placeholders::_2));
}


int main() {

    // Initialise IMU serial
    serial_port = std::make_shared<asio::serial_port>(io_service, "/dev/ttyACM0");
	serial_port->set_option(asio::serial_port::baud_rate(115200));

    // Initialise NGIMU receive module
    NgimuReceiveInitialise();

    // Assign NGIMU receive callback functions
    NgimuReceiveSetReceiveErrorCallback(ngimuReceiveErrorCallback);
    NgimuReceiveSetSensorsCallback(ngimuSensorsCallback);
    //NgimuReceiveSetQuaternionCallback(ngimuQuaternionCallback);
    NgimuReceiveSetEulerCallback(ngimuEulerCallback);

	// start the io service reading
	asio::async_read(*serial_port, asio::buffer(&byteBuffer, 1), std::bind(&onSerialRead, std::placeholders::_1, std::placeholders::_2));

	io_service.run();
}

// This function is called each time there is a receive error
void ngimuReceiveErrorCallback(const char* const errorMessage) {
    std::cout << errorMessage << std::endl;
}

// This function is called each time a "/sensors" message is received
void ngimuSensorsCallback(const NgimuSensors ngimuSensors) {
    std::cout << "/sensors, "
    << ngimuSensors.gyroscopeX
    << ", "
    << ngimuSensors.gyroscopeY
    << ", "
    << ngimuSensors.gyroscopeZ
    << ", "
    << ngimuSensors.accelerometerX
    << ", "
    << ngimuSensors.accelerometerY
    << ", "
    << ngimuSensors.accelerometerZ
    << ", "
    << ngimuSensors.magnetometerX
    << ", "
    << ngimuSensors.magnetometerY
    << ", "
    << ngimuSensors.magnetometerZ
    << ngimuSensors.barometer
    << std::endl;
}

// This function is called each time a "/quaternion" message is received
void ngimuQuaternionCallback(const NgimuQuaternion ngimuQuaternion) {
    std::cout <<"/quaternion, "
    << ngimuQuaternion.w
    << ", "
    << ngimuQuaternion.x
    << ", "
    << ngimuQuaternion.y
    << ", "
    << ngimuQuaternion.z
    << std::endl;
}

// This function is called each time a "/euler" message is received.
void ngimuEulerCallback(const NgimuEuler ngimuEuler) {
    std::cout << "/euler, "
    << ngimuEuler.roll
    << ", "
    << ngimuEuler.pitch
    << ", "
    << ngimuEuler.yaw
    << std::endl;
}
