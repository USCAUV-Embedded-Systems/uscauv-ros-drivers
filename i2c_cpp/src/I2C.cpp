/*
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Name        : I2C.cpp
 * Author      : Georgi Todorov
 * Version     :
 * Created on  : Dec 30, 2012
 *
 * Copyright © 2012 Georgi Todorov  <terahz@geodar.com>
 */

#include <sys/ioctl.h>
#include <errno.h>
#include <stdio.h>      /* Standard I/O functions */
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <ros/ros.h>
#include <i2c_cpp/I2C.h>
#include <sstream>

I2C::I2C(int bus, int address) {
	_i2cbus = bus;
	_i2caddr = address;
	snprintf(busfile, sizeof(busfile), "/dev/i2c-%d", bus);
	openfd();
}

I2C::~I2C() {
	close(fd);
}

uint8_t I2C::read_byte(uint8_t address) {
	if (fd != -1) {
		
		uint8_t buff;
		
		buff = address;
		if (write(fd, &buff, 1) != 1) 
		{
			std::stringstream errorMessage;
			errorMessage << "read_byte(): I2C slave " << std::hex << _i2caddr;
			errorMessage << " failed to go to register " << address << " [" << strerror(errno) << ']' << std::endl;
			throw I2CException(errorMessage);
		}
		else 
		{
			uint8_t dataBuffer;
			if (read(fd, &dataBuffer, 1) != 1) 
			{
				std::stringstream errorMessage;
				errorMessage << "read_byte(): Could not read from I2C slave " << std::hex << _i2caddr;
				errorMessage << " register " << address << " [" << strerror(errno) << ']' << std::endl;
				throw I2CException(errorMessage);
			} 
			else 
			{
				return dataBuffer;
			}
		}
	} 
	else 
	{
		throw I2CException("Device File not available. Aborting read");
		return 0;
	}

}

std::vector<uint8_t> I2C::read_bytes(size_t numBytes)
{
	if(fd == -1)
	{
		throw I2CException("Device File not available. Aborting read");
	}
	
	uint8_t * byteBuffer = new uint8_t[numBytes];
	
	if (read(fd, byteBuffer, numBytes) != numBytes) 
	{
		delete byteBuffer;
		
		std::stringstream errorMessage;
		errorMessage << "read_bytes(): Could not read " << std::dec << numBytes << std::hex;
		errorMessage << " bytes from I2C slave " << _i2caddr << " [" << strerror(errno) << ']' << std::endl;
		throw I2CException(errorMessage);
	} 
	else 
	{
		// copy bytes to vector
		std::vector<uint8_t> bytes;
		bytes.reserve(numBytes);
		
		for(size_t index = 0; index < numBytes; ++index)
		{
			bytes.push_back(byteBuffer[index]);
		}
		
		delete byteBuffer;
		return bytes;
	}
}

void I2C::write_byte(uint8_t address, uint8_t data) 
{
	if (fd != -1)
	{
		uint8_t buff[2];
		buff[0] = address;
		buff[1] = data;
		if (write(fd, buff, sizeof(buff)) != 2) 
		{
			std::stringstream errorMessage;
			errorMessage << "write_byte(): Failed to write to I2C slave " << std::hex << _i2caddr;
			errorMessage << " register " << address << " [" << strerror(errno) << ']' << std::endl;
			throw I2CException(errorMessage);
		}
	}
	else 
	{
		throw I2CException("Device File not available. Aborting write");
	}
}

void I2C::write_bytes(std::vector<uint8_t> bytes)
{
	if(fd == -1)
	{
		throw I2CException("Device File not available. Aborting read");
	}
	
	if (write(fd, bytes.data(), bytes.size()) != bytes.size()) 
	{
		std::stringstream errorMessage;
		errorMessage << "write_bytes(): Could not write " << std::dec << bytes.size() << std::hex;
		errorMessage << " bytes to I2C slave " << _i2caddr << " [" << strerror(errno) << ']' << std::endl;
		throw I2CException(errorMessage);		
	}
}


//! Open device file for I2C Device
void I2C::openfd() {
	if ((fd = open(busfile, O_RDWR)) < 0) {
		ROS_ERROR("Couldn't open I2C Bus %d [openfd():open %d]", _i2cbus,
				errno);
	}
	if (ioctl(fd, I2C_SLAVE, _i2caddr) < 0) {
		ROS_ERROR("I2C slave %d could not be opened [openfd():ioctl %s]", _i2caddr, strerror(errno));
	}
}
