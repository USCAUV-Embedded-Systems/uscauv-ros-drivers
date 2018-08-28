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
			ROS_ERROR("I2C slave 0x%x failed to go to register 0x%x [read_byte():write %d]", _i2caddr, address, errno);
			return 0;
		}
		else 
		{
			uint8_t dataBuffer;
			if (read(fd, &dataBuffer, 1) != 1) 
			{
				ROS_ERROR("Could not read from I2C slave 0x%x, register 0x%x [read_byte():read %d]", _i2caddr, address, errno);
				return 0;
			} 
			else 
			{
				return dataBuffer;
			}
		}
	} 
	else 
	{
		ROS_ERROR("Device File not available. Aborting read");
		return 0;
	}

}

std::vector<uint8_t> I2C::read_bytes(size_t numBytes)
{
	if(fd == -1)
	{
		ROS_ERROR("Device File not available. Aborting read");
		return {};
	}
	
	uint8_t * byteBuffer = new uint8_t[numBytes];
	
	if (read(fd, byteBuffer, numBytes) != numBytes) 
	{
		delete byteBuffer;
		ROS_ERROR("Could not read %lu bytes from from I2C slave 0x%x [read_byte():read %d]", numBytes, _i2caddr, errno);
		return {};
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
			ROS_ERROR("Failed to write to I2C Slave 0x%x @ register 0x%x [write_byte():write %d]", _i2caddr, address, errno);
		}
	}
	else 
	{
		ROS_ERROR("Device File not available. Aborting write");
	}
}

void I2C::write_bytes(std::vector<uint8_t> bytes)
{
	if(fd == -1)
	{
		ROS_ERROR("Device File not available. Aborting read");
		return;
	}
	
	if (write(fd, bytes.data(), bytes.size()) != bytes.size()) 
	{
		ROS_ERROR("Failed to write %lu bytes to I2C Slave 0x%x [write_byte():write %d]", bytes.size(), _i2caddr, errno);
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
