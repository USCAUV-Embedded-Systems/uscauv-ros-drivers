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
#include <ros/ros.h>
#include <i2c_cpp/I2C.h>

I2C::I2C(int bus, int address) {
	_i2cbus = bus;
	_i2caddr = address;
	snprintf(busfile, sizeof(busfile), "/dev/i2c-%d", bus);
}

I2C::~I2C() {
}

uint8_t I2C::read_byte(uint8_t address)
{
	return 0;

}

std::vector<uint8_t> I2C::read_bytes(size_t numBytes)
{
	return std::vector<uint8_t>(numBytes, 0);
}

void I2C::write_byte(uint8_t address, uint8_t data)
{

}

void I2C::write_bytes(std::vector<uint8_t> bytes)
{

}


//! Open device file for I2C Device
void I2C::openfd() {

}
