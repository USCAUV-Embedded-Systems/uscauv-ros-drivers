#!/usr/bin/env python
import subprocess

#test to see if we can call the cpp code with python

print("Ready to Drop the Ball");
subprocess.call(["g++", "I2C_Ball_Drop.cpp"])
subprocess.call("./I2C_Ball_Drop")


