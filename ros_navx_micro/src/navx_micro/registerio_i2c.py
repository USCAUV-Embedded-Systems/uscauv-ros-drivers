#----------------------------------------------------------------------------
# Copyright (c) Kauai Labs 2015. All Rights Reserved.
#
# Created in support of Team 2465 (Kauaibots).  Go Purple Wave!
#
# Open Source Software - may be modified and shared by FRC teams. Any
# modifications to this code must be accompanied by the \License.txt file
# in the root directory of the project
#----------------------------------------------------------------------------

import threading


from periphery import I2C 
import traceback
import logging
import binascii
logger = logging.getLogger('navx.i2c')

MAX_WPILIB_I2C_READ_BYTES = 127

class RegisterIO_I2C:
    
    def __init__(self, port):
                
        self.i2c = I2C(port)
        self.mutex = threading.Lock()
        
    def init(self):
        logger.info("NavX i2c initialized")
        return True
    
    def write(self, address, value):
        aborted=False
        with self.mutex:
            msgs = [I2C.Message([address | 0x80, value])]
            try:
                self.i2c.transfer(0x32, msgs)
            except:
                aborted=True
            
        if aborted:
            logger.warn("navX-MXP I2C write error")
        return not aborted
    
    def read(self, first_address, count):
        dat_buffer = []
        bytes_already_read = 0
        while bytes_already_read != count:
            read_len = min(MAX_WPILIB_I2C_READ_BYTES, count - bytes_already_read)
            
            with self.mutex:
                try:
                    # first transfer address and length we want to read
                    register_address = first_address + bytes_already_read
                    self.i2c.transfer(0x32, [I2C.Message([register_address, read_len])])
                    
                    # now read the data
                    # NOTE: these two statements should theoretically be able to be combined into a single transaction,
                    # but when I tried, it didn't work
                    read_msg = I2C.Message(bytearray(read_len),read=True)
                    self.i2c.transfer(0x32, [read_msg])
                    
                    #print("Read %d bytes at register %d: %s" % (read_len, register_address, binascii.hexlify(read_msg.data)))
                    if not read_msg.data:
                        break
                    
                    dat_buffer.extend(read_msg.data)
                    bytes_already_read = len(dat_buffer)
                except Exception:
                    traceback.print_exc()
                    break
        
        if bytes_already_read != count:
            raise IOError("Read error")
        
        return dat_buffer
    
    def shutdown(self):
        logger.info("NavX i2c shutdown")
        self.i2c.close()
        return True
