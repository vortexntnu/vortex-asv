import smbus
import time
import struct

class TemperatureModule:
    def __init__(self):
        # to read temperature from arduino through I2C
        self.i2c_address = 0x22  

        # init of I2C bus communication
        self.bus = smbus.SMBus(1)
        time.sleep(1)

    def get_data(self):
        try:
            # Define the number of floats and the corresponding byte length
            num_floats = 6
            byte_length = num_floats * 4  # Each float is 4 bytes

            # Read a block of bytes from the I2C device
            data = self.bus.read_i2c_block_data(self.i2c_address, 0, byte_length)

            # Convert the byte list to a list of floats
            floats = struct.unpack('<' + 'f' * num_floats, bytes(data[:byte_length]))

            return floats
        except Exception as e:
            print(f"Error: {e}")
            return None

    def shutdown(self):
        self.bus.close()