import smbus
import time
#from MCP342x import MCP342x

class TemperatureModule:

    def __init__(self):

        # Parameters
        # to read temperature from arduino through I2C
        self.i2c_address = 0x08  

        # init of I2C bus communication
        self.bus = smbus.SMBus(1) #was 1 before
          
        #time.sleep(1)


    def get_temperature(self):
        # Sometimes an I/O timeout or error happens, it will run again when the error disappears
        try:  
            #system_temperature = self.bus.read_byte_data(self.i2c_adress, 1)
            #print(str(self.bus.read_byte_data(self.i2c_adress, 1)))
            #system_temperature = "here is the temperature"
            #system_temperature = str(self.bus.read_byte_data(self.i2c_adress, 1))


            #for i in range(100):
             #   data_received = self.bus.read_i2c_block_data(self.i2c_address, i, 16)
              #  print(f"Data received from register {i}: {data_received}")


            data_received = self.bus.read_i2c_block_data(self.i2c_address, 0, 11)  # Read 16 bytes from address 0
            
            #read_i2c_block_data(i2c_addr, register, length, force=None) --> what register??

            print(data_received)

            string_received = ''.join(chr(i) for i in data_received).strip('\x00')  

            return string_received

        except IOError:
            return
        

    def shutdown(self):
        self.bus.close()