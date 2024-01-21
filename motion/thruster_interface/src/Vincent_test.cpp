
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

const int I2C_ADDRESS = 0x21;
const char *I2C_DEVICE = "/dev/i2c-1";
int8_t i2c_slave_addr = 0x21;

bool started = false;

std::vector<uint8_t> pwm_to_bytes(std::vector<uint16_t> pwm_values);

int main() {
  int file;
  if (started == false) {
    if ((file = open(I2C_DEVICE, O_RDWR)) < 0) {
      std::cerr << "error, could not connect to i2c bus" << std::endl;
      return 0;
    } else {
      std::cout << "connected to i2c bus!!" << std::endl;
    }
    if (ioctl(file, I2C_SLAVE, i2c_slave_addr) < 0) {
      std::cerr << "error, could not set adress" << std::endl;
      close(file);
      return 0;
    } else {
      std::cout << "i2c adress set" << std::endl;
    }
    started = true;
  }

  //----------------------------------------------------------------------

  // int8_t status = 0; //0 if everything goes well and 1 if there is a problem
  // ; change later has to come from somewhere

  //     if(write(file, &status, 1) != 1){
  //         std::cerr << "error, could not send status" << std::endl;
  //         close(file);
  //         return 0;
  //     }else{
  //         std::cout<<"status data has been sent"<<std::endl;
  //     }

  //----------------------------------------------------------------------

  std::vector<uint16_t> pwm_values = {
      1100, 2233, 5533, 1900}; // change later has to come from somewhere
  std::vector<uint8_t> bytes = pwm_to_bytes(pwm_values);

  write(file, bytes.data(), bytes.size());

  // for (int i = 0; i < bytes.size(); ++i) {

  //     if(write(file, &bytes[i], 8) != 1){
  //         std::cerr << "error, could not send PWM data" << std::endl;
  //         close(file);
  //         return 0;
  //     }else{
  //         std::cout<<"PWM data has been sent"<<std::endl;
  //     }

  // }

  close(file);

  return 1;
}

//----------------------------------------------------------------------

std::vector<uint8_t> pwm_to_bytes(std::vector<uint16_t> pwm_values) {

  std::vector<uint8_t> bytes;

  for (int i = 0; i < pwm_values.size(); ++i) {
    // Split the integer into most significant byte (MSB) and least significant
    // byte (LSB)
    uint8_t msb = (pwm_values[i] >> 8) & 0xFF;
    uint8_t lsb = pwm_values[i] & 0xFF;
    // Add MSB and LSB to the bytes vector
    bytes.push_back(msb);
    bytes.push_back(lsb);
  }
  return bytes;
}
