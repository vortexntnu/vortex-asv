
#include "I2C.hpp"

const int I2C_ADDRESS = 0x21;
const char *I2C_DEVICE = "/dev/i2c-1";
int8_t i2c_slave_addr = 0x21;

bool started = false;

int main() {

    std::vector<uint16_t> pwm_values = {1100,2233,5533,1900}; //change later has to come from somewhere
    int8_t status = 0; //0 if everything goes well and 1 if there is a problem ; change later has to come from somewhere
    int communication_choice = 0; //0 for sending (normal case), 1 for receiving
    
    int file;
    if(started == false){
        init(file);
        started = true;
    }


    send_communication_choice(communication_choice, file);
    send_status(status, file);
    send_pwm(pwm_values, file);

    close(file);
    return 1;
}

//----------------------------------------------------------------------


void init(int &file){
    if((file = open(I2C_DEVICE, O_RDWR)) < 0){
                std::cerr << "error, could not connect to i2c bus" << std::endl;
                close(file);
                exit(EXIT_FAILURE);
            }
            else{
                std::cout<<"connected to i2c bus!!"<<std::endl;
            }
            if(ioctl(file, I2C_SLAVE, i2c_slave_addr) < 0){
                std::cerr << "error, could not set adress" << std::endl;
                close(file);
                exit(EXIT_FAILURE);
            }
            else{
                std::cout<<"i2c adress set"<<std::endl;
            }
}



void send_communication_choice(int8_t choice, int file){

    if(write(file, &choice, 1) != 1){
    std::cerr << "error, could not send communication choice" << std::endl;
    close(file);
    //exit(EXIT_FAILURE);
    }else{
        std::cout<<"communication choice has been sent"<<std::endl;
    }
}

     
void send_status(int8_t status, int file){
    if(write(file, &status, 1) != 1){
    std::cerr << "error, could not send status" << std::endl;
    close(file);
    //exit(EXIT_FAILURE);
    }else{
        std::cout<<"status data has been sent"<<std::endl;
    }

}


void send_pwm(std::vector<uint16_t> pwm_values, int file){
    std::vector<uint8_t> bytes = pwm_to_bytes(pwm_values); 
    if(write(file, bytes.data(), bytes.size()) != bytes.size()){
        std::cerr << "error, could not send PWM data" << std::endl;
        close(file);
        exit(EXIT_FAILURE);

    }else{
        std::cout<<"PWM data has been sent"<<std::endl;
    }

}
 

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





