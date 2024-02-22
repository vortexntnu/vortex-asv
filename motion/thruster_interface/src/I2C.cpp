
#include "I2C.hpp"

const int I2C_ADDRESS = 0x21;
const char *I2C_DEVICE = "/dev/i2c-1";
int8_t i2c_slave_addr = 0x21;

bool started = false;

int8_t hardware_status = 0;

int main() {

    std::vector<uint16_t> pwm_values = {1100,2233,5533,1900}; //change later has to come from somewhere
    uint8_t software_killswitch = 0; //0 if everything goes well and 1 if there is a problem ; change later has to come from somewhere
    std::vector<float> temperature;

    int file;
    if(started == false){
        init(file);
        started = true;
    }
// SENDING
    send_status(software_killswitch, file);
    send_pwm(pwm_values, file);


// RECEIVING
    temperature = readFloatsFromI2C(file);

    for (size_t i = 0; i < temperature.size(); i++)
    {
        std::cout << i << "temperature value = " << temperature[i] << std::endl;
    }
    
    hardware_status = read_hardware_statusFromI2C(file);
    cout << "hardware status = " << (uint16_t)(hardware_status) << endl;

    close(file);

    return 1;
}


//--------------------------FUNCTIONS--------------------------

//--------------------------INITIALISATION--------------------------

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

//--------------------------SENDING--------------------------
     
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


//--------------------------RECEIVING--------------------------

std::vector<float> readFloatsFromI2C(int file) {
    // Define byte length
    int num_floats = 6;
    int byte_length = num_floats * sizeof(float);
    std::vector<float> floats(num_floats);

    // Read a block of bytes from the I2C device
    std::vector<uint8_t> data(byte_length);
    if (read(file, data.data(), byte_length) != byte_length) {
        std::cerr << "Failed to read data from the I2C device" << std::endl;
        return floats; // Returns empty vector if reading fails
    }

    // Convert the byte array to a list of floats
    memcpy(floats.data(), data.data(), byte_length);
    return floats;
}

uint8_t read_hardware_statusFromI2C(int file){
    uint8_t hardware_status;
    if (read(file, &hardware_status, 1) != 1) {
        std::cerr << "Failed to read hardware status from the I2C device" << std::endl;
        exit(EXIT_FAILURE);
        }
    return hardware_status;
}



