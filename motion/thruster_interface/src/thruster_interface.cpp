#include <thruster_interface/thruster_interface.hpp>

const int I2C_ADDRESS = 0x21;
const char *I2C_DEVICE = "/dev/i2c-1";
int8_t i2c_slave_addr = 0x21;

bool started = false;

int8_t hardware_status = 0;

#ifndef PWM_TABLE_HPP
#define PWM_TABLE_HPP

std::map<float, float> pwm_table;
std::vector<float> temperature;
std::vector<uint16_t> pwm_values;

#endif

/**
int main() {

    std::vector<float> forces_values = {110000,5000,-7700,-1300000.15}; //change
later has to come from somewhere uint8_t software_killswitch = 0; //0 if
everything goes well and 1 if there is a problem ; change later has to come from
somewhere

    pwm_values.resize(4);
    pwm_values = interpolate_all(forces_values);

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
**/

//--------------------------FUNCTIONS--------------------------

//--------------------------INITIALISATION--------------------------

void init(int &file) {
  if ((file = open(I2C_DEVICE, O_RDWR)) < 0) {
    std::cerr << "error, could not connect to i2c bus" << std::endl;
    close(file);
    exit(EXIT_FAILURE);
  } else {
    std::cout << "connected to i2c bus!!" << std::endl;
  }
  if (ioctl(file, I2C_SLAVE, i2c_slave_addr) < 0) {
    std::cerr << "error, could not set adress" << std::endl;
    close(file);
    exit(EXIT_FAILURE);
  } else {
    std::cout << "i2c adress set" << std::endl;
  }
}

//--------------------------SENDING--------------------------

void send_status(int8_t status, int file) {
  if (write(file, &status, 1) != 1) {
    std::cerr << "error, could not send status" << std::endl;
    close(file);
    // exit(EXIT_FAILURE);
  } else {
    std::cout << "status data has been sent" << std::endl;
  }
}

void send_pwm(std::vector<uint16_t> pwm_values, int file) {
  std::vector<uint8_t> bytes = pwm_to_bytes(pwm_values);
  // if(write(file, bytes.data(), bytes.size()) != bytes.size()){
  if (static_cast<size_t>(write(file, bytes.data(), bytes.size())) !=
      bytes.size()) {
    std::cerr << "error, could not send PWM data" << std::endl;
    close(file);
    exit(EXIT_FAILURE);

  } else {
    std::cout << "PWM data has been sent" << std::endl;
  }
}

std::vector<uint8_t> pwm_to_bytes(std::vector<uint16_t> pwm_values) {

  std::vector<uint8_t> bytes;

  for (size_t i = 0; i < pwm_values.size(); ++i) {
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

uint8_t read_hardware_statusFromI2C(int file) {
  uint8_t hardware_status;
  if (read(file, &hardware_status, 1) != 1) {
    std::cerr << "Failed to read hardware status from the I2C device"
              << std::endl;
    exit(EXIT_FAILURE);
  }
  return hardware_status;
}

//--------------------------INTERPOLATION--------------------------

void get_pwm_table() {
  // Open the data file
  std::ifstream file("src/vortex-asv/motion/thruster_interface/config/"
                     "ThrustMe_P1000_force_mapping.csv");
  if (!file.is_open()) {
    std::cerr << "Unable to open PWM table\n";
    exit(1);
  }

  std::string line;
  // Ignore the header line
  std::getline(file, line);
  // read line by line
  while (std::getline(file, line)) {
    std::istringstream stream(line);
    std::string force_str, pwm_str;
    std::getline(stream, force_str, '\t'); // read the line until a tab is found
    std::getline(stream, pwm_str);         // read the rest of the line
    // convert into float
    float force = std::stod(force_str);
    float pwm = std::stod(pwm_str);
    // add values to the the table
    pwm_table[force] = pwm;
  }
  file.close();
}

uint16_t interpolate(float force) {

  if (pwm_table.empty()) {
    get_pwm_table();
  }

  auto it = pwm_table.lower_bound(
      force); // Find the first element with a key not less than force

  // If the force is less than or equal to the smallest force in the table
  if (it == pwm_table.begin())
    return it->second;

  // If the force is greater than or equal to the largest force in the table
  if (it == pwm_table.end()) {
    --it;
    return it->second;
  }

  // Linear interpolation
  auto prev = std::prev(it); // Get the element with the next smaller key
  double force1 = prev->first;
  double force2 = it->first;
  double pwm1 = prev->second;
  double pwm2 = it->second;

  return pwm1 + (pwm2 - pwm1) * (force - force1) / (force2 - force1);
}

std::vector<uint16_t> interpolate_all(std::vector<float> &force_values) {
  std::vector<uint16_t> interpolatedVector;
  // Interpolate each value in the input vector
  for (const auto &force : force_values) {
    interpolatedVector.push_back(interpolate(force));
  }

  return interpolatedVector;
}