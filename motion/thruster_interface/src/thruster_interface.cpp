#include <thruster_interface/thruster_interface.hpp>

// const int I2C_ADDRESS = 0x21;
const char *I2C_DEVICE = "/dev/i2c-1";
int8_t i2c_slave_addr = 0x40; // change later to 0x21
std::map<float, float> pwm_table;

//--------------------------FUNCTIONS--------------------------

//--------------------------INITIALISATION--------------------------

void init(int &file) {
  if ((file = open(I2C_DEVICE, O_RDWR)) < 0) {
    close(file);
    throw I2C_Exception("error, could not connect to i2c bus");
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
    close(file);
    throw I2C_Exception("error, could not send status");
  } else {
    std::cout << "status data has been sent" << std::endl;
  }
}

void send_pwm(std::vector<uint16_t> pwm_values, int file) {
  std::vector<uint8_t> bytes = pwm_to_bytes(pwm_values);
  if (static_cast<size_t>(write(file, bytes.data(), bytes.size())) !=
      bytes.size()) {
    throw I2C_Exception("error, could not send PWM data");
    close(file);
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
    throw I2C_Exception("Failed to read data from the I2C device");
    return floats; // Returns empty vector if reading fails
  }
  // Convert the byte array to a list of floats
  memcpy(floats.data(), data.data(), byte_length);
  return floats;
}

uint8_t read_hardware_statusFromI2C(int file) {
  uint8_t hardware_status;
  if (read(file, &hardware_status, 1) != 1) {
    throw I2C_Exception("Failed to read hardware status from the I2C device");
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

uint16_t interpolate(double force, int PWM_min, int PWM_max) {
  uint16_t pwm;
  if (pwm_table.empty()) {
    get_pwm_table();
  }
  // Find the first element with a key not less than force
  auto it = pwm_table.lower_bound(force);
  if (it == pwm_table.begin()) { // If the force is less than or equal to the
                                 // smallest force in the table
    pwm = it->second;
  } else if (it == pwm_table.end()) { // If the force is greater than or equal
                                      // to the largest force in the table
    --it;
    pwm = it->second;
  } else {
    // Linear interpolation
    auto prev = std::prev(it); // Get the element with the next smaller key
    double force1 = prev->first;
    double force2 = it->first;
    double pwm1 = prev->second;
    double pwm2 = it->second;
    pwm = pwm1 + (pwm2 - pwm1) * (force - force1) / (force2 - force1);
  }
  // check if the calculated pwm values are within the defined limits [PWM_min,
  // PWM_max].
  if (pwm < PWM_min) {
    return PWM_min;
  } else if (pwm > PWM_max) {
    return PWM_max;
  } else {
    return pwm;
  }
}

std::vector<uint16_t> interpolate_all(std::vector<double> &force_values,
                                      int PWM_min, int PWM_max) {
  std::vector<uint16_t> interpolatedVector;
  // Interpolate each value in the input vector
  for (const auto &force : force_values) {
    interpolatedVector.push_back(interpolate(force, PWM_min, PWM_max));
  }

  return interpolatedVector;
}
