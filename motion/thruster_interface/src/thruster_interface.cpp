#include <thruster_interface/thruster_interface.hpp>

ThrusterInterface::ThrusterInterface(std::string mapping_file) {
  // Open the data file
  std::ifstream data(mapping_file);

  if (!data.is_open()) {
    std::cerr << "Unable to open file\n";
    exit(1);
  }

  std::string line;
  // Ignore the header line
  std::getline(data, line);

  while (std::getline(data, line)) {
    std::istringstream stream(line);

    std::string force_str, pwm_str;

    std::getline(stream, force_str, '\t');
    std::getline(stream, pwm_str);

    double force = std::stod(force_str);
    double pwm = std::stod(pwm_str);

    pwm_table[force] = pwm;
  }
}

float ThrusterInterface::interpolate(float force) {
  auto upper_bound = pwm_table.upper_bound(force);
  if (upper_bound == pwm_table.end()) {
    --upper_bound;
  }
  auto lower_bound = std::prev(upper_bound);

  if (lower_bound == pwm_table.end()) {
    return upper_bound->second;
  }

  float force1 = lower_bound->first;
  float force2 = upper_bound->first;

  float pwm1 = lower_bound->second;
  float pwm2 = upper_bound->second;

  if (force1 == force2) {
    return pwm1;
  }

  int pwm_signal = std::round(
      pwm1 + ((force - force1) * (pwm2 - pwm1)) / (force2 - force1) + 0.5);

  return pwm_signal;
}

std::vector<uint8_t>
ThrusterInterface::pwm_to_bytes(const std::vector<int> &pwm_values) {

  std::vector<uint8_t> bytes;
  for (const auto &val : pwm_values) {
    // Ensure the value is in the correct range and cast to an integer
    int pwm_int = static_cast<int>(
        std::min(std::max(val, 1300), 1700)); // min 1100, max 1900

    // Split the integer into most significant byte (MSB) and least significant
    // byte (LSB)
    uint8_t msb = (pwm_int >> 8) & 0xFF;
    uint8_t lsb = pwm_int & 0xFF;

    // Add MSB and LSB to the bytes vector
    bytes.push_back(msb);
    bytes.push_back(lsb);
  }
  return bytes;
}

void ThrusterInterface::publish_thrust_to_escs(std::vector<double> forces, std::vector<int> thruster_to_pin_map, std::vector<int> direction_map, std::vector<int> pwm_offsets) {
  std::vector<int> pwm_values;

  int center_pwm_value = 1500;

  // Use correct direction and correct pwm offsets 
  for (int i = 0; i < 4; i++) {
    int true_center_value = center_pwm_value + pwm_offsets[i];

    int pwm_value_correct_direction =
        true_center_value + direction_map[i] * interpolate(forces[i]);

    int pwm_clamped = std::min(std::max(pwm_value_correct_direction, 1300),
                               1700); // min 1100, max 1900
    
    pwm_values.push_back(pwm_clamped);
  }

  // Remap the pwm values to the correct pins
  std::vector<int> remapped_pwm_values(int(thruster_to_pin_map.size()));
  for (int i = 0; i < int(thruster_to_pin_map.size()); i++)
    remapped_pwm_values[thruster_to_pin_map[i]] = pwm_values[i];

  pwm_values = remapped_pwm_values;

  std::vector<uint8_t> pwm_bytes = pwm_to_bytes(pwm_values);
  int data_size = pwm_bytes.size();

  // DEBUG
  // for (auto pwm : pwm_values) {
  //    std::cout << pwm << " ";
  //}
  // std::cout << std::endl;

  int file = open(I2C_DEVICE, O_RDWR);
  if (file < 0) {
    std::cerr << "Error opening device\n";
    exit(1);
  }

  if (ioctl(file, I2C_SLAVE, I2C_ADDRESS) < 0) {
    std::cerr << "Error setting I2C address\n";
    exit(1);
  }

  // Send the I2C message
  if (write(file, pwm_bytes.data(), data_size) != data_size) {
    std::cerr << "Error sending data, ignoring message...\n";
  }

  close(file);
}
