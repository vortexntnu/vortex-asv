// Custom libraies
#include <thruster_interface_asv/thruster_interface_asv_node.hpp>

int main() {
  // Get MAX and MIN PWM values
  int16_t minPWM[4] = {1100, 1100, 1100, 1100};
  int16_t maxPWM[4] = {1900, 1900, 1900, 1900};

  // Get filepath of .CSV file
  std::string forcesToPWMDataFilePath =
      ament_index_cpp::get_package_share_directory("thruster_interface_asv");
  forcesToPWMDataFilePath += "/config/ThrustMe_P1000_force_mapping.csv";

  // Initialize Thruster driver
  thruster_interface_asv_driver_lib::init(forcesToPWMDataFilePath, minPWM,
                                          maxPWM);

  // Test (START) --------------------------------------------------
  float test_input[4];
  int16_t *test_output;

  // Test 1 ----------
  test_input[0] = 0.0;
  test_input[1] = 0.0;
  test_input[2] = 0.0;
  test_input[3] = 0.0;
  test_output = thruster_interface_asv_driver_lib::drive_thrusters(test_input);
  std::cout << "Test 1: ";
  std::cout << test_output[0] << ", ";
  std::cout << test_output[1] << ", ";
  std::cout << test_output[2] << ", ";
  std::cout << test_output[3] << std::endl;
  sleep(10);

  // Test 2 ----------
  test_input[0] = 100.0;
  test_input[1] = 100.0;
  test_input[2] = 100.0;
  test_input[3] = 100.0;
  test_output = thruster_interface_asv_driver_lib::drive_thrusters(test_input);
  std::cout << "Test 2: ";
  std::cout << test_output[0] << ", ";
  std::cout << test_output[1] << ", ";
  std::cout << test_output[2] << ", ";
  std::cout << test_output[3] << std::endl;
  sleep(3);

  // Test 3 ----------
  test_input[0] = 0.0;
  test_input[1] = 0.0;
  test_input[2] = 0.0;
  test_input[3] = 0.0;
  test_output = thruster_interface_asv_driver_lib::drive_thrusters(test_input);
  std::cout << "Test 3: ";
  std::cout << test_output[0] << ", ";
  std::cout << test_output[1] << ", ";
  std::cout << test_output[2] << ", ";
  std::cout << test_output[3] << std::endl;

  delete[] test_output;
  // Test (STOP) --------------------------------------------------
}