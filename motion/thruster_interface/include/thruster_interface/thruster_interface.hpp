#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <linux/i2c-dev.h>
#include <map>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

class ThrusterInterface {
private:
  std::map<double, double> pwm_table;
  const int I2C_BUS = 1;
  const int I2C_ADDRESS = 0x21;
  const char *I2C_DEVICE = "/dev/i2c-1";

  std::vector<uint8_t> pwm_to_bytes(const std::vector<int> &pwm_values);

public:
  ThrusterInterface(std::string mapping_file);
  void publish_thrust_to_escs(std::vector<double> forces,
                              std::vector<int> pin_map,
                              std::vector<int> direction_map,
                              std::vector<int> pwm_offsets);
  float interpolate(float force);
};
