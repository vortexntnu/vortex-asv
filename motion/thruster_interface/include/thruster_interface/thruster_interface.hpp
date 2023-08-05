#ifndef THRUSTER_INTERFACE_HPP
#define THRUSTER_INTERFACE_HPP

#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <thread>
#include <chrono>


class ThrusterInterface {
private:
    std::map<double, double> pwm_table;
    const int I2C_BUS = 1;
    const int I2C_ADDRESS = 0x21;
    const char* I2C_DEVICE = "/dev/i2c-1";

    float interpolate(float force);
    std::vector<uint8_t> pwm_to_bytes(const std::vector<int> &pwm_values);

public:
    ThrusterInterface(std::string mapping_file);
    void publish_thrust_to_escs(std::vector<double> forces);
};

#endif // THRUSTER_INTERFACE_HPP
