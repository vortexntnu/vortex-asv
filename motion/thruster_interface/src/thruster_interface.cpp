#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <sstream>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <cmath>

#include <vector>

float interpolate(const std::map<double, double>& pwm_table, float force) {
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

    return std::round(pwm1 + ((force - force1) * (pwm2 - pwm1)) / (force2 - force1) + 0.5);
}

std::vector<uint8_t> pwm_to_bytes(const std::vector<int>& pwm_values) {
    std::vector<uint8_t> bytes;
    for (const auto& val : pwm_values) {
        // Ensure the value is in the correct range and cast to an integer
        int pwm_int = static_cast<int>(std::min(std::max(val, 1100), 1900));

        // Split the integer into most significant byte (MSB) and least significant byte (LSB)
        uint8_t msb = (pwm_int >> 8) & 0xFF;
        uint8_t lsb = pwm_int & 0xFF;

        // Add MSB and LSB to the bytes vector
        bytes.push_back(msb);
        bytes.push_back(lsb);
    }
    return bytes;
}



int main() {
    // Create an empty map
    std::map<double, double> pwm_table;
    
    // Open the data file
    std::ifstream data("../config/ThrustMe_P1000_force_mapping.csv");
    if (!data.is_open()) {
        std::cerr << "Unable to open file\n";
        return 1;
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
    
    const int I2C_BUS = 1;
    const int I2C_ADDRESS = 0x21;

    int file = open("/dev/i2c-1", O_RDWR);
    if (file < 0) {
        std::cerr << "Error opening device\n";
        return 1;
    }

    if (ioctl(file, I2C_SLAVE, I2C_ADDRESS) < 0) {
        std::cerr << "Error setting I2C address\n";
        return 1;
    }

    float desired_force_thr_1 = 9000;
    float desired_force_thr_2 = 9000;
    float desired_force_thr_3 = 9000;
    float desired_force_thr_4 = 9000;

    int pwm_thr_1 = interpolate(pwm_table, desired_force_thr_1);
    int pwm_thr_2 = interpolate(pwm_table, desired_force_thr_2);
    int pwm_thr_3 = interpolate(pwm_table, desired_force_thr_3);
    int pwm_thr_4 = interpolate(pwm_table, desired_force_thr_4);

    std::vector<int> pwm_values = {pwm_thr_1, pwm_thr_2, pwm_thr_3, pwm_thr_4};
    std::vector<uint8_t> pwm_bytes = pwm_to_bytes(pwm_values);
    int data_size = pwm_bytes.size();


    // Send the I2C message
    if (write(file, pwm_bytes.data(), data_size) != data_size) {
        std::cerr << "Error sending data\n";
        return 1;
    }

    close(file);


    return 0;
}