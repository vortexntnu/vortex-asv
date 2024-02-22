#include <cstring> // Include for memcpy
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

using namespace std;

void send_status(int8_t status, int file);
std::vector<uint8_t> pwm_to_bytes(std::vector<uint16_t> pwm_values);
void send_pwm(std::vector<uint16_t> pwm_values, int file);
void init(int &file);

std::vector<float> readFloatsFromI2C(int file);
uint8_t read_hardware_statusFromI2C(int file);