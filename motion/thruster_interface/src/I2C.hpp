#include <linux/i2c-dev.h>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>
#include <cstring> // Include for memcpy

using namespace std;


void send_status(int8_t status, int file);
std::vector<uint8_t> pwm_to_bytes(std::vector<uint16_t> pwm_values);
void send_pwm(std::vector<uint16_t> pwm_values, int file);
void init(int &file);
void send_communication_choice(int8_t choice, int file);

std::vector<float> readFloatsFromI2C(int file);