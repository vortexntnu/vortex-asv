
#include <cmath>
#include <cstring> // Include for memcpy
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <linux/i2c-dev.h>
#include <map>
#include <sstream>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>
using namespace std;

#ifndef PWM_TABLE_HPP
#define PWM_TABLE_HPP
extern const int I2C_ADDRESS;
extern const char *I2C_DEVICE;
extern int8_t i2c_slave_addr;
extern bool started;
extern int8_t hardware_status;
extern uint8_t software_killswitch;
extern std::map<float, float> pwm_table;
#endif

//--------------------------INITIALISATION--------------------------
void init(int &file);

//--------------------------SENDING--------------------------
void send_status(int8_t status, int file);
std::vector<uint8_t> pwm_to_bytes(std::vector<uint16_t> pwm_values);
void send_pwm(std::vector<uint16_t> pwm_values, int file);

//--------------------------RECEIVING--------------------------
std::vector<float> readFloatsFromI2C(int file);
uint8_t read_hardware_statusFromI2C(int file);

//--------------------------INTERPOLATION--------------------------
void get_pwm_table();
uint16_t interpolate(double force);
std::vector<uint16_t> interpolate_all(std::vector<double> &force_values);
