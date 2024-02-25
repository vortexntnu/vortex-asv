#include <linux/i2c-dev.h>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>
#include <cstring> // Include for memcpy
#include <fstream>
#include <sstream>
#include <map>
#include <cmath>


using namespace std;

std::map<float, float> pwm_table;
std::vector<float> temperature;
std::vector<uint16_t> pwm_values;

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
uint16_t interpolate(float force);
std::vector<uint16_t> interpolate_all(std::vector<float> &force_values);
