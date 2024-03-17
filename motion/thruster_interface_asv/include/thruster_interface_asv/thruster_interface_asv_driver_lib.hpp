// C++ Libraries
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdint.h>
#include <string>
#include <vector>

#include <fcntl.h>         // Used for the O_RDWR define
#include <linux/i2c-dev.h> // Used for the I2C_SLAVE define
#include <sys/ioctl.h>
#include <unistd.h> // Used for the close function

// Declare functions we can use
namespace thruster_interface_asv_driver_lib {
void init(const std::string &pathToCSVFile, int16_t *minPWM, int16_t *maxPWM);
int16_t *drive_thrusters(float *forces);
} // namespace thruster_interface_asv_driver_lib