#include <thruster_interface_asv/thruster_interface_asv_driver_lib.hpp>

namespace thruster_interface_asv_driver_lib {
// Interpolation from forces to pwm functions (START) ----------
struct _ForcePWM {
  float force;
  float pwm;
};

// Local variables used inside the interpolation
std::vector<_ForcePWM> _datatableForForceAndPWM;

std::vector<_ForcePWM> _loadDataFromCSV(const std::string &filepath) {
  // Prepare the datastructure we will load our data in
  std::vector<_ForcePWM> data;

  // Try opening the file and if suceeded fill our datastructure with all the
  // values in the .CSV file
  std::ifstream file(filepath);
  if (file.is_open()) {
    // Variable for the currect line value in the .CSV file
    std::string line;

    // Skip the header line
    std::getline(file, line);

    // Read data line by line
    while (std::getline(file, line)) {
      // Temporarty variables to store data correctly ----------
      // Define temporary placeholders for variables we are extracting
      std::string tempVar;
      // Define data structure format we want our .CSV vlaues to be ordered in
      _ForcePWM formatOfForcePWMDatastructure;

      // Data manipulation ----------
      // csvLineSplit variable converts "line" variable to a char stream of data
      // that can further be used to split and filter out values we want
      std::istringstream csvLineSplit(line);
      // Extract Forces from "csvLineSplit" variable
      std::getline(csvLineSplit, tempVar, '\t');
      formatOfForcePWMDatastructure.force = std::stof(tempVar);
      // Convert grams into Newtons as we expect to get Forces in Newtons but
      // the .CSV file calculates forsces in grams
      formatOfForcePWMDatastructure.force =
          formatOfForcePWMDatastructure.force * (9.81 / 1000.0);
      // Extract PWM from "csvLineSplit" variable
      std::getline(csvLineSplit, tempVar, '\t');
      formatOfForcePWMDatastructure.pwm = std::stof(tempVar);

      // Push processed data with correct formating into the complete dataset
      // ----------
      data.push_back(formatOfForcePWMDatastructure);
    }

    file.close();
  } else {
    std::cerr << "ERROR: Unable to open file: " << filepath << std::endl;
    exit(1);
  }

  return data;
}

int16_t *_interpolate_force_to_pwm(float *forces) {
  int16_t *interpolatedPWMArray = new int16_t[4]{0, 0, 0, 0};

  // Interpolate ----------
  for (int8_t i = 0; i < 4; i++) {
    // Edge case, if the force is out of the bounds of your data table, handle
    // accordingly
    if (forces[i] <= _datatableForForceAndPWM.front().force) {
      interpolatedPWMArray[i] = static_cast<int16_t>(
          _datatableForForceAndPWM.front().pwm); // To small Force
    } else if (forces[i] >= _datatableForForceAndPWM.back().force) {
      interpolatedPWMArray[i] = static_cast<int16_t>(
          _datatableForForceAndPWM.back().pwm); // To big Force
    } else {
      // Set temporary variables for interpolating
      // Initialize with the first element
      _ForcePWM low = _datatableForForceAndPWM.front();
      _ForcePWM high;

      // Interpolate
      // Find the two points surrounding the given force
      // Run the loop until the force value we are givven is lower than the
      // current dataset value
      for (const _ForcePWM &CurrentForcePWMData : _datatableForForceAndPWM) {
        if (CurrentForcePWMData.force >= forces[i]) {
          high = CurrentForcePWMData;
          break;
        }
        low = CurrentForcePWMData;
      }

      // Linear interpolation formula: y = y0 + (y1 - y0) * ((x - x0) / (x1 -
      // x0))
      float interpolatedPWM =
          low.pwm + (high.pwm - low.pwm) *
                        ((forces[i] - low.force) / (high.force - low.force));

      // Save the interpolated value in the final array
      interpolatedPWMArray[i] = static_cast<int16_t>(interpolatedPWM);
    }
  }

  return interpolatedPWMArray;
}
// Interpolation from forces to pwm functions (STOP) ----------

// The most important function that sends PWM values through I2C to Arduino that
// then sends PWM signal further to the ESCs
const int8_t _I2C_BUS = 1;
const int16_t _I2C_ADDRESS = 0x21;
const char *_I2C_DEVICE = "/dev/i2c-1";
void _send_pwm_to_ESCs(int16_t *pwm) {
  // Variables ----------
  // 4 PWM values of 16-bits
  // Each byte is 8-bits
  // 4*16-bits = 64-bits => 8 bytes
  uint8_t dataSize = 8;
  uint8_t messageInBytesPWM[8];

  // Convert PWM int 16-bit to bytes ----------
  for (int8_t i = 0; i < 4; i++) {
    // Split the integer into most significant byte (MSB) and least significant
    // byte (LSB)
    uint8_t msb = (pwm[i] >> 8) & 0xFF;
    uint8_t lsb = pwm[i] & 0xFF;

    // Add the bytes to the finished array
    messageInBytesPWM[i * 2] = msb;
    messageInBytesPWM[i * 2 + 1] = lsb;
  }

  // Data sending ----------
  // Open I2C conection
  int fileI2C = open(_I2C_DEVICE, O_RDWR);

  // Error handling in case of edge cases with I2C
  if (fileI2C < 0) {
    std::cerr << "ERROR: Couldn't opening I2C device" << std::endl;
    exit(2);
  }
  if (ioctl(fileI2C, I2C_SLAVE, _I2C_ADDRESS) < 0) {
    std::cerr << "ERROR: Couldn't set I2C address" << std::endl;
    close(fileI2C); // Close I2C connection before exiting
    exit(3);
  }

  // Send the I2C message
  if (write(fileI2C, messageInBytesPWM, dataSize) != dataSize) {
    std::cerr << "ERROR: Couldn't send data, ignoring message..." << std::endl;
    exit(4);
  }

  // Close I2C connection
  close(fileI2C);
}

// Initial function to set everything up with thruster driver
// Set initial PWM limiting variables
int16_t _minPWM[4] = {1100, 1100, 1100, 1100};
int16_t _maxPWM[4] = {1900, 1900, 1900, 1900};

void init(const std::string &pathToCSVFile, int16_t *minPWM, int16_t *maxPWM) {
  // Load data from the .CSV file
  // We load it here instead of interpolation step as this will save time as we
  // only open and load all the data once, savind time in intrepolation isnce we
  // dont need to open the .CSV file over and over again.
  _datatableForForceAndPWM = _loadDataFromCSV(pathToCSVFile);

  // Set correct parameters for limiting PWM
  for (int8_t i = 0; i < 4; i++) {
    _minPWM[i] = minPWM[i];
    _maxPWM[i] = maxPWM[i];
  }
}

// The main core functionality of interacting and controling the thrusters
int16_t *drive_thrusters(float *forces) {
  // Interolate forces to raw PWM values
  int16_t *pwm = _interpolate_force_to_pwm(forces);

  // Limit PWM
  for (int8_t i = 0; i < 4; i++) {
    if (pwm[i] < _minPWM[i]) {
      pwm[i] = _minPWM[i]; // To small PWM
    } else if (pwm[i] > _maxPWM[i]) {
      pwm[i] = _maxPWM[i]; // To big PWM
    }
  }

  // Send PWM vlaues through I2C to the microcontroller to control thrusters
  _send_pwm_to_ESCs(pwm);

  // Return PWM values for debuging and logging purposes
  return pwm;
}
} // namespace thruster_interface_asv_driver_lib
