#include <thruster_interface_asv/thruster_interface_asv_driver_lib.hpp>

namespace thruster_interface_asv_driver_lib {
// Interpolation from forces to pwm functions (START) ----------
struct _ForcePWM {
  float force;
  float pwm;
};

// Local variables used inside the interpolation
std::vector<_ForcePWM> _ForcePWMTable;

std::vector<_ForcePWM> _loadDataFromCSV(const std::string &filepath) {
  // Prepare the datastructure we will load our data in
  std::vector<_ForcePWM> data;

  // Try opening the file and if suceeded fill our datastructure with all the
  // values from the force mapping .CSV file
  std::ifstream file(filepath);
  if (file.is_open()) {
    std::string line;

    // Skip the header line
    std::getline(file, line);

    // Read data line by line
    while (std::getline(file, line)) {
      // Temporarty variables to store data correctly ----------
      // Define temporary placeholders for variables we are extracting
      std::string tempVar;

      // Define data structure format we want our .CSV vlaues to be ordered in
      _ForcePWM ForcePWMDataStructure;

      // Data manipulation ----------
      // csvLineSplit variable converts "line" variable to a char stream of data
      // that can further be used to split and filter out values we want
      std::istringstream csvLineSplit(line);

      // Extract Forces from "csvLineSplit" variable
      std::getline(csvLineSplit, tempVar, '\t');
      ForcePWMDataStructure.force = std::stof(tempVar);

      // Convert grams into Newtons as we expect to get Forces in Newtons but
      // the .CSV file contains forces in grams
      ForcePWMDataStructure.force =
          ForcePWMDataStructure.force * (9.81 / 1000.0);

      // Extract PWM from "csvLineSplit" variable
      std::getline(csvLineSplit, tempVar, '\t');
      ForcePWMDataStructure.pwm = std::stof(tempVar);

      // Push processed data with correct formating into the complete dataset
      // ----------
      data.push_back(ForcePWMDataStructure);
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
    if (forces[i] <= _ForcePWMTable.front().force) {
      interpolatedPWMArray[i] =
          static_cast<int16_t>(_ForcePWMTable.front().pwm); // Too small Force

    } else if (forces[i] >= _ForcePWMTable.back().force) {
      interpolatedPWMArray[i] =
          static_cast<int16_t>(_ForcePWMTable.back().pwm); // Too big Force

    } else {
      // Set temporary variables for interpolating
      // Initialize with the first element
      _ForcePWM low = _ForcePWMTable.front();
      _ForcePWM high;

      // Interpolate
      // Find the two points surrounding the given force
      // Run the loop until the force value we are givven is lower than the
      // current dataset value
      for (const _ForcePWM &CurrentForcePWMData : _ForcePWMTable) {
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
  }

  // Close I2C connection
  close(fileI2C);
}

// Initial function to set everything up with thruster driver
// Set initial PWM limiting variables
int8_t _thrusterMapping[4];
int8_t _thrusterDirection[4];
int16_t _offsetPWM[4];
int16_t _minPWM[4];
int16_t _maxPWM[4];

void init(const std::string &pathToCSVFile, int8_t *thrusterMapping,
          int8_t *thrusterDirection, int16_t *offsetPWM, int16_t *minPWM,
          int16_t *maxPWM) {
  // Load data from the .CSV file
  _ForcePWMTable = _loadDataFromCSV(pathToCSVFile);

  // Set correct parameters
  for (int8_t i = 0; i < 4; i++) {
    _thrusterMapping[i] = thrusterMapping[i];
    _thrusterDirection[i] = thrusterDirection[i];
    _offsetPWM[i] = offsetPWM[i];
    _minPWM[i] = minPWM[i];
    _maxPWM[i] = maxPWM[i];
  }
}

// The main core functionality of interacting and controling the thrusters
int16_t *drive_thrusters(float *thrusterForces) {
  // Remap Thrusters
  // From: [pin1:thruster1, pin2:thruster2, pin3:thruster3, pin4:thruster4] 
  // To:   [pin1:<specifiedThruster>, pin2:<specifiedThruster>, pin3:<specifiedThruster>, pin4:<specifiedThruster>]
  float thrusterForcesChangedMapping[4] = {0.0, 0.0, 0.0, 0.0};
  for (int8_t pinNr = 0; pinNr < 4; pinNr++) {
    int8_t remapedThrusterForcesIndex = _thrusterMapping[pinNr];
    thrusterForcesChangedMapping[pinNr] =
        thrusterForces[remapedThrusterForcesIndex];
  }

  // Change direction of the thruster (Forward(+1)/Backwards(-1)) according to
  // the direction parameter
  float thrusterForcesChangedDirection[4] = {0.0, 0.0, 0.0, 0.0};
  for (int8_t i = 0; i < 4; i++) {
    thrusterForcesChangedDirection[i] =
        thrusterForcesChangedMapping[i] * _thrusterDirection[i];
  }

  // Interpolate forces to raw PWM values
  int16_t *pwm = _interpolate_force_to_pwm(thrusterForcesChangedDirection);

  // Offset PWM
  for (int8_t i = 0; i < 4; i++) {
    pwm[i] += _offsetPWM[i];
  }

  // Limit PWM
  for (int8_t i = 0; i < 4; i++) {
    pwm[i] = std::clamp(pwm[i], _minPWM[i], _maxPWM[i]);
  }

  // Send PWM vlaues through I2C to the microcontroller to control thrusters
  _send_pwm_to_ESCs(pwm);

  // Return PWM values for debuging and logging purposes
  return pwm;
}
} // namespace thruster_interface_asv_driver_lib
