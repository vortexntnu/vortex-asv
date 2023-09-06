#include <boost/process.hpp>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sstream>
#include <string>

/**
 * Executes the jbdtool command to query the BMS connected to the specified
 * serial port.
 *
 * @param usb_port The USB port number to connect to (e.g., "ttyUSB0",
 * "ttyUSB1", etc.)
 * @param output The output string containing the command result
 */

void execute_jbdtool(const std::string &usb_port, std::string &output) {
  std::string command = "./jbdtool -t serial:/dev/" + usb_port;
  std::string working_directory = "/home/vortex/bms/jbdtool";

  boost::process::ipstream is;
  boost::process::child c(command, boost::process::std_out > is,
                          boost::process::start_dir = working_directory);

  std::string line;
  while (c.running() && std::getline(is, line)) {
    output += line + "\n";
  }

  c.wait();
}

/**
 * Parses the output from the BMS to extract information about Voltage, Current,
 * Temps, and Cells.
 *
 * @param response The response string from the BMS command execution
 * @param voltage The extracted voltage value
 * @param current The extracted current value
 * @param temps The extracted temperature values as a string
 * @param cells The extracted cell values as a string
 */

void parse_bms_data(const std::string &response, float &voltage, float &current,
                    std::string &temps, std::string &cells) {
  std::stringstream ss(response);

  std::string line;
  while (std::getline(ss, line)) {
    if (line.find("Voltage") != std::string::npos) {
      voltage = std::stof(line.substr(line.find("Voltage") + 8));
    } else if (line.find("Current") != std::string::npos) {
      current = std::stof(line.substr(line.find("Current") + 8));
    } else if (line.find("Temps") != std::string::npos) {
      temps = line.substr(line.find("Temps") + 6);
    } else if (line.find("Cells") != std::string::npos) {
      cells = line.substr(line.find("Cells") + 6);
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "bms");
  ros::NodeHandle nh;

  ros::Publisher battery_pub =
      nh.advertise<sensor_msgs::BatteryState>("/internal/status/bms", 10);

  std::vector<std::string> usb_ports = {"ttyUSB0", "ttyUSB1"}; //

  ros::Rate rate(1); // 1 Hz

  // Main loop for publishing BatteryState messages from BMS data
  while (ros::ok()) {
    for (const std::string &usb_port : usb_ports) {
      std::string response;
      execute_jbdtool(usb_port, response);

      if (!response.empty()) {
        float voltage, current;
        std::string temps, cells;
        parse_bms_data(response, voltage, current, temps, cells);

        sensor_msgs::BatteryState battery_msg;
        battery_msg.voltage = voltage;
        battery_msg.current = current;
        battery_msg.temperature = std::stof(temps.substr(0, temps.find(',')));
        battery_msg.location = usb_port;

        battery_pub.publish(battery_msg);
      }
    }

    rate.sleep();
  }

  return 0;
}
