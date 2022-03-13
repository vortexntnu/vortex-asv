/**
 * @file
 * @brief A ROS wrapper layer for the quaternion PID controller
 *
 */
#ifndef VORTEX_CONTROLLER_CONTROLLER_ROS_H
#define VORTEX_CONTROLLER_CONTROLLER_ROS_H

#include <Eigen/Dense>

#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>

#include "dp_controller/control_modes.h"
#include "eigen_typedefs.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "vortex_msgs/ControlMode.h" //service
#include "vortex_msgs/Debug.h"
#include "vortex_msgs/PropulsionCommand.h"
#include "vortex_msgs/RovState.h"
#include <dp_controller/VortexControllerConfig.h>

#include "dp_controller/quaternion_pd_controller.h"


// Action server
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

// typedef so you dont have to write out definition every time
typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>
    MoveBaseActionServer;

/**
 * @brief the Controller class
 *
 * This class serves as a wrapper for the lower-level controller implementation
 * @see quaternion_pd_controller.h
 *
 */
class Controller {

public:
  /**
   * @brief Controller class constructor
   *
   * @param nh ROS nodehandle
   */
  explicit Controller(ros::NodeHandle nh);

  /**
   * @brief Callback for the state subscriber
   *
   * If the orientation given in @p msg is invalid, this function returns early.
   * Else it publishes the calculated feedback through the action server. It
   * also returns "success" if the setpoint is within the circle of acceptance.
   *
   * @param msg   A nav_msg::Odometry message containing state data about the
   * AUV.
   */
  void stateCallback(const nav_msgs::Odometry &msg);

  /**
   * @brief Callback for the dynamic reconfigure server
   *
   * @param config   A VortexControllerConfig object used to store parameters
   * @param level    Unused integer
   *
   * This function sets the controller gains for the @p config and passes these
   * to the @c setGains() command in the controller.
   */
  void configCallback(const dp_controller::VortexControllerConfig &config,
                      uint32_t level);

  /**
   * @brief Service server callback for setting control mode
   *
   * @param req   The requested control mode
   * @param res   The server respose to the @p req
   *
   * @return Always returns true if function execution makes it to
   * the end of the function
   */
  bool controlModeCallback(vortex_msgs::ControlMode::Request &req,
                           vortex_msgs::ControlMode::Response &res);

  /**
   * @brief Action server; goal
   *
   * Called when a new goal is set, and simply accepts the new goal.
   *
   */
  void actionGoalCallBack();

  /**
   * @brief Action server; preemptive goal
   *
   * Called whenever external applications like rviz sends a simple goal.
   */
  void preemptCallBack();

  /**
   * @brief class wrapper for the usual ros::spin() command
   *
   * for each spinOnce, this function gets the newest state
   * and newest setpoints, and calculates a force vector
   * depending on the current control mode.
   */
  void spin();

  ros::ServiceServer control_mode_service_;

private:
  ros::NodeHandle m_nh;

  ros::Subscriber m_command_sub;
  ros::Subscriber m_state_sub;

  ros::Publisher m_wrench_pub;
  ros::Publisher m_debug_pub;

  dynamic_reconfigure::Server<dp_controller::VortexControllerConfig>
      m_dr_srv; /** dynamic_reconfigure server */

  ControlMode m_control_mode; /** Current control mode                        */
  int m_frequency;            /** Update frequency for controller (ros rate)  */
  bool m_debug_mode = true; 
  const double c_normalized_force_deadzone = 0.01;
  const double c_max_quat_norm_deviation = 0.1;
  bool m_goal_reached;
  
  std::unique_ptr<QuaternionPdController> m_controller; 

  // EIGEN CONVERSION INITIALIZE
  Eigen::Vector3d position_state;                /** Current position      */
  Eigen::Quaterniond orientation_state;          /** Current orientation   */
  Eigen::Vector6d velocity_state;                /** Current velocity      */
  Eigen::Vector3d position_setpoint;       /** Position setpoint     */
  Eigen::Quaterniond orientation_setpoint; /** Orientation setpoint  */

  std::vector<double> tau_command_max;
  std::vector<double> tau_command_scaling;

  enum PoseIndex {
    SURGE = 0,
    SWAY = 1,
    HEAVE = 2,
    ROLL = 3,
    PITCH = 4,
    YAW = 5
  };
  enum EulerIndex { EULER_YAW = 0, EULER_PITCH = 1, EULER_ROLL = 2 };

  /**
   * @brief Convert integer to ControlMode
   *
   * @param mode  An integer that is to be converted to a ControlMode
   *
   * @return The ControlMode corresponding to the given @p mode integer
   * as defined in control_modes.h
   */
  ControlMode getControlMode(int mode);

  /**
   * @brief Initialize setpoints by loading and setting default wrench params
   */
  void initSetpoints();

  /**
   * @brief Reset setpoints by setting them equal to current state
   */
  void resetSetpoints();

  /**
   * @brief Update the current setpoint for a given @p axis
   *
   * @param axis  The selected axis for which the setpoint updates
   */
  void updateSetpoint(PoseIndex axis);

  bool isPositionInCOA();

  bool isYawInCOA();

  /**
   * @brief Publish a vortex_msgs Debug message containing current state and
   * setpoint data
   *
   * @param position_state          A 3d vector containing the current body
   * position
   * @param orientation_state       A quaternion containing the current
   * orientation
   * @param velocity_state          A 6d vector containing the current velocity
   *
   * @param position_setpoint       A 3d vector containing the position setpoint
   * @param orientation_setpoint    A quaternion containing the orientation
   * setpoint
   */
  void publishDebugMsg(const Eigen::Vector3d &position_state,
                       const Eigen::Quaterniond &orientation_state,
                       const Eigen::Vector6d &velocity_state,
                       const Eigen::Vector3d &position_setpoint,
                       const Eigen::Quaterniond &orientation_setpoint);

protected:
  MoveBaseActionServer *mActionServer; /** Action server object */

  ros::ServiceServer mControlModeService; /** Service server object */

  float R; /** Radius of the circle of acceptance */
};

#endif // VORTEX_CONTROLLER_CONTROLLER_ROS_H
