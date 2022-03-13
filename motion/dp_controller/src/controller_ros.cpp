#include "dp_controller/controller_ros.h"
#include "dp_controller/eigen_helper.h"

#include "std_msgs/String.h"
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>

#include <map>
#include <math.h>
#include <string>
#include <vector>

Controller::Controller(ros::NodeHandle nh) : m_nh(nh), m_frequency(10) {
  // Subscribers
  m_state_sub =
      m_nh.subscribe("/pose_gt", 10, &Controller::stateCallback, this);

  // Publishers
  std::string thrust_topic;

  if (!m_nh.getParam("/controllers/dp/thrust_topic", thrust_topic)) {
    thrust_topic = "/thrust/desired_forces";
    ROS_WARN("Failed to read parameter thrust_topic, defaulting to "
             "/thrust/desired_forces");
  }

  m_wrench_pub = m_nh.advertise<geometry_msgs::Wrench>(thrust_topic, 1);
  m_debug_pub = m_nh.advertise<vortex_msgs::Debug>("debug/controlstates", 10);

  // Initial control mode and
  m_control_mode = ControlModes::OPEN_LOOP;

  // Launch file specifies <auv>.yaml as directory
  if (!m_nh.getParam("/controllers/dp/frequency", m_frequency))
    ROS_WARN(
        "Failed to read parameter controller frequency, defaulting to %i Hz.",
        m_frequency);

  if (!m_nh.getParam("/controllers/dp/circleOfAcceptance", R)) {
    ROS_WARN("Failed to read parameter circleOfAcceptance");
  }
  
  // Initialize the controller itself
  // Read controller gains from parameter server
  double a, b, c, i;
  if (!m_nh.getParam("/controllers/dp/velocity_gain", a))
    ROS_ERROR("Failed to read parameter velocity_gain.");
  if (!m_nh.getParam("/controllers/dp/position_gain", b))
    ROS_ERROR("Failed to read parameter position_gain.");
  if (!m_nh.getParam("/controllers/dp/attitude_gain", c))
    ROS_ERROR("Failed to read parameter attitude_gain.");
  if (!m_nh.getParam("/controllers/dp/integral_gain", i))
    ROS_ERROR("Failed to read parameter integral_gain.");

  // Read center of gravity and buoyancy vectors from <auv>.yaml
  std::vector<double> r_G_vec, r_B_vec;
  if (!m_nh.getParam("/physical/center_of_mass", r_G_vec))
    ROS_FATAL("Failed to read robot center of mass parameter.");
  if (!m_nh.getParam("/physical/center_of_buoyancy", r_B_vec))
    ROS_FATAL("Failed to read robot center of buoyancy parameter.");
  Eigen::Vector3d r_G(r_G_vec.data());
  Eigen::Vector3d r_B(r_B_vec.data());

  // Read and calculate ROV weight and buoyancy from <auv>.yaml
  double mass, displacement, acceleration_of_gravity, density_of_water;
  if (!m_nh.getParam("/physical/mass_kg", mass))
    ROS_FATAL("Failed to read parameter mass.");
  if (!m_nh.getParam("/physical/displacement_m3", displacement))
    ROS_FATAL("Failed to read parameter displacement.");
  if (!m_nh.getParam("/gravity/acceleration", acceleration_of_gravity))
    ROS_FATAL("Failed to read parameter acceleration of gravity");
  if (!m_nh.getParam("/water/density", density_of_water))
    ROS_FATAL("Failed to read parameter density of water");
  double W = mass * acceleration_of_gravity;
  double B = density_of_water * displacement * acceleration_of_gravity;

  m_controller.reset(new QuaternionPdController(a, b, c, i, W, B, r_G, r_B));
  m_goal_reached = false;

  // Set up a dynamic reconfigure server
  dynamic_reconfigure::Server<
      dp_controller::VortexControllerConfig>::CallbackType dr_cb;
  dr_cb = boost::bind(&Controller::configCallback, this, _1, _2);
  m_dr_srv.setCallback(dr_cb);

  ROS_INFO("Initialized at %i Hz.", m_frequency);

  /* Action server */
  // ros::NodeHandle nodeHandle("move_base");
  mActionServer =
      new MoveBaseActionServer(m_nh, "move_base", /*autostart*/ false);

  // register the goal and feeback callbacks
  mActionServer->registerGoalCallback(
      boost::bind(&Controller::actionGoalCallBack, this));
  mActionServer->registerPreemptCallback(
      boost::bind(&Controller::preemptCallBack, this));
  mActionServer->start();
  ROS_INFO("Started action server.");
  mControlModeService = m_nh.advertiseService(
      "control_mode_service", &Controller::controlModeCallback, this);
  ROS_INFO("Started service server.");


  std::vector<double> v;

  if (!m_nh.getParam("/propulsion/command/wrench/max", v)) {
    ROS_FATAL("Failed to read parameter max wrench command.");
    tau_command_max = v;
  }
  if (!m_nh.getParam("/propulsion/command/wrench/scaling", v)) {
    
    ROS_FATAL("Failed to read parameter scaling wrench command.");
    tau_command_scaling = v;
  }
}

void Controller::spin() {

  ros::Rate rate(m_frequency);

  while (ros::ok()) {

    Eigen::Vector6d tau_command = Eigen::VectorXd::Zero(6);

    if (m_debug_mode) {
      publishDebugMsg(position_state, orientation_state, velocity_state,
                      position_setpoint, orientation_setpoint);
    }

    switch (m_control_mode) {
    case ControlModes::OPEN_LOOP: {
      tau_command(SURGE) = 0;
      tau_command(SWAY) = 0;
      tau_command(YAW) = 0;
      break;
    }

    case ControlModes::POSITION_HOLD: {
      tau_command = m_controller->getFeedback(
          position_state, Eigen::Quaterniond::Identity(), velocity_state,
          position_setpoint, Eigen::Quaterniond::Identity());
      tau_command(YAW) = 0;

      if (!m_goal_reached && isPositionInCOA()) {
        ROS_INFO("Reached setpoint!");
        m_goal_reached = true;
      }
      break;
    }

    case ControlModes::HEADING_HOLD: {
      tau_command = m_controller->getFeedback(
          Eigen::Vector3d::Zero(), orientation_state, velocity_state,
          Eigen::Vector3d::Zero(), orientation_setpoint);
      tau_command(SURGE) = 0;
      tau_command(SWAY) = 0;

      if (!m_goal_reached && isYawInCOA()) {
        ROS_INFO("Reached setpoint!");
        m_goal_reached = true;
      }

      break;
    }

    case ControlModes::POSE_HOLD: {
      tau_command = m_controller->getFeedback(position_state, orientation_state,
                                              velocity_state, position_setpoint,
                                              orientation_setpoint);
      if (!m_goal_reached && isYawInCOA() && isPositionInCOA()) {
        ROS_INFO("Reached setpoint!");
        m_goal_reached = true;
      }
      break;
    }

    default: {
      ROS_ERROR("Default control mode reached.");
    }
    }

    tau_command(HEAVE) = 0;
    tau_command(ROLL) = 0;
    tau_command(PITCH) = 0;

    geometry_msgs::Wrench tau_msg;
    tf::wrenchEigenToMsg(tau_command, tau_msg);
    m_wrench_pub.publish(tau_msg);

    ros::spinOnce();
    rate.sleep();
  }
}

bool Controller::isPositionInCOA() {
  return m_controller->circleOfAcceptanceXY(position_state, position_setpoint, R);
}

bool Controller::isYawInCOA() {
  return m_controller->circleOfAcceptanceYaw(orientation_state, orientation_setpoint, 0.05);
}


/* SERVICE SERVER */
bool Controller::controlModeCallback(vortex_msgs::ControlMode::Request &req,
                                     vortex_msgs::ControlMode::Response &res) {

  ControlMode new_control_mode = static_cast<ControlMode>(req.controlMode);
  if (new_control_mode != m_control_mode) {
    m_control_mode = new_control_mode;
    ROS_INFO_STREAM("Changing mode to " << controlModeString(m_control_mode)
                                        << ".");
  }
  res.result = "success";
  return true;
}

/* ACTION SERVER */
void Controller::preemptCallBack() {

  // notify the ActionServer that we've successfully preempted
  ROS_DEBUG_NAMED("move_base", "Move base preempting the current goal");

  // set the action state to preempted
  mActionServer->setPreempted();
}

void Controller::actionGoalCallBack() {

  // set current target position to previous position
  m_controller->x_d_prev = position_state;
  m_controller->x_d_prev_prev = position_state;
  m_controller->x_ref_prev = position_state;
  m_controller->x_ref_prev_prev = position_state;

  // Integral action reset
  m_controller->integral = Eigen::Vector6d::Zero();

  // accept the new goal - do I have to cancel a pre-existing one first?
  geometry_msgs::PoseStamped new_goal = mActionServer->acceptNewGoal()->target_pose;

  // Transform from Msg to Eigen
  tf::pointMsgToEigen(new_goal.pose.position, position_setpoint);
  tf::quaternionMsgToEigen(new_goal.pose.orientation, orientation_setpoint);

  Eigen::Vector3d euler =
      orientation_setpoint.toRotationMatrix().eulerAngles(2, 1, 0);
  ROS_INFO("Controller::actionGoalCallBack(): driving to %2.2f/%2.2f/%2.2f",
           position_setpoint[0], position_setpoint[1], 180.0 / M_PI * euler[0]);
}

/* DYNAMIC RECONFIGURE */

void Controller::configCallback(
    const dp_controller::VortexControllerConfig &config, uint32_t level) {
  ROS_INFO("DP controller reconfigure:");
  ROS_INFO("\t velocity_gain: %2.4f", config.velocity_gain);
  ROS_INFO("\t position_gain: %2.4f", config.position_gain);
  ROS_INFO("\t attitude_gain: %2.4f", config.attitude_gain);
  ROS_INFO("\t integral_gain: %2.4f", config.integral_gain);

  m_controller->setGains(config.velocity_gain, config.position_gain,
                         config.attitude_gain, config.integral_gain);
}

/* SUBSCRIBER CALLBACKS */
void Controller::stateCallback(const nav_msgs::Odometry &msg) {

  // Convert to eigen for computation
  tf::pointMsgToEigen(msg.pose.pose.position, position_state);
  tf::quaternionMsgToEigen(msg.pose.pose.orientation, orientation_state);
  tf::twistMsgToEigen(msg.twist.twist, velocity_state);

  bool orientation_invalid =
      (abs(orientation_state.norm() - 1) > c_max_quat_norm_deviation);
  if (isFucked(position_state) || isFucked(velocity_state) || orientation_invalid) {
    ROS_WARN_THROTTLE(1, "Invalid state estimate received, ignoring...");
    return;
  }

  // geometry_msgs/PoseStamped Pose
  if (!mActionServer->isActive())
    return;

  // return a feedback message to the client
  move_base_msgs::MoveBaseFeedback feedback;
  feedback.base_position.header.stamp = ros::Time::now();
  feedback.base_position.pose = msg.pose.pose;
  mActionServer->publishFeedback(feedback);

  // if within circle of acceptance, return result succeeded
  if (m_goal_reached) {
    mActionServer->setSucceeded(move_base_msgs::MoveBaseResult(),
                                "Goal reached.");
    m_goal_reached = false;
  }
}

/* PUBLISH HELPERS */
void Controller::publishDebugMsg(
    const Eigen::Vector3d &position_state,
    const Eigen::Quaterniond &orientation_state,
    const Eigen::Vector6d &velocity_state,
    const Eigen::Vector3d &position_setpoint,
    const Eigen::Quaterniond &orientation_setpoint) {
  vortex_msgs::Debug dbg_msg;

  // Estimated position
  dbg_msg.state_position.x = position_state[0];
  dbg_msg.state_position.y = position_state[1];
  dbg_msg.state_position.z = position_state[2];

  // Estimated linear velocity
  dbg_msg.state_velocity.linear.x = velocity_state[0];
  dbg_msg.state_velocity.linear.y = velocity_state[1];
  dbg_msg.state_velocity.linear.z = velocity_state[2];

  // Estimated angular velocity
  dbg_msg.state_velocity.angular.x = velocity_state[3];
  dbg_msg.state_velocity.angular.y = velocity_state[4];
  dbg_msg.state_velocity.angular.z = velocity_state[5];

  // Setpoint position
  dbg_msg.setpoint_position.x = position_setpoint[0];
  dbg_msg.setpoint_position.y = position_setpoint[1];
  dbg_msg.setpoint_position.z = position_setpoint[2];

  // Debub state euler orientation
  Eigen::Vector3d dbg_state_orientation =
      orientation_state.toRotationMatrix().eulerAngles(2, 1, 0);
  Eigen::Vector3d dbg_setpoint_orientation =
      orientation_setpoint.toRotationMatrix().eulerAngles(2, 1, 0);

  // Debug state orientation
  dbg_msg.state_yaw = dbg_state_orientation[0];
  dbg_msg.state_pitch = dbg_state_orientation[1];
  dbg_msg.state_roll = dbg_state_orientation[2];

  // Debug setpoint euler orientation
  dbg_msg.setpoint_yaw = dbg_setpoint_orientation[0];
  dbg_msg.setpoint_pitch = dbg_setpoint_orientation[1];
  dbg_msg.setpoint_roll = dbg_setpoint_orientation[2];

  /*
  // tau body
  Eigen::Vector6d tau_body = m_controller->getFeedback(position_state,
  Eigen::Quaterniond::Identity(), velocity_state, position_setpoint,
  Eigen::Quaterniond::Identity());

  dbg_msg.tau_xb = tau_body[0];
  dbg_msg.tau_yb = tau_body[1];
  dbg_msg.tau_zb = tau_body[2]; */

  // publish
  m_debug_pub.publish(dbg_msg);
}
