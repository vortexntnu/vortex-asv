#ifndef VORTEX_ALLOCATOR_ALLOCATOR_UTILS_HPP
#define VORTEX_ALLOCATOR_ALLOCATOR_UTILS_HPP

#include "rclcpp/rclcpp.hpp"
#include "vortex_msgs/msg/thruster_forces.hpp"
#include <eigen3/Eigen/Eigen>
#include <string>
#include <vector>

// Return true if M has any NaN or INF elements.
template <typename Derived>
inline bool isInvalidMatrix(const Eigen::MatrixBase<Derived> &M) {
  bool has_nan = !(M.array() == M.array()).all();
  bool has_inf = M.array().isInf().any();
  return has_nan || has_inf;
}

inline void printMatrix(std::string name, const Eigen::MatrixXd &M) {
  std::stringstream ss;
  ss << std::endl << name << " = " << std::endl << M;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), ss.str());
}

inline void printVector(std::string name, const Eigen::VectorXd &M) {
  std::stringstream ss;
  ss << std::endl << name << " = " << std::endl << M;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), ss.str());
}

// Calculate the pseudoinverse matrix of the matrix M.
// Return false if the calculations fails.
inline bool calculatePseudoinverse(const Eigen::MatrixXd &M,
                                   Eigen::MatrixXd *M_pinv) {
  Eigen::MatrixXd pseudoinverse = M.transpose() * (M * M.transpose()).inverse();

  if (isInvalidMatrix(pseudoinverse)) {
    return false;
  }
  *M_pinv = pseudoinverse;
  return true;
}

// Saturate all elements of vector v to within [min, max].
// Return true if all elements already are within the range.
inline bool saturateVectorValues(Eigen::VectorXd *vec, double min, double max) {
  bool vector_in_range = true;
  for (int i = 0; i < vec->size(); ++i) {
    if ((*vec)(i) > max) {
      (*vec)(i) = max;
      vector_in_range = false;
    } else if ((*vec)(i) < min) {
      (*vec)(i) = min;
      vector_in_range = false;
    }
  }
  return vector_in_range;
}

// Copies vector elements into ThrusterForces message
inline void arrayEigenToMsg(const Eigen::VectorXd &u,
                            vortex_msgs::msg::ThrusterForces *msg) {
  int r = u.size();
  std::vector<double> u_vec(r);
  for (int i = 0; i < r; ++i)
    u_vec[i] = u(i);
  msg->thrust = u_vec;
}

// Return the 3-by-3 skew-symmetric matrix of the vector v.
// CURRENTLY UNUSED
inline Eigen::Matrix3d createSkewSymmetricMatrix(const Eigen::Vector3d &v) {
  Eigen::Matrix3d S;
  S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return S;
}

#endif // VORTEX_ALLOCATOR_ALLOCATOR_UTILS_HPP
