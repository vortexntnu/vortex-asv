#ifndef VORTEX_ALLOCATOR_ALLOCATOR_UTILS_HPP
#define VORTEX_ALLOCATOR_ALLOCATOR_UTILS_HPP

#include <eigen3/Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include <vortex_msgs/msg/thruster_forces.hpp>

/**
 * @file allocator_utils.hpp
 * @brief This file contains utility functions for the thruster allocator
 * module.
 */

/**
 * @brief Check if the matrix has any NaN or INF elements.
 *
 * @tparam Derived The type of the matrix.
 * @param M The matrix to check.
 * @return true if the matrix has any NaN or INF elements, false otherwise.
 */
template <typename Derived>
inline bool isInvalidMatrix(const Eigen::MatrixBase<Derived> &M) {
  bool has_nan = !(M.array() == M.array()).all();
  bool has_inf = M.array().isInf().any();
  return has_nan || has_inf;
}

/**
 * @brief Returns a string stream containing the matrix with the given name.
 *
 * @param name The name of the matrix.
 * @param M The matrix to print.
 * @return std::stringstream The string stream containing the matrix.
 */
inline std::stringstream printMatrix(std::string name,
                                     const Eigen::MatrixXd &M) {
  std::stringstream ss;
  ss << std::endl << name << " = " << std::endl << M;
  return ss;
}

/**
 * @brief Calculates the right pseudoinverse of the given matrix.
 *
 * @param M The matrix to calculate the pseudoinverse of.
 * @param M_pinv The resulting pseudoinverse matrix.
 * @throws char* if the pseudoinverse is invalid.
 */
inline void calculateRightPseudoinverse(const Eigen::MatrixXd &M,
                                        Eigen::MatrixXd &M_pinv) {
  Eigen::MatrixXd pseudoinverse = M.transpose() * (M * M.transpose()).inverse();
  // pseudoinverse.completeOrthogonalDecomposition().pseudoInverse();
  if (isInvalidMatrix(pseudoinverse)) {
    throw std::runtime_error("Invalid Psuedoinverse Calculated");
  }
  M_pinv = pseudoinverse;
}

/**
 * @brief Saturates the values of a given Eigen vector between a minimum and maximum value.
 * 
 * @param vec The Eigen vector to be saturated.
 * @param min The minimum value to saturate the vector values to.
 * @param max The maximum value to saturate the vector values to.
 * @return True if all vector values are within the given range, false otherwise.
 */
inline bool saturateVectorValues(Eigen::VectorXd &vec, double min, double max) {
  bool vector_in_range = true;
  for (int i = 0; i < vec.size(); ++i) {
    if ((vec)(i) > max) {
      (vec)(i) = max;
      vector_in_range = false;
    } else if ((vec)(i) < min) {
      (vec)(i) = min;
      vector_in_range = false;
    }
  }
  return vector_in_range;
}

/**
 * @brief Converts an Eigen VectorXd to a vortex_msgs::msg::ThrusterForces
 * message.
 *
 * @param u The Eigen VectorXd to be converted.
 * @param msg The vortex_msgs::msg::ThrusterForces message to store the
 * converted values.
 */
inline void arrayEigenToMsg(const Eigen::VectorXd &u,
                            vortex_msgs::msg::ThrusterForces &msg) {
  int r = u.size();
  std::vector<double> u_vec(r);
  for (int i = 0; i < r; ++i)
    u_vec[i] = u(i);
  msg.thrust = u_vec;
}

#endif // VORTEX_ALLOCATOR_ALLOCATOR_UTILS_HPP
