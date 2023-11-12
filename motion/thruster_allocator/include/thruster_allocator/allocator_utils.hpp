/**
 * @file allocator_utils.hpp
 * @brief This file contains utility functions for the thruster allocator
 * module.
 */

#ifndef VORTEX_ALLOCATOR_ALLOCATOR_UTILS_HPP
#define VORTEX_ALLOCATOR_ALLOCATOR_UTILS_HPP

#include <eigen3/Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include <vortex_msgs/msg/thruster_forces.hpp>

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
 * @throws char* if the pseudoinverse is invalid.
 * @return The pseudoinverse of the given matrix.
 */
inline Eigen::MatrixXd calculateRightPseudoinverse(const Eigen::MatrixXd &T) {
  Eigen::MatrixXd pseudoinverse = T.transpose() * (T * T.transpose()).inverse();
  // pseudoinverse.completeOrthogonalDecomposition().pseudoInverse();
  if (isInvalidMatrix(pseudoinverse)) {
    throw std::runtime_error("Invalid Psuedoinverse Calculated");
  }
  return pseudoinverse;
}

/**
 * @brief Saturates the values of a given Eigen vector between a minimum and
 * maximum value.
 *
 * @param vec The Eigen vector to be saturated.
 * @param min The minimum value to saturate the vector values to.
 * @param max The maximum value to saturate the vector values to.
 * @return True if all vector values are within the given range, false
 * otherwise.
 */
inline bool saturateVectorValues(Eigen::VectorXd &vec, double min, double max) {
  bool all_values_in_range =
      std::all_of(vec.begin(), vec.end(),
                  [min, max](double val) { return val >= min && val <= max; });

  std::transform(vec.begin(), vec.end(), vec.begin(), [min, max](double val) {
    return std::min(std::max(val, min), max);
  });

  return all_values_in_range;
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
  std::copy_n(u.begin(), r, u_vec.begin());
  msg.thrust = u_vec;
}

/**
 * @brief Converts a 1D array of doubles to a 2D Eigen matrix.
 *
 * @param matrix The 1D array of doubles to be converted.
 * @param rows The number of rows in the resulting Eigen matrix.
 * @param cols The number of columns in the resulting Eigen matrix.
 * @return The resulting Eigen matrix.
 */
inline Eigen::MatrixXd
doubleArrayToEigenMatrix(const std::vector<double> &matrix, int rows,
                         int cols) {
  return Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                        Eigen::RowMajor>>(matrix.data(), rows,
                                                          cols);
}

#endif // VORTEX_ALLOCATOR_ALLOCATOR_UTILS_HPP
