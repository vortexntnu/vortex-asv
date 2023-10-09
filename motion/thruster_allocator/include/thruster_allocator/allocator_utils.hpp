#ifndef VORTEX_ALLOCATOR_ALLOCATOR_UTILS_HPP
#define VORTEX_ALLOCATOR_ALLOCATOR_UTILS_HPP

#include "rclcpp/rclcpp.hpp"
#include "vortex_msgs/msg/thruster_forces.hpp"
#include <eigen3/Eigen/Eigen>
#include <string>
#include <vector>

// Return true if X has any nan or inf elements.
template <typename Derived>
inline bool isInvalidMatrix(const Eigen::MatrixBase<Derived> &X) {
  bool has_nan = !(X.array() == X.array()).all();
  bool has_inf = !((X - X).array() == (X - X).array()).all();
  return has_nan || has_inf;
}

inline void printMatrix(std::string name, const Eigen::MatrixXd &X) {
  std::stringstream ss;
  ss << std::endl << name << " = " << std::endl << X;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), ss.str());
}

inline void printVector(std::string name, const Eigen::VectorXd &X) {
  std::stringstream ss;
  ss << std::endl << name << " = " << std::endl << X;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), ss.str());
}

// Calculate the pseudoinverse matrix of the matrix X.
// Return false if the calculations fails.
inline bool calculatePseudoinverse(const Eigen::MatrixXd &X,
                                   Eigen::MatrixXd *X_pinv) {
  Eigen::MatrixXd pseudoinverse = X.transpose() * (X * X.transpose()).inverse();
  
  if (isInvalidMatrix(pseudoinverse)) {
    return false;
  }
  *X_pinv = pseudoinverse;
  return true;
}

//
// TODO: Trengs disse funksjonene under?
//

// Read a matrix from the ROS parameter server.
// Return false if unsuccessful.
// Burde ikke ligge her!!
// inline bool getMatrixParam(ros::NodeHandle nh, std::string name,
//                            Eigen::MatrixXd *X) {
//   XmlRpc::XmlRpcValue matrix;
//   nh.getParam(name, matrix);

//   try {
//     const int rows = matrix.size();
//     const int cols = matrix[0].size();
//     X->setZero(rows, cols);
//     for (int i = 0; i < rows; ++i)
//       for (int j = 0; j < cols; ++j)
//         (*X)(i, j) = matrix[i][j];
//   } catch (...) {
//     return false;
//   }
//   return true;
// }

// Return the 3-by-3 skew-symmetric matrix of the vector v.
inline Eigen::Matrix3d createSkewSymmetricMatrix(const Eigen::Vector3d &v) {
  Eigen::Matrix3d S;
  S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return S;
}

inline void arrayEigenToMsg(const Eigen::VectorXd &u,
                            vortex_msgs::msg::ThrusterForces *msg) {
  int r = u.size();
  std::vector<double> u_vec(r);
  for (int i = 0; i < r; ++i)
    u_vec[i] = u(i);
  msg->thrust = u_vec;
}

// Saturate all elements of vector v to within [min, max].
// Return true if all elements already are within the range.
inline bool clampVectorValues(Eigen::VectorXd *v, double min, double max) {
  bool vector_in_range = true;
  for (int i = 0; i < v->size(); ++i) {
    if ((*v)(i) > max) {
      (*v)(i) = max;
      vector_in_range = false;
    } else if ((*v)(i) < min) {
      (*v)(i) = min;
      vector_in_range = false;
    }
  }
  return vector_in_range;
}

#endif // VORTEX_ALLOCATOR_ALLOCATOR_UTILS_HPP
