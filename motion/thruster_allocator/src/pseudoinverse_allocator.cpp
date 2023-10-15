#include <thruster_allocator/pseudoinverse_allocator.hpp>

/**
 * @brief Constructor for the PseudoinverseAllocator class.
 * 
 * @param T_pinv The pseudoinverse of the thruster configuration matrix.
 */
PseudoinverseAllocator::PseudoinverseAllocator(const Eigen::MatrixXd &T_pinv)
    : T_pinv(T_pinv) {}

/**
 * @brief Calculates the allocated thrust given the input torques using the pseudoinverse allocator.
 * 
 * @param tau The input torques as a vector.
 * @return The allocated thrust as a vector.
 */
Eigen::VectorXd
PseudoinverseAllocator::calculateAllocatedThrust(const Eigen::VectorXd &tau) {
  Eigen::VectorXd u = T_pinv * tau;
  return u;
}