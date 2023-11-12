#include <gtest/gtest.h>

#include "thruster_allocator/pseudoinverse_allocator.hpp"

class PseudoinverseTest : public ::testing::Test {};

TEST_F(PseudoinverseTest, ZeroPinvThenZeroThrust) {
  constexpr int size = 3;
  Eigen::MatrixXd T = Eigen::MatrixXd::Zero(size, size);
  PseudoinverseAllocator allocator(T);

  Eigen::VectorXd input_thrust = Eigen::VectorXd::Constant(size, 10.0);
  Eigen::VectorXd calculated_thrust =
      allocator.calculateAllocatedThrust(input_thrust);

  Eigen::VectorXd expected_thrust = Eigen::VectorXd::Zero(size);

  EXPECT_EQ(calculated_thrust, expected_thrust);
}

TEST_F(PseudoinverseTest, IdentityPinvThenSameThrust) {
  constexpr int size = 3;
  Eigen::MatrixXd T = Eigen::MatrixXd::Identity(size, size);
  PseudoinverseAllocator allocator(T);

  Eigen::VectorXd input_thrust = Eigen::VectorXd::Constant(size, 10.0);
  Eigen::VectorXd calculated_thrust =
      allocator.calculateAllocatedThrust(input_thrust);

  EXPECT_EQ(calculated_thrust, input_thrust);
}

TEST_F(PseudoinverseTest, SomePinvThenSameThrust) {
  constexpr int rows = 3;
  constexpr int cols = 4;

  Eigen::MatrixXd T(rows, cols);
  T << 0.70711, 0.70711, 0.70711, 0.70711, -0.70711, 0.70711, -0.70711, 0.70711,
      0.27738, 0.27738, -0.27738, -0.27738;

  Eigen::MatrixXd T_pinv = T.transpose() * (T * T.transpose()).inverse();

  PseudoinverseAllocator allocator(T_pinv);

  Eigen::VectorXd input_thrust(rows);
  input_thrust << 10.0, 0.0, 0.0;

  Eigen::VectorXd allocated_thrust =
      allocator.calculateAllocatedThrust(input_thrust);

  Eigen::VectorXd expected_thrust(cols);
  expected_thrust << 3.53552, 3.53552, 3.53552, 3.53552;
  constexpr double tolerance = 1e-6;

  EXPECT_TRUE(allocated_thrust.isApprox(expected_thrust, tolerance));
}
