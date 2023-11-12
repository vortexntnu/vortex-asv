#include <gtest/gtest.h>

#include "thruster_allocator/allocator_utils.hpp"

class AllocatorUtilsTest : public ::testing::Test {
protected:
};

TEST_F(AllocatorUtilsTest, IsInvalidMatrix) {
    Eigen::MatrixXd valid_matrix = Eigen::MatrixXd::Identity(3, 3);
    ASSERT_FALSE(isInvalidMatrix(valid_matrix));
    
    Eigen::MatrixXd invalid_matrix(3, 3);
    invalid_matrix << 1, std::nan(""), 3, 4, 5, 6, 7, 8, 9;
    ASSERT_TRUE(isInvalidMatrix(invalid_matrix));
}

TEST_F(AllocatorUtilsTest, CalculateRightPseudoinverseNoThrow) {
    Eigen::MatrixXd T = Eigen::MatrixXd::Random(3, 3);
    Eigen::MatrixXd pinv;
    ASSERT_NO_THROW(calculateRightPseudoinverse(T, pinv));
}

TEST_F(AllocatorUtilsTest, CalculateRightPseudoinverseIdentity) {
    constexpr int size = 3;
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(size, size);
    Eigen::MatrixXd pinv;
    calculateRightPseudoinverse(T, pinv);

    constexpr double tolerance = 1e-6;
    EXPECT_TRUE(pinv.isApprox(T, tolerance));
}

TEST_F(AllocatorUtilsTest, CalculateRightPseudoinverseSomeMatrix) {
    constexpr int rows = 3;
    constexpr int cols = 4;

    Eigen::MatrixXd T(rows, cols);
    T << 0.70711, 0.70711,  0.70711,  0.70711, 
        -0.70711, 0.70711, -0.70711,  0.70711, 
         0.27738, 0.27738, -0.27738, -0.27738;

    Eigen::MatrixXd T_pinv;
    calculateRightPseudoinverse(T, T_pinv);

    Eigen::MatrixXd T_expected(cols, rows);
    T_expected << 0.353552, -0.353552,  0.901291, 
                  0.353552,  0.353552,  0.901291,
                  0.353552, -0.353552, -0.901291,
                  0.353552,  0.353552, -0.901291;

    constexpr double tolerance = 1e-6;
    EXPECT_TRUE(T_pinv.isApprox(T_expected, tolerance));
}

TEST_F(AllocatorUtilsTest, SaturateVectorValues) {
    Eigen::VectorXd vec(3);
    vec << -1, 5, 0;
    ASSERT_FALSE(saturateVectorValues(vec, 0, 1));

    Eigen::VectorXd expected_output(3);
    expected_output << 0, 1, 0; 
    constexpr double tolerance = 1e-6;

    ASSERT_TRUE(vec.isApprox(expected_output, tolerance));
}

TEST_F(AllocatorUtilsTest, ArrayEigenToMsg) {
    Eigen::VectorXd u = Eigen::VectorXd::Random(3);
    vortex_msgs::msg::ThrusterForces msg;
    arrayEigenToMsg(u, msg);

    constexpr double tolerance = 1e-6;
    bool isEqual = std::equal(u.begin(), u.end(), msg.thrust.begin(), 
                              [](double elem1, double elem2) {
                                  return std::abs(elem1 - elem2) < tolerance; 
                              });

    ASSERT_TRUE(isEqual);
}

TEST_F(AllocatorUtilsTest, DoubleArrayToEigenMatrix) {
    std::vector<double> array = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    constexpr int rows = 2;
    constexpr int cols = 3;
    Eigen::MatrixXd T = doubleArrayToEigenMatrix(array, rows, cols);
    ASSERT_EQ(T.rows(), rows);
    ASSERT_EQ(T.cols(), cols);

    Eigen::MatrixXd T_expected(rows, cols);
    T_expected << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

    constexpr double tolerance = 1e-6;
    ASSERT_TRUE(T.isApprox(T_expected, tolerance));

}