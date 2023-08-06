// lqr_controller.hpp
#include <iostream>
#include <Eigen/Dense>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;

class LQRController {
    Eigen::Matrix3d M;
    Eigen::Matrix3d D;
    Eigen::Matrix<double, 8, 8> Q;
    Eigen::Matrix3d R;
    Vector6d setpoint;
    double actuator_limits;

    int yaw_idx;
    Eigen::Vector2d integral_states;

public:
    LQRController(Eigen::Matrix3d M, Eigen::Matrix3d D, Eigen::VectorXd Q, Eigen::Vector3d R, Vector6d setpoint, double actuator_limits);

    Eigen::Vector3d control(Vector6d state, double dt);

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> linearize(Vector8d state, double dt);

    Eigen::Matrix<double, 3, 8> lqr(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::Matrix<double, 8, 8> Q, Eigen::Matrix3d R);
};

// // lqr_controller.cpp
// #include "lqr_controller.hpp"

double ssa(double angle) {
    return fmod((angle + M_PI), (2 * M_PI)) - M_PI;
}

LQRController::LQRController(Eigen::Matrix3d M, Eigen::Matrix3d D, Eigen::VectorXd Q, Eigen::Vector3d R, Vector6d setpoint, double actuator_limits) {
    this->Q = Q.asDiagonal();
    this->R = R.asDiagonal();
    this->setpoint = setpoint;
    this->M = M;
    this->D = D;

    this->yaw_idx = 2;
    this->integral_states = Eigen::Vector2d::Zero();

    this->actuator_limits = actuator_limits;
}

Eigen::Vector3d LQRController::control(Vector6d state, double dt) {
    Vector8d state_with_integral;
    state_with_integral << state, integral_states;

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> linearized = linearize(state_with_integral, dt);
    Eigen::MatrixXd A = linearized.first;
    Eigen::MatrixXd B = linearized.second;
    Eigen::Matrix<double, 3, 8> K = lqr(A, B, Q, R);

    Vector8d error = state_with_integral;
    error.segment<6>(0) -= setpoint;  // Subtract 'setpoint' from the first 6 elements of 'error'
    error.segment<2>(6) -= Eigen::Vector2d(0, 0);  // Subtract [0, 0] from the last 2 elements of 'error'

    error[yaw_idx] = ssa(error[yaw_idx]);

    Eigen::Vector3d tau_body = -K * error;

    if (tau_body.cwiseAbs().maxCoeff() > actuator_limits) {
        return tau_body;
    }

    integral_states += (state.head(2) - setpoint.head(2)) * dt;

    if ((state.head(2) - setpoint.head(2)).cwiseSign() != integral_states.cwiseSign()) {
        integral_states = Eigen::Vector2d::Zero();
    }

    return tau_body;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> LQRController::linearize(Vector8d state, double dt) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(8, 8);
    Eigen::Matrix3d Minv = M.inverse();
    A.block<3, 3>(3, 3) -= dt * Minv * D;
    A.block<3, 3>(0, 3) = dt * (Eigen::Matrix3d() <<
        std::cos(state[yaw_idx]), -std::sin(state[yaw_idx]), 0,
        std::sin(state[yaw_idx]), std::cos(state[yaw_idx]), 0,
        0, 0, 1).finished();

    A(6, 0) = dt;  // add integral effect on x
    A(7, 1) = dt;  // add integral effect on y

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(8, 3);
    B.block<3, 3>(3, 0) = dt * Minv;

    return {A, B};
}

Eigen::MatrixXd dareSolver(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R) {
    Eigen::MatrixXd X = Q;
    Eigen::MatrixXd X_prev;

    // Set a threshold for convergence of the algorithm
    double threshold = 1e-9;

    // Maximum number of iterations to prevent endless loop
    int max_iterations = 1000;
    int i;

    // The Riccati equation solver
    for (i = 0; i < max_iterations; i++) {
        X_prev = X;
        X = A.transpose() * X_prev * A - A.transpose() * X_prev * B * (R + B.transpose() * X_prev * B).inverse() * B.transpose() * X_prev * A + Q;

        // Check for convergence
        if ((X - X_prev).norm() < threshold) {
            break;
        }
    }

    // If the maximum number of iterations is reached, print a warning
    if (i == max_iterations) {
        std::cout << "Warning: Maximum number of iterations reached in DARE solver." << std::endl;
    }

    return X;
}

Eigen::Matrix<double, 3, 8> LQRController::lqr(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::Matrix<double, 8, 8> Q, Eigen::Matrix3d R) {
    Eigen::MatrixXd X = dareSolver(A, B, Q, R);
    Eigen::MatrixXd K = (B.transpose() * X * B + R).inverse() * B.transpose() * X * A;
    
    return K;
}

int main() {
    Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
    Eigen::VectorXd Q(8);
    Q << 0.1, 0.1,0.1,0.1,0.1,0.1,0.1,0.1;
    Eigen::Vector3d R;
    R << 1, 2, 3;
    Vector6d setpoint;
    setpoint << 1, 2, 3, 4, 5, 6;
    double actuator_limits = 100.0;

    LQRController controller(M, D, Q, R, setpoint, actuator_limits);
    
    Vector6d state;
    state << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
    double dt = 0.1;

    int num_steps = 100;
    for (int i = 0; i < num_steps; i++) {
        Eigen::Vector3d control_output = controller.control(state, dt);

        // Assume the control output is a force. Divide by mass to get acceleration.
        // Assume mass = 1 for simplicity
        double mass = 1.0;
        Eigen::Vector3d acceleration = control_output / mass;

        // Update state based on acceleration
        state.segment<3>(0) += dt * state.segment<3>(3);  // Update position based on velocity
        state.segment<3>(3) += dt * acceleration;  // Update velocity based on acceleration

        std::cout << "Time: " << i * dt << ", state:\n" << state << "setpoint:\n" << setpoint <<"\n\n";
    }

    return 0;
}