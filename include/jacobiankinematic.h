#pragma once
#include <Eigen/Dense>
#include <tuple>
#include <stdexcept>
#include <kinematic_bicycle.h>

// Template class for variable states and control inputs
template <int num_states, int num_controls, int num_measurements>

class JacobianKinematicBicycle{

public:

    double dt; //Time step

    // Constructor to initialize dt and l
    JacobianKinematicBicycle(double dt) : dt(dt) {}

    // Jacobian of state transition function with respect to state variables F = df/dx
    Eigen::Matrix<double, num_states, num_states> F_jacobian(const Eigen::Matrix<double, num_states, 1>& jacobian_var) {

        // Check the dimensions of the input Jacobian variable matrix
        if (jacobian_var.rows() != num_states || jacobian_var.cols() != 1) {
            throw std::invalid_argument("Jacobian variable input in the JACOBIANKINEMATICBICYCLE must be of size (num_states, 1) and has a size of (" 
                + std::to_string(jacobian_var.rows()) + ", " 
                + std::to_string(jacobian_var.cols()) + ")");
        }

        double beta = jacobian_var(0, 0);
        double psi = jacobian_var(1, 0);
        double vx = jacobian_var(2, 0);
        Eigen::Matrix<double, num_states, num_states> F ; // Initialize the Jacobian matrix
        F << 1, 0, -vx * sin(psi + beta) * dt,
             0, 1,  vx * cos(psi + beta) * dt,
             0, 0, 1; // Fill in the Jacobian matrix based on the kinematic bicycle model

        if (F.array().isNaN().any()) {
            throw std::runtime_error("Jacobian contains NaN values. Check input parameters.");
        }
        return F;

    }

    // Jacobian of state transition function with respect to control inputs L = df/du
    Eigen::Matrix<double,num_states, num_controls> L_jacobian(const Eigen::Matrix<double, num_states, 1>& jacobian_var) {
        double beta = jacobian_var(0, 0);
        double psi = jacobian_var(1, 0);
        double vx = jacobian_var(2, 0);
        Eigen::Matrix<double, num_states, num_controls> L; 
        L<< cos(beta + psi) * dt, 0,
            sin(beta + psi) * dt, 0,
            0, dt; 

        if (L.array().isNaN().any()) {
            throw std::runtime_error("Jacobian contains NaN values. Check input parameters.");
        }
        return L;
    }

    // Measurement Jacobian H = dh/dx
    Eigen::Matrix<double, num_measurements, num_states> H_jacobian() {
        Eigen::Matrix<double, num_measurements, num_states> H; 
        H << 1, 0, 0,
             0, 1, 0; // Assuming we are measuring x and y directly

        if (H.array().isNaN().any()) {
            throw std::runtime_error("Jacobian contains NaN values. Check input parameters.");
        }
        return H;
    }

    // Measurement Jacobian M = dh/du
    Eigen::Matrix<double, num_measurements, num_measurements> M_jacobian(){
        Eigen::Matrix<double, num_measurements, num_measurements> M;
        M << 1, 0,
             0, 1; // Assuming measurement noise is directly added to the measurements

        if (M.array().isNaN().any()) {
            throw std::runtime_error("Jacobian contains NaN values. Check input parameters.");
        }
        return M;
    }
};