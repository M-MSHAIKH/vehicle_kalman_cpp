#pragma once
#include <Eigen/Dense>
#include <tuple>
#include <stdexcept>
#include "vehicle_model/kinematic_bicycle.h"

// Template class for variable states and control inputs
template <int num_states, int num_controls, int num_measurements>

class JacobianKinematicBicycle{

public:

    double dt; //Time step
    double l; //Wheelbase

    // Constructor to initialize dt and l
    JacobianKinematicBicycle(double dt, double l) : dt(dt), l(l) {}

    // Jacobian of state transition function with respect to state variables
    Eigen::Matrix<double, num_states, num_states> F_jacobian(const Eigen::Matrix<double,num_states,1>& jacobian_var) {
        double beta = jacobian_var(0);
        double psi = jacobian_var(1);
        double vx = jacobian_var(2);
        Eigen::Matrix<double, num_states, num_states> F ; // Initialize the Jacobian matrix
        F << 1, 0, -vx * sin(psi + beta) * dt,
             0, 1,  vx * cos(psi + beta) * dt,
             0, 0, 1; // Fill in the Jacobian matrix based on the kinematic bicycle model

        if (F.array().isNaN().any()) {
            throw std::runtime_error("Jacobian contains NaN values. Check input parameters.");
        }
        return F;

    }

    // Jacobian of state transition function with respect to control inputs
    Eigen::Matrix<double,num_states, num_controls> L_jacobian(const Eigen::Matrix<double,num_states,1>& jacobian_var) {
        double beta = jacobian_var(0);
        double psi = jacobian_var(1);
        double vx = jacobian_var(2);
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