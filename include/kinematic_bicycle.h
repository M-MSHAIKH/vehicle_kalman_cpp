#pragma once
#include <Eigen/Dense>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <stdexcept> 

/*

File Info:
This file contains the implementation of the Kinematic Bicycle model for state estimation.
*/

// Template class for variable states and control inputs
template <int num_states, int num_controls>


class KinematicBicycle {
public:
    double dt;
    double l;
    
    KinematicBicycle(double dt, double l):
        dt(dt), l(l) {}

    std::tuple<Eigen::Matrix<double,num_states, 1>,Eigen::Matrix<double, num_states, 1>> state_transition(Eigen::Matrix<double, num_states, 1> input_state, 
                                                                                                            Eigen::Matrix<double, num_controls, 1> input_control){
        
        // Check the dimensions of the input state and control matrices
        if (input_state.rows() != num_states || input_state.cols() != 1) {
            throw std::invalid_argument("State input in the KINEMATICBICYCLE must be of size (num_states, 1) and has a size of (" 
                + std::to_string(input_state.rows()) + ", " 
                + std::to_string(input_state.cols()) + ")");
        }
        if (input_control.rows() != num_controls || input_control.cols() != 1) {
            throw std::invalid_argument("Control input in the KINEMATICBICYCLE must be of size (num_controls, 1) and has a size of (" 
                + std::to_string(input_control.rows()) + ", " 
                + std::to_string(input_control.cols()) + ")");
        }

        double x = input_state(0, 0);    // state vector [x, y, psi]
        double y = input_state(1, 0);
        double psi = input_state(2, 0);
        double vx = input_control(0, 0);   // control input vector [vx, omega_z]
        double omega_z = input_control(1, 0);

        double vx_safe = std::max(vx, 0.003); // Ensure velocity is slightly greater than zero to avoid division by zero
        double max_arg = M_PI / 2;
        double min_arg = -M_PI / 2;
        double actual_arg = (omega_z * l) / vx_safe; // Calculate the argument for atan2

        double arg; // Declare the variable to hold the clipped argument value
        // Clip the argument to the valid range for atan2 to prevent NaN values
        if (actual_arg >= max_arg) {
            arg = max_arg; // Clip the argument to the maximum value
        } else if (actual_arg <= min_arg) {
            arg = min_arg; // Clip the argument to the minimum value
        } else {
            arg = actual_arg; // Use the actual argument if it's within the valid range
        }
        
        // double arg = std::clamp(clip_val, min_arg, max_arg); // Clip the argument to the valid range for atan2
        double beta = std::atan(arg); // Calculate the slip angle beta

        // Calculate the predicted state
        Eigen::Matrix<double, num_states, 1> state_pred; // [x, y, psi]
        state_pred << x + (vx * cos(psi + beta) * dt),
                      y + (vx * sin(psi + beta) * dt),
                      psi + (omega_z * dt); // Assuming constant angular velocity

        // Calculate the jaciobian variable 
        // << insertion operator 
        Eigen::Matrix<double, num_states, 1> jacobian_var;
        jacobian_var << beta,
                psi,
                vx; // Partial derivative of state with respect to omega_z

        // Check for NaN values in the predicted state
        if (std::isnan(state_pred(0)) || std::isnan(state_pred(1)) || std::isnan(state_pred(2))) {
            throw std::runtime_error("Predicted state contains NaN values. Check input parameters.");
        }

        if (std::isnan(jacobian_var(0)) || std::isnan(jacobian_var(1)) || std::isnan(jacobian_var(2))) {
            throw std::runtime_error("Jacobian contains NaN values. Check input parameters.");
        }

    return std::make_tuple(state_pred, jacobian_var); // Return the predicted state and Jacobian as a tuple
    }
};

/*
NOTE:
- Change the jacobian variable when the number of states or the model is changed.
*/
