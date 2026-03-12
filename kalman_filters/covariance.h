#pragma once
#include <Eigen/Dense>
#include <tuple>
#include <stdexcept>

// Template class for variable states and control inputs
template <int num_states, int num_controls, int num_measurements>

class CovarianceMatrices{

public:

    // Process noise covariance matrix Q_u
    Eigen::Matrix<double, num_controls,num_controls> process_noise_covariance(double vx_variance, double omega_z_variance){
        Eigen::Matrix<double, num_controls,num_controls> Q_u;
        Q_u << vx_variance, 0,
               0, omega_z_variance; // Assuming process noise is uncorrelated between velocity and angular velocity 
        if (Q_u.array().isNaN().any()) {
            throw std::runtime_error("Process noise covariance matrix contains NaN values. Check input parameters.");
        }
        return Q_u;
    }

    // Measurement noise covariance matrix R
    Eigen::Matrix<double, num_measurements, num_measurements> measurement_noise_covariance(double x_variance, double y_variance){
        Eigen::Matrix<double, num_measurements, num_measurements> R;
        R << x_variance, 0,
             0, y_variance; // Assuming measurement noise is uncorrelated between x and y measurements
        if (R.array().isNaN().any()) {
            throw std::runtime_error("Measurement noise covariance matrix contains NaN values. Check input parameters.");
        }
        return R;
    }
};