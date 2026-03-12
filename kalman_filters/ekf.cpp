// // #include <tuple>
// // #include <Eigen/Dense>
// // #include <iostream>
// // #include <stdexcept>
// #include "vehicle_model/kinematic_bicycle.h"
// #include "kalman_filters/ekf.h"

// // int main() {
// //     KinematicBicycle model(0.01, 2.7);
// //     auto [state_pred, jacobian_var] = model.state_transition(0, 0, 0.1, 10, 0);
// //     std::cout << "Predicted state: " << state_pred.transpose() << std::endl;
// //     std::cout << "Jacobian: " << jacobian_var.transpose() << std::endl;
// //     return 0;
// // }

// // Template class for variable states and control inputs
// template <int num_states, int num_controls, int num_measurements>

// class EKF{

// public:

//     double dt; // Time step
//     double l; // Wheelbase

//     EKF(double dt, double l) : dt(dt), l(l) {} // Constructor to initialize wheelbase and time step, useful for intializing the class

//     // Prediction step for EKF
//     std::tuple <Eigen::Matrix<double, num_states,1>, Eigen::Matrix<double, num_states, num_states>> predict(const Eigen::Matrix<double, num_states,1>& state, 
//                                                                                                             const Eigen::Matrix<double, num_states, num_states>& P, 
//                                                                                                             const Eigen::Matrix<double, num_controls,1>& control_input, 
//                                                                                                             const Eigen::Matrix<double, num_states, num_states>& F, 
//                                                                                                             const Eigen::Matrix<double, num_states, num_controls>& L,
//                                                                                                             const Eigen::Matrix<double, num_states, num_states>& Q_u) {
//         // Implement the prediction step of the EKF
//         KinematicBicycle model(dt, l); // Initialize the kinematic bicycle model with the time step and wheelbase
//         auto [predicted_state, jacobian_var] = model.state_transition(state(0), state(1), control_input(0), control_input(1), state(2)); // Get the predicted state and Jacobian from the kinematic bicycle model
        
//         // Predict state covariance
//         Eigen::Matrix<double, num_states, num_states> predicted_P = (F * P * F.transpose()) + (L * Q_u * L.transpose()); // Predict the state covariance using the Jacobian and process noise

//         // Throw an error if any element of the predicted state or predicted covariance contains NaN values
//         if (predicted_state.array().isNaN().any()) {
//             throw std::runtime_error("Predicted state contains NaN values. Check input parameters.");
//         }   
//         if (predicted_P.array().isNaN().any()) {
//             throw std::runtime_error("Predicted covariance contains NaN values. Check input parameters.");
//         }

//         return std::make_tuple(predicted_state, predicted_P);
//     }

//     // Update step fot the EKF
//     std::tuple<Eigen::Matrix<double, num_states,1>, Eigen::Matrix<double, num_states, num_states>> update (const Eigen::Matrix<double, num_states, 1>& predicted_state, 
//                                                                                                         const Eigen::Matrix<double, num_states, num_states>& predicted_P, 
//                                                                                                         const Eigen::Matrix<double, num_measurements, num_states>& H, 
//                                                                                                         const Eigen::Matrix<double, num_measurements, num_measurements>& M, 
//                                                                                                         const Eigen::Matrix<double, num_measurements, 1>& y,
//                                                                                                         const Eigen::Matrix<double, num_measurements, 1>& h,
//                                                                                                         const Eigen::Matrix<double, num_measurements, num_measurements>& R){
        
//         // Innovation Covariance S = (H * predicted_P * H.transpose()) + (M * R * M.transpose())
//         Eigen::Matrix<double,num_measurements, num_measurements> S = (H * predicted_P * H.transpose()) + (M * R * M.transpose()); // Calculate the innovation covariance
        
//         // Kalman Gain K = predicted_P * H.transpose() * S.inverse()
//         Eigen::Matrix<double,num_states, num_measurements> K = predicted_P * H.transpose() * S.inverse();

//         // Updated state estimate x = predicted_state + K * (y - h)
//         Eigen::Matrix<double, num_states, 1> updated_state = predicted_state + K * (y - h);

//         // Updated estimate covariance P = (I - K * H) * predicted_P    
//         Eigen::Matrix<double, num_states, num_states> P_upd = (Eigen::Matrix<double, num_states, num_states>::Identity() - (K * H)) * predicted_P;
//          // Implement the update step of the EKF
//         // This function will be implemented later as it requires the measurement model and measurement noise covariance matrix
        
//         if (updated_state.array().isNaN().any()) {
//             throw std::runtime_error("Updated state contains NaN values. Check input parameters.");
//         }
//         if (P_upd.array().isNaN().any()) {
//             throw std::runtime_error("Updated covariance contains NaN values. Check input parameters.");
//         }
        
//         return std::make_tuple(updated_state, P_upd); // Return the updated state and covariance as a tuple                                                                                                        
//     }
// };