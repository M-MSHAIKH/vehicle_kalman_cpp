#include <iostream>
#include <stdexcept>
#include "vehicle_model/kinematic_bicycle.h"
#include "kalman_filters/ekf.h"

// int main() {
//     KinematicBicycle model(0.01, 2.7);
//     auto [state_pred, jacobian_var] = model.state_transition(0, 0, 0.1, 10, 0);
//     std::cout << "Predicted state: " << state_pred.transpose() << std::endl;
//     std::cout << "Jacobian: " << jacobian_var.transpose() << std::endl;
//     std::cout << "Main EKF function is working correctly." << std::endl;
//     return 0;
// }
