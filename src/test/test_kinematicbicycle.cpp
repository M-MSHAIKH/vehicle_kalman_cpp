#include <kinematic_bicycle.h>
#include <iostream>
#include <loadcsv.h>

/*

This file only contains the test of the kinematic bicycle model.

*/

int test_kinematic_bicycle(){

    // Load the actual data from the CSV files
    Eigen::MatrixXd x_pos = loadCSV("/Users/moaadil/audi_a2dc/state_estimation_copy/data/x_pos.csv");
    Eigen::MatrixXd y_pos = loadCSV("/Users/moaadil/audi_a2dc/state_estimation_copy/data/y_pos.csv");
    Eigen::MatrixXd psi = loadCSV("/Users/moaadil/audi_a2dc/state_estimation_copy/data/yaw.csv");
    Eigen::MatrixXd vx = loadCSV("/Users/moaadil/audi_a2dc/state_estimation_copy/data/vx.csv");
    Eigen::MatrixXd omega_z = loadCSV("/Users/moaadil/audi_a2dc/state_estimation_copy/data/omega_z.csv");
    std::cout << x_pos.size();
    std::cout <<x_pos.rows() << " " << x_pos.cols() << std::endl;

    if (x_pos.rows() != y_pos.rows() || x_pos.rows() != psi.rows() || x_pos.rows() != vx.rows() || x_pos.rows() != omega_z.rows()) {
        std::cerr << "Error: The number of rows in the CSV files do not match." << std::endl;
        return 1; // Exit with an error code
    }

    if (x_pos.cols() != y_pos.cols() || x_pos.cols() != psi.cols() || x_pos.cols() != vx.cols() || x_pos.cols() != omega_z.cols()) {
        std::cerr << "Error: The number of columns in the CSV files do not match." << std::endl;
        return 1; // Exit with an error code
    }

    // Create an instance of the KinematicBicycle class
    KinematicBicycle<3, 2> kinematic_bicycle(0.1, 2.5); // dt = 0.1s, l = 2.5m

    // Define an input state and control input
    const int num_steps = x_pos.rows();

    Eigen::Matrix<double, 3, Eigen::Dynamic> state_pred(3, num_steps); // To store the predicted states for each step
    Eigen::Matrix<double, 3, Eigen::Dynamic> jacobian_variables(3, num_steps);

    for (int i = 0; i < num_steps; i++) {
        // if (i == 0){
        //     state_pred.col(i) 
        // }

        // Remove this if you are not debugging
        Eigen::Matrix<double, 3, 1> actual_state;
        actual_state << x_pos(i, 0), y_pos(i, 0), psi(i, 0); // Actual state: [x, y, psi]


        Eigen::Matrix<double, 2, 1> control_input;
        control_input << vx(i, 0), omega_z(i, 0); // Control input: [vx, omega_z]

        // Call the state_transition function
        if (i == 0){
            Eigen::Matrix<double, 3, 1> initial_state;
            initial_state << 0, 0, 0; // Initial state: [x, y, psi]
            auto result = kinematic_bicycle.state_transition(initial_state, control_input);
            state_pred.col(i) = std::get<0>(result); // Predicted state
            jacobian_variables.col(i) = std::get<1>(result); // Jacobian variables

        } else {
            auto result = kinematic_bicycle.state_transition(state_pred.col(i-1), control_input);
            state_pred.col(i) = std::get<0>(result); // Predicted state
            jacobian_variables.col(i) = std::get<1>(result); // Jacobian variables  

        }

        // Print the predicted state and Jacobian variables for the current step
        // if (i >= 200){
        //     std::cout << "Step " << i << ":\n";
        //     std::cout << "Predicted State:\n" << state_pred.col(i) << std::endl;
        //     std::cout << "Actual State:\n" << actual_state << std::endl;
        //     std::cout << "Jacobian Variables:\n" << jacobian_variables.col(i) << std::endl;
        //     std::cout << "Control Input:\n" << control_input << std::endl;
        // }
        // std::cout << "-----------------------------" << std::endl; 

    }


    // Print the predicted state and Jacobian variables
    // std::cout << "Predicted State:\n" << state_pred << std::endl;
    // std::cout << "Jacobian Variables:\n" << jacobian_variables << std::endl;
    std::cout << "No. of columns in state_pred: " << state_pred.cols() << std::endl;
    std::cout << "No. of rows in state_pred: " << state_pred.rows() << std::endl;
    std::cout << "No. of columns in jacobian_variables: " << jacobian_variables.cols() << std::endl;
    std::cout << "No. of rows in jacobian_variables: " << jacobian_variables.rows() << std::endl;
    std::cout << "Processed " << num_steps << " steps." << std::endl;

    return 0;
}

int main() {
    return test_kinematic_bicycle();
}