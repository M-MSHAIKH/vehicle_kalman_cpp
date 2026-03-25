#include <Eigen/Dense>
#include <tuple>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <kinematic_bicycle.h>
#include <jacobiankinematic.h>
#include <covariance.h>
#include <ekf.h>
#include <loadcsv.h>

constexpr int num_states = 3; // Number of states in the state vector (e.g., x, y, psi)
constexpr int num_controls = 2; // Number of control inputs (e.g., vx, omega_z)
constexpr int num_measurements = 2; // Number of measurements (e.g., x_meas, y_meas)

// Function to save a matrix to a CSV file
void saveCSV(const std::string& filename,
         const Eigen::MatrixXd& mat) {
    std::ofstream file(filename);
    for (int i = 0; i < mat.cols(); i++) {
        for (int j = 0; j < mat.rows(); j++) {
            file << std::setprecision(10) << mat(j, i);
            if (j < mat.rows() - 1) file << ",";
        }
        file << "\n";
    }
    file.close();
}

int test_ekf(){

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

    double dt = 0.005; // Time step in seconds
    double l = 2.5; // Wheelbase length in meters

    // Create an instance of the KinematicBicycle class
    KinematicBicycle<num_states, num_controls> kinematic_bicycle(dt, l); 
    JacobianKinematicBicycle<num_states, num_controls, num_measurements> jacobian_kinematic_bicycle(dt);
    CovarianceMatrices<num_states, num_controls, num_measurements> covariance;
    EKF<num_states, num_controls, num_measurements> ekf(dt,l);

    // Define number of steps based on the number of rows in the CSV files
    const int num_steps = x_pos.rows();

    // Create matrices to store the values
    Eigen::Matrix<double, num_states, Eigen::Dynamic> X_pred(num_states, num_steps); // To store the predicted states for each step
    Eigen::Matrix<double, num_states, Eigen::Dynamic> X_upd(num_states, num_steps);
    Eigen::Matrix<double, num_states, Eigen::Dynamic> jacobian_variables(num_states, num_steps);
    std::vector<Eigen::Matrix3d> P_pred_save; // To store the predicted covariance matrix
    P_pred_save.reserve(num_steps); // Pre-allocate space for the predicted covariance matrices
    Eigen::Matrix3d P0 = Eigen::Matrix3d::Identity(); // Initial covariance matrix
    P_pred_save.push_back(P0); // Add the initial covariance matrix to the vector 
    std::vector<Eigen::Matrix3d> P_upd_save; // To store the updated covariance matrix 
    P_upd_save.reserve(num_steps); // Pre-allocate space for the updated covariance matrices
    Eigen::Matrix3d P0_upd = Eigen::Matrix3d::Identity(); // Initial updated covariance matrix
    P_upd_save.push_back(P0_upd); // Add the initial updated covariance matrix to the vector
    std::vector<Eigen::Matrix<double, num_states, num_measurements>> K_save; // To store the Kalman gain
    K_save.reserve(num_steps); // Pre-allocate space for the Kalman gains
    Eigen::Matrix<double, num_states, num_measurements> K0 = Eigen::Matrix<double, num_states, num_measurements>::Zero(); // Initial Kalman gain
    K_save.push_back(K0); // Add the initial Kalman gain to the vector

    // Prediction: Call the covariance function
    Eigen::Matrix<double, num_controls, num_controls> Q_u = covariance.process_noise_covariance(0.1, 0.1); // Example process noise covariance values
    Eigen::Matrix<double, num_measurements, num_measurements> R = covariance.measurement_noise_covariance(0.1, 0.1); // Example measurement noise covariance values

    // Prediction: Call the jacobian function
    Eigen::Matrix<double, num_measurements, num_states> H = jacobian_kinematic_bicycle.H_jacobian();
    Eigen::Matrix<double, num_measurements, num_measurements> M = jacobian_kinematic_bicycle.M_jacobian();

    


    // Main for loop
    for (int i = 0; i < num_steps; i++) {
        // if (i == 0){
        //     state_pred.col(i) 
        // }

        // Measurement vector (x, y)
        Eigen::Matrix<double, num_measurements, 1> measurement;
        measurement << x_pos(i, 0), y_pos(i, 0);


        Eigen::Matrix<double, 2, 1> control_input;
        control_input << vx(i, 0), omega_z(i, 0); // Control input: [vx, omega_z]
        
        

        // Prediction: Call the state_transition function
        if (i == 0){
            Eigen::Matrix<double, 3, 1> initial_state;
            initial_state << 0, 0, 0; // Initial state: [x, y, psi]

            // Prediction: Call the predict function
            auto result_ekf_pred = ekf.predict(initial_state, P0, control_input, Q_u);
            X_pred.col(i) = std::get<0>(result_ekf_pred); // Predicted state
            Eigen::Matrix<double, num_states, num_states> P_pred = std::get<1>(result_ekf_pred); // Predicted state
            P_pred_save.push_back(P_pred); // Save the predicted covariance matrix

            // Update: Call the update function
            auto result_ekf_upd = ekf.update(X_pred.col(i), P0_upd, H, M, measurement, R);
            X_upd.col(i) = std::get<0>(result_ekf_upd); // Updated state
            Eigen::Matrix<double, num_states, num_states> P_upd = std::get<1>(result_ekf_upd); // Updated covariance matrix 
            P_upd_save.push_back(P_upd); // Save the updated covariance matrix
            Eigen::Matrix<double, num_states, num_measurements> K = std::get<2>(result_ekf_upd); // Kalman gain
            K_save.push_back(K); // Save the Kalman gain


        } else {
            // Prediction: Call the predict function
            auto result_ekf_pred = ekf.predict(X_upd.col(i-1), P_upd_save[i-1], control_input, Q_u);
            X_pred.col(i) = std::get<0>(result_ekf_pred); // Predicted state
            Eigen::Matrix<double, num_states, num_states> P_pred = std::get<1>(result_ekf_pred); // Predicted state
            P_pred_save.push_back(P_pred); // Save the predicted covariance matrix

            // Update: Call the update function
            auto result_ekf_upd = ekf.update(X_pred.col(i), P_pred_save[i], H, M, measurement, R);
            X_upd.col(i) = std::get<0>(result_ekf_upd); // Updated state
            Eigen::Matrix<double, num_states, num_states> P_upd = std::get<1>(result_ekf_upd); // Updated covariance matrix 
            P_upd_save.push_back(P_upd); // Save the updated covariance matrix
            Eigen::Matrix<double, num_states, num_measurements> K = std::get<2>(result_ekf_upd); // Kalman gain
            K_save.push_back(K); // Save the Kalman gain

        }

        // write X_upd (num_steps rows, 3 cols)
        // std::ofstream fx("X_upd.csv");
        // for (int i=0; i < num_steps; ++i) {
        //     fx << X_upd(0,i) << ',' << X_upd(1,i) << ',' << X_upd(2,i) << '\n';
        // }
        // fx.close();

        // // write K_save (each line: 3x2 flattened row-major -> 6 values)
        // std::ofstream fk("K.csv");
        // for (auto &K : K_save) {
        //     fk << K(0,0) << ',' << K(1,0) << ',' << K(2,0) << ',' << K(0,1) << ',' << K(1,1) << ',' << K(2,1) << '\n';
        // }
        // fk.close();

        // // write P_upd_save (each line: 3x3 flattened -> 9 values)
        // std::ofstream fp("P_upd.csv");
        // for (auto &P : P_upd_save) {
        //     for (int r=0;r<3;++r) for (int c=0;c<3;++c) {
        //         fp << P(r,c);
        //         if (!(r==2 && c==2)) fp << ',';
        // }
        // fp << '\n';
        // }
        // fp.close();

    }
    saveCSV("X_upd.csv", X_upd);
    saveCSV("K.csv", Eigen::Map<Eigen::Matrix<double, num_states * num_measurements, Eigen::Dynamic>>(K_save[0].data(), num_states * num_measurements, K_save.size()));
    saveCSV("P_upd.csv", Eigen::Map<Eigen::Matrix<double, num_states * num_states, Eigen::Dynamic>>(P_upd_save[0].data(), num_states * num_states, P_upd_save.size()));


    std::cout << "process complete" << std::endl;

    return 0;
}



int main(){
    return test_ekf();
}