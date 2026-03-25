#include <loadcsv.h>


/*

File Info:
This file load the .csv file of the data from the python script.

*/

// #include <Eigen/Dense>
// #include <fstream>
// #include <sstream>
// #include <vector>
// #include <iostream>

// Eigen::MatrixXd loadCSV(const std::string& path) {
//     std::ifstream file(path);
//     std::string line;
//     std::vector<std::vector<double>> data;

//     while (std::getline(file, line)) {
//         std::stringstream ss(line);
//         std::string value;
//         std::vector<double> row;

//         while (std::getline(ss, value, ',')) {
//             row.push_back(std::stod(value));
//         }

//         data.push_back(row);
//     }

//     int rows = data.size();
//     int cols = data[0].size();

//     Eigen::MatrixXd mat(rows, cols);
//     for (int i = 0; i < rows; ++i)
//         for (int j = 0; j < cols; ++j)
//             mat(i, j) = data[i][j];

//     return mat;
// };


int main(){
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

    if (x_pos.cols() != y_pos.cols() || x_pos.cols() != psi.cols() || x_pos.cols() != vx.cols() || x_pos.cols() != omega_z.cols()) {
        std::cerr << "Error: The number of columns in the CSV files do not match." << std::endl;
        return 1; // Exit with an error code
    }
    return 0;
};


