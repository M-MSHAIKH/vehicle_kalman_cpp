#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>

Eigen::MatrixXd loadCSV(const std::string& path) {
    std::ifstream file(path);
    std::string line;
    std::vector<std::vector<double>> data;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> row;

        while (std::getline(ss, value, ',')) {
            row.push_back(std::stod(value));
        }

        data.push_back(row);
    }

    int rows = data.size();
    int cols = data[0].size();

    Eigen::MatrixXd mat(rows, cols);
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            mat(i, j) = data[i][j];

    return mat;
};


/*
Load CSV files into Eigen matrices. The loadCSV function reads a CSV file from the 
specified path and returns an Eigen::MatrixXd containing the data. The main function 
demonstrates how to use the loadCSV function to read multiple CSV files and store 
their contents in Eigen matrices. Finally, it prints the size of one of the loaded 
matrices to verify that the data has been loaded correctly.
*/