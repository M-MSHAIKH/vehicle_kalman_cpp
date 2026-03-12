#pragma once
#include <Eigen/Dense>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <stdexcept> 

class KinematicBicycle {
public:
    double dt;
    double l;
    KinematicBicycle(double dt, double l);
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> state_transition(double x, double y,double psi, double vx, double omega_z);
};