#pragma once
#include <Eigen/Dense>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <iostream>

class KinematicBicycle {
public:
    double dt;
    double L;
    KinematicBicycle(double dt, double L);
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> state_transition(double x, double y, double omega_z, double vx, double psi);
};