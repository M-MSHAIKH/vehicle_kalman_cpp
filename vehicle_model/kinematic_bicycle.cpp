#include "vehicle_model/kinematic_bicycle.h"


// Constructor to initialize dt and l
KinematicBicycle::KinematicBicycle(double dt, double l) : dt(dt), l(l) {}

// State transition function
// use tuple for getting 3+ more return values from a function
std::tuple<Eigen::Vector3d, Eigen::Vector3d> KinematicBicycle::state_transition(double x, double y, double psi, double vx, double omega_z){
    double vx_safe = std::max(vx, 0.003); // Ensure velocity is slightly greater than zero to avoid division by zero
    double max_arg = M_PI / 2;
    double min_arg = -M_PI / 2;
    double clip_val = (omega_z * l) / vx_safe; // Calculate the argument for atan2
    double arg = std::clamp(clip_val, min_arg, max_arg); // Clip the argument to the valid range for atan2
    double beta = std::atan(arg); // Calculate the slip angle beta

    // Calculate the predicted state
    Eigen::Vector3d state_pred; // [x, y, psi], declare the size first to avoid any error later
        state_pred << x + (vx * cos(psi + beta) * dt),
                    y + (vx * sin(psi + beta) * dt),
                    psi + (omega_z * dt); // Assuming constant angular velocity

    // Calculate the jaciobian variable 
    // << insertion operator 
    Eigen::Vector3d jacobian_var;
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


/* How to access the state_pred and jacobian_var from the tuple returned by state_transition function:  
// Option A — structured binding (C++17)
    auto [state_pred, jacobian_var] = kf.predict();

    // Option B — std::get
    auto result   = kf.predict();
    auto state2   = std::get<0>(result);
    auto jacobian2= std::get<1>(result);

    std::cout << "State:    " << state_pred.transpose()    << std::endl;
    std::cout << "Jacobian: " << jacobian_var.transpose() << std::endl;

*/


// class KinematicBicycle{
// public:
//     double dt; // time step
//     double l;  // wheelbase

//     // Constructor — tells C++ how to initialize dt and l
//     KinematicBicycle(double dt, double l) : dt(dt), l(l) {}

//     py::dict state_transition(double x,double y,double omega_z, double vx, double psi) {

//         py::array_t<double> state_matrix({3,1}); // [x, y, psi] , Creates a numpy 2D array of size 3 [3,1] name state
//         auto buf = state_matrix.request();   // Get buffer info to access the underlying data
//         double *state_pred = (double *) buf.ptr;   // Get direct access to memory, Converts the raw memory pointer to a double* pointer so you can read/write values directly

//         double vx_safe = std::max(vx, 0.003); // Ensure velocity is non-negative
//         double max_arg = M_PI/2;
//         double min_arg = -M_PI /2;
//         double clip_val = (omega_z * l) / vx_safe; // Calculate the argument for atan2
//         double arg = std::clamp(clip_val, min_arg, max_arg); // Clip the argument to the valid range for atan2
//         double beta = std::atan(arg); // Calculate the slip angle beta

//         // next_state (pointers) and the state (numpy array) are shared the same memory through linker.
//         // So, you cannot name them same (as both are different data types) but they share the same value.
//         state_pred[0] = x + (vx * cos(psi + beta) * dt);
//         state_pred[1] = y + (vx * sin(psi + beta) * dt);
//         state_pred[2] = psi + (omega_z * dt); // Assuming constant angular velocity
        
//         // These variables goes for the jacobian calculation
//         py::array_t<double> jacobian_matrix ({3,1}); // [beta, psi, vx] , Creates a numpy 2D array of size 2 [3,1] name jacobian_matrix
//         auto jacobian_buf = jacobian_matrix.request();   // Get buffer info to access the underlying data
//         double *jacobian_ptr = (double *) jacobian_buf.ptr;   //    
//         jacobian_ptr[0] = beta; // Partial derivative of state with respect to vx
//         jacobian_ptr[1] = psi;
//         jacobian_ptr[2] = vx; // Partial derivative of state with respect to omega_z

//         // Throw an error if the predicted state contains NaN values
//         if (std::isnan(state_pred[0]) || std::isnan(state_pred[1]) || std::isnan(state_pred[2])) {
//             throw std::runtime_error("Predicted state contains NaN values. Check input parameters.");
//         }   

//         if (std::isnan(jacobian_ptr[0]) || std::isnan(jacobian_ptr[1]) || std::isnan(jacobian_ptr[2])) {
//             throw std::runtime_error("Jacobian contains NaN values. Check input parameters.");
//         }   

//         // Create a dictionary to hold the results
//         py::dict result;
//         result["state"] = state_matrix; // Add the predicted state to the result dictionary
//         result["jacobian"] = jacobian_matrix; // Add the Jacobian variable to the result dictionary
//         return result; // Return the predicted state and the Jacobian variable as a tuple
//     }
// };

// // class KinematicBicycle {
// // };

// // Here type the same module name as you typed in setup.py and CmakeLists.txt
// // PYBIND11_MODULE(kinematic_bicycle, m) {
// //     m.doc() = "Kinematic bicycle model"; // optional module docstring
// //     m.def("some_fu", &KinematicBicycle, "A function that adds two numbers");
// // }

// PYBIND11_MODULE(kinematic_bicycle, m) {
//     m.doc() = "Kinematic bicycle model";
//     py::class_<KinematicBicycle>(m, "KinematicBicycle")
//         .def(py::init<double, double>(), py::arg("dt"), py::arg("L"))
//         .def("state_transition", &KinematicBicycle::state_transition,
//              py::arg("x"), py::arg("y"), py::arg("omega_z"),
//              py::arg("vx"), py::arg("psi"));
// }