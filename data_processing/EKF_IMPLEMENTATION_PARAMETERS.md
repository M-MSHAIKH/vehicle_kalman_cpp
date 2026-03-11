# EKF Implementation Parameters - Quick Reference
**Audi A2D2 Dataset**

---

## Sensor Bias Corrections (Apply in Preprocessing)

```python
# Remove these biases from raw measurements:
ax_corrected = ax_raw - (-0.0252)  # m/s²
ay_corrected = ay_raw - 0.1434     # m/s²
az_corrected = az_raw - 0.0         # m/s² (negligible bias)
omega_z_corrected = omega_z_raw - (-0.3244)  # deg/s
```

---

## Sampling Rates

| Sensor | Rate | Period | Use |
|--------|------|--------|-----|
| **Accelerometer (ax, ay, az)** | 200 Hz | 5 ms | Prediction step |
| **Gyroscope (omega_z)** | 200 Hz | 5 ms | Prediction step |
| **GPS (lat, lon)** | 5 Hz | 200 ms | Update step |
| **Steering Angle** | 100 Hz | 10 ms | Control input |
| **Vehicle Speed** | Variable | - | Update step |

**Recommended EKF timestep:** `dt = 0.005 sec` (5 ms, matching IMU)

---

## Measurement Noise Standard Deviations (for R matrix)

### From Statistical Analysis:

```python
# Measurement noise standard deviations (σ)
sigma_ax = 0.6284      # m/s²  (use 0.2 during low dynamics)
sigma_ay = 0.5438      # m/s²
sigma_az = 0.2065      # m/s²
sigma_omega_z = 4.7818 # deg/s = 0.0834 rad/s

# GPS noise (typical values for automotive GPS)
sigma_gps_lat = 0.00005  # degrees ≈ 5 meters
sigma_gps_lon = 0.00005  # degrees ≈ 5 meters
# Alternative: convert to meters after coordinate transformation
sigma_gps_x = 5.0      # meters
sigma_gps_y = 5.0      # meters

# Velocity (if using vehicle speed from CAN)
sigma_speed = 0.5      # m/s (typical)
```

### Measurement Noise Covariance Matrix R:

**Option 1: GPS + IMU**
```python
R = np.diag([
    sigma_gps_x**2,      # GPS x
    sigma_gps_y**2,      # GPS y
    sigma_ax**2,         # Acceleration x
    sigma_ay**2,         # Acceleration y
    (sigma_omega_z * np.pi/180)**2  # Yaw rate (rad/s)
])
```

**Option 2: If including vehicle speed**
```python
R = np.diag([
    sigma_gps_x**2,      # GPS x
    sigma_gps_y**2,      # GPS y
    sigma_ax**2,         # Acceleration x
    sigma_ay**2,         # Acceleration y
    (sigma_omega_z * np.pi/180)**2,  # Yaw rate
    sigma_speed**2       # Vehicle speed
])
```

---

## Process Noise Covariance (Q matrix)

**Initial values (tune empirically):**

```python
# For state x = [px, py, psi, vx, vy, psi_dot]
Q = np.diag([
    0.1,     # px process noise (m²)
    0.1,     # py process noise (m²)
    0.01,    # psi process noise (rad²)
    0.5,     # vx process noise ((m/s)²)
    0.5,     # vy process noise ((m/s)²)
    0.01     # psi_dot process noise ((rad/s)²)
])
```

**Tuning guidance:**
- Increase Q if EKF is too slow to respond to maneuvers
- Decrease Q if estimates are too noisy/jumpy
- Start conservative (smaller Q), increase if needed

---

## Initial State and Covariance

```python
# Initial state (at t=0)
x0 = np.array([
    long_x[0],    # px: Initial longitude → convert to local meters
    lat_y[0],     # py: Initial latitude → convert to local meters
    0.0,          # psi: Initial heading (or compute from first GPS points)
    0.0,          # vx: Initial velocity (or use Vx[0])
    0.0,          # vy: Assume zero lateral velocity initially
    0.0           # psi_dot: Initial yaw rate (or use omega_z[0])
])

# Initial covariance (high uncertainty initially)
P0 = np.diag([
    100.0,   # px uncertainty (m²)
    100.0,   # py uncertainty (m²)
    1.0,     # psi uncertainty (rad²)
    25.0,    # vx uncertainty ((m/s)²)
    25.0,    # vy uncertainty ((m/s)²)
    0.1      # psi_dot uncertainty ((rad/s)²)
])
```

---

## State Vector Definition

```python
# State vector (6 states)
x = [px, py, psi, vx, vy, psi_dot]

# Where:
# px, py:      Position in local ENU coordinates (meters)
# psi:         Heading angle (radians), 0 = North, increasing clockwise
# vx, vy:      Velocity in global frame (m/s)
# psi_dot:     Yaw rate (rad/s)
```

---

## Process Model (Constant Velocity)

### Prediction Equations:
```python
def predict_state(x, dt):
    """
    Constant velocity motion model
    """
    px, py, psi, vx, vy, psi_dot = x
    
    # Update position
    px_new = px + vx * dt
    py_new = py + vy * dt
    
    # Update heading
    psi_new = psi + psi_dot * dt
    
    # Velocity and yaw rate constant (will be updated by measurements)
    vx_new = vx
    vy_new = vy
    psi_dot_new = psi_dot
    
    return np.array([px_new, py_new, psi_new, vx_new, vy_new, psi_dot_new])
```

### State Transition Jacobian (F):
```python
def jacobian_F(x, dt):
    """
    Jacobian of process model
    """
    F = np.array([
        [1, 0, 0, dt, 0,  0],
        [0, 1, 0, 0,  dt, 0],
        [0, 0, 1, 0,  0,  dt],
        [0, 0, 0, 1,  0,  0],
        [0, 0, 0, 0,  1,  0],
        [0, 0, 0, 0,  0,  1]
    ])
    return F
```

---

## Measurement Model

### GPS Measurement (5 Hz - asynchronous):
```python
def h_gps(x):
    """
    GPS observes position only
    """
    px, py, _, _, _, _ = x
    return np.array([px, py])

def jacobian_H_gps(x):
    """
    Jacobian for GPS measurement
    """
    H = np.array([
        [1, 0, 0, 0, 0, 0],  # px
        [0, 1, 0, 0, 0, 0]   # py
    ])
    return H
```

### IMU Measurement (200 Hz):
```python
def h_imu(x):
    """
    IMU observes accelerations and yaw rate
    Note: This is simplified - proper model needs body frame transformation
    """
    px, py, psi, vx, vy, psi_dot = x
    
    # Simplified: assume measured accelerations
    # In full model, would compute expected ax, ay from velocity derivatives
    # For now, yaw rate directly observable:
    return np.array([psi_dot])

def jacobian_H_imu(x):
    """
    Jacobian for IMU measurement
    """
    H = np.array([
        [0, 0, 0, 0, 0, 1]   # psi_dot
    ])
    return H
```

---

## Unit Conversions

```python
# Degrees to radians
omega_z_rad = omega_z_deg * (np.pi / 180)

# km/h to m/s
v_ms = v_kmh / 3.6

# GPS degrees to meters (approximate for small areas)
# At latitude ~48° (Munich):
meters_per_deg_lat = 111320  # meters
meters_per_deg_lon = 74000   # meters (varies with latitude)

# Convert GPS to local ENU:
lat_ref = lat_y[0]
lon_ref = long_x[0]

px = (long_x - lon_ref) * meters_per_deg_lon
py = (lat_y - lat_ref) * meters_per_deg_lat
```

**Better approach: Use geodetic library**
```python
from pyproj import Proj

# Define local projection centered at first GPS point
projection = Proj(proj='utm', zone=32, ellps='WGS84')  # Munich is in zone 32
px, py = projection(long_x, lat_y)
```

---

## Dataset-Specific Information

```python
# Dataset path
data_path = "/Users/moaadil/audi_a2dc/munich/camera_lidar/20190401_121727/bus/20190401121727_bus_signals.json"

# Signal names in JSON
signals = {
    'acceleration_x': 'acceleration_x',
    'acceleration_y': 'acceleration_y', 
    'acceleration_z': 'acceleration_z',
    'yaw_rate': 'angular_velocity_omega_z',
    'latitude': 'latitude_degree',
    'longitude': 'longitude_degree',
    'steering': 'steering_angle_calculated',
    'speed': 'vehicle_speed'
}

# Dataset characteristics
total_duration = 919.69  # seconds (estimated from timestamps)
total_samples_imu = 183937
total_samples_gps = 4599
```

---

## EKF Algorithm Pseudocode

```python
# Initialize
x_est = x0
P_est = P0
dt = 0.005  # 5 ms

# Storage for results
x_history = []
P_history = []

# Main loop (at IMU rate: 200 Hz)
for i in range(len(time_ax)):
    
    # PREDICTION STEP (every timestep)
    # -------------------------------
    F = jacobian_F(x_est, dt)
    
    # Predict state
    x_pred = predict_state(x_est, dt)
    
    # Predict covariance
    P_pred = F @ P_est @ F.T + Q
    
    
    # UPDATE STEP (when measurements available)
    # -----------------------------------------
    
    # Check if GPS measurement available at this timestep
    if gps_available[i]:
        # GPS update
        z_gps = np.array([px_gps[i], py_gps[i]])
        H_gps = jacobian_H_gps(x_pred)
        
        # Innovation
        y = z_gps - h_gps(x_pred)
        
        # Innovation covariance
        S = H_gps @ P_pred @ H_gps.T + R_gps
        
        # Kalman gain
        K = P_pred @ H_gps.T @ np.linalg.inv(S)
        
        # Update state and covariance
        x_est = x_pred + K @ y
        P_est = (np.eye(6) - K @ H_gps) @ P_pred
    
    # IMU update (every timestep at 200 Hz)
    else:
        # Use yaw rate measurement
        z_imu = np.array([omega_z[i]])
        H_imu = jacobian_H_imu(x_pred)
        
        # Innovation
        y = z_imu - h_imu(x_pred)
        
        # Innovation covariance
        S = H_imu @ P_pred @ H_imu.T + R_imu
        
        # Kalman gain
        K = P_pred @ H_imu.T @ np.linalg.inv(S)
        
        # Update
        x_est = x_pred + K @ y
        P_est = (np.eye(6) - K @ H_imu) @ P_pred
    
    # Store results
    x_history.append(x_est.copy())
    P_history.append(P_est.copy())
```

---

## Performance Metrics to Calculate

```python
import numpy as np

# Position RMSE
position_error = np.sqrt((x_ekf[:, 0] - px_gps)**2 + (x_ekf[:, 1] - py_gps)**2)
rmse = np.sqrt(np.mean(position_error**2))
max_error = np.max(position_error)

print(f"Position RMSE: {rmse:.2f} m")
print(f"Max Position Error: {max_error:.2f} m")

# Innovation analysis (should be zero-mean, white)
innovations = []  # collect during filtering
innovation_mean = np.mean(innovations, axis=0)
innovation_std = np.std(innovations, axis=0)

print(f"Innovation mean: {innovation_mean}")  # Should be near zero
print(f"Innovation std: {innovation_std}")
```

---

## Recommended Libraries

```python
# Installation
pip install numpy scipy matplotlib filterpy pyproj

# Usage
import numpy as np
from scipy.linalg import inv
import matplotlib.pyplot as plt
from filterpy.kalman import ExtendedKalmanFilter
from pyproj import Proj
```

---

## Troubleshooting Guide

| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| **Filter diverges** | Q too small or R too large | Increase Q or decrease R |
| **Filter too slow** | Q too small | Increase process noise Q |
| **Estimate too noisy** | R too small or Q too large | Increase measurement noise R |
| **Position drift** | Missing GPS updates | Check GPS update logic |
| **Heading error** | Wrong yaw rate integration | Verify unit conversion (deg→rad) |
| **Large initial transient** | P0 too small | Increase initial uncertainty P0 |

---

## Next Steps Checklist

- [ ] Apply bias corrections to raw data
- [ ] Convert GPS to local coordinates (ENU)
- [ ] Convert all units (deg→rad, km/h→m/s)
- [ ] Implement prediction step with F matrix
- [ ] Implement GPS update step
- [ ] Implement IMU update step
- [ ] Test with first 1000 samples
- [ ] Tune Q and R matrices
- [ ] Validate against full dataset
- [ ] Plot trajectory: EKF vs GPS
- [ ] Calculate performance metrics

---

## Expected Results (Benchmark)

After proper tuning, you should achieve:

- **Position RMSE:** < 2 meters
- **Max position error:** < 5 meters
- **Velocity estimate:** Smooth, no jumps
- **Innovation sequence:** Zero-mean, consistent with S_k

---

*Quick Reference Guide - Ready for Implementation*  
*All values derived from Audi A2D2 dataset analysis*
