# RoboSub PID Controller

A comprehensive 6-DOF PID controller package for submarine control in underwater environments. This package handles water resistance and provides smooth, stable control for all degrees of freedom.

## Overview

This package provides:
- 6-DOF PID controllers (3 linear + 3 angular)
- Real-time parameter tuning capabilities
- Automated tuning utilities using step response analysis
- DVL and IMU sensor feedback integration
- Anti-windup protection and output limiting

## Package Structure

```
robosub_pid_controller/
├── robosub_pid_controller/
│   ├── __init__.py
│   ├── pid_controller.py    # Main PID controller node
│   └── pid_tuner.py         # Automated tuning utility
├── config/
│   └── pid_config.yaml      # PID parameters configuration
├── launch/
│   └── pid_controller.launch.py  # Launch file
└── README.md
```

## Nodes

### 1. PID Controller Node (`pid_controller`)

**Subscribed Topics:**
- `/cmd_vel_raw` (geometry_msgs/Twist): Raw velocity commands
- `/sensors/imu` (sensor_msgs/Imu): IMU feedback for orientation
- `/sensors/dvl/velocity` (dave_interfaces/msg/DVL): DVL feedback for velocity

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist): PID-controlled velocity commands

**Parameters:**
- `linear_x.kp`, `linear_x.ki`, `linear_x.kd`: PID gains for X-axis linear control
- `linear_y.kp`, `linear_y.ki`, `linear_y.kd`: PID gains for Y-axis linear control
- `linear_z.kp`, `linear_z.ki`, `linear_z.kd`: PID gains for Z-axis linear control
- `angular_x.kp`, `angular_x.ki`, `angular_x.kd`: PID gains for roll control
- `angular_y.kp`, `angular_y.ki`, `angular_y.kd`: PID gains for pitch control
- `angular_z.kp`, `angular_z.ki`, `angular_z.kd`: PID gains for yaw control
- `linear_output_limit`: Maximum output for linear controllers
- `angular_output_limit`: Maximum output for angular controllers
- `velocity_feedback_weight`: Weight of velocity feedback (0.0-1.0)

### 2. PID Tuner Node (`pid_tuner`)

Automated tuning utility that performs step response tests and suggests PID parameters using the Ziegler-Nichols method.

## Usage

### Basic Launch

```bash
# Launch with default configuration
ros2 launch robosub_pid_controller pid_controller.launch.py

# Launch with custom config file
ros2 launch robosub_pid_controller pid_controller.launch.py config_file:=/path/to/custom_config.yaml

# Launch with parameter tuning GUI
ros2 launch robosub_pid_controller pid_controller.launch.py use_gui:=true
```

### Manual Parameter Tuning

```bash
# Real-time parameter adjustment
ros2 param set /robosub_pid_controller linear_x.kp 2.5
ros2 param set /robosub_pid_controller linear_x.ki 0.2
ros2 param set /robosub_pid_controller linear_x.kd 0.8

# View current parameters
ros2 param list /robosub_pid_controller
ros2 param get /robosub_pid_controller linear_x.kp
```

### Automated Tuning

```bash
# Run full 6-DOF tuning sequence
ros2 run robosub_pid_controller pid_tuner full

# Test specific axis
ros2 run robosub_pid_controller pid_tuner linear x
ros2 run robosub_pid_controller pid_tuner angular z
```

## PID Tuning Process

### 1. Initial Setup
1. Ensure the submarine simulation is running
2. Verify sensor topics are publishing (`/sensors/imu`, `/sensors/dvl/velocity`)
3. Launch the PID controller with default parameters

### 2. Automated Tuning (Recommended)
```bash
# Run automated tuning for all axes
ros2 run robosub_pid_controller pid_tuner full
```

This will:
- Perform step response tests for each DOF
- Analyze system response characteristics
- Suggest PID parameters using Ziegler-Nichols method
- Display tuning recommendations

### 3. Manual Fine-tuning

Based on the automated suggestions, manually adjust parameters:

**For stable, slow response:**
- Increase Kp for faster response
- Increase Kd to reduce overshoot
- Increase Ki to eliminate steady-state error

**For oscillatory response:**
- Decrease Kp
- Increase Kd
- Decrease Ki

**For sluggish response:**
- Increase Kp
- Decrease Kd if causing instability

### 4. Parameter Guidelines

**Linear Motion (X, Y, Z):**
- Start with Kp: 1.0-3.0
- Ki: 0.05-0.2 (low to prevent windup)
- Kd: 0.3-0.8

**Angular Motion (Roll, Pitch, Yaw):**
- Start with Kp: 1.0-2.0
- Ki: 0.02-0.1
- Kd: 0.2-0.5

### 5. Advanced Tuning

**Output Limits:**
- Adjust `linear_output_limit` and `angular_output_limit` based on thruster capabilities
- Too high: May cause thruster saturation
- Too low: Limits controller authority

**Feedback Weight:**
- `velocity_feedback_weight`: Balance between feedforward and feedback control
- Higher values (0.8-1.0): More responsive to velocity feedback
- Lower values (0.3-0.7): More emphasis on command tracking

## Testing and Validation

### Step Response Test
```bash
# Test linear X response
ros2 topic pub --once /cmd_vel_raw geometry_msgs/Twist '{linear: {x: 0.5, y: 0, z: 0}}'

# Monitor response
ros2 topic echo /sensors/dvl/velocity
```

### Performance Metrics
- **Rise Time:** Time to reach 90% of steady-state value
- **Settling Time:** Time to stay within 2% of steady-state
- **Overshoot:** Maximum overshoot percentage
- **Steady-State Error:** Final tracking error

## Troubleshooting

### Common Issues

1. **High Oscillations:**
   - Reduce Kp gains
   - Increase Kd gains
   - Check for sensor noise

2. **Slow Response:**
   - Increase Kp gains
   - Verify sensor feedback is working
   - Check output limits

3. **Steady-State Error:**
   - Increase Ki gains
   - Check for bias in sensors
   - Verify integral limits

4. **Controller Instability:**
   - Reduce all gains by 50%
   - Increase control loop frequency
   - Check for delays in feedback

### Debugging Commands

```bash
# Check node status
ros2 node info /robosub_pid_controller

# Monitor control output
ros2 topic echo /cmd_vel

# Check sensor inputs
ros2 topic echo /sensors/dvl/velocity --once
ros2 topic echo /sensors/imu --once

# View parameter values
ros2 param dump /robosub_pid_controller
```

## Configuration Files

### Default Configuration (`config/pid_config.yaml`)

The default configuration provides conservative PID gains suitable for most submarine configurations. Modify these values based on your specific vehicle characteristics:

- **Mass and inertia:** Heavier vehicles need higher gains
- **Thruster configuration:** More thrusters allow higher limits
- **Operating environment:** Currents and disturbances affect tuning

### Custom Configurations

Create custom configuration files for different operating modes:
- `aggressive_config.yaml`: Higher gains for fast response
- `stable_config.yaml`: Lower gains for stable operation
- `precision_config.yaml`: Tuned for precise maneuvering

## Integration Notes

1. **Thruster Allocation:** This controller outputs desired velocities. Ensure your thruster allocation node subscribes to `/cmd_vel`

2. **Coordinate Frames:** The controller assumes:
   - X: Forward (surge)
   - Y: Starboard (sway)
   - Z: Down (heave)
   - Roll, Pitch, Yaw: Right-hand rule

3. **Safety:** Always set appropriate output limits to prevent thruster damage or vehicle instability

## License

This package is released under the Apache 2.0 License. 