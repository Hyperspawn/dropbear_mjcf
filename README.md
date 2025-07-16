# Dropbear MJCF

A comprehensive MuJoCo MJCF (MuJoCo Modeling XML Format) model of the Dropbear humanoid robot, featuring detailed mechanical components, electronics, and a complete simulation environment.

https://github.com/user-attachments/assets/e93272bf-6c66-4bb6-bac3-c0b725a8e7bc


## Overview

The Dropbear humanoid robot is a sophisticated bipedal robot model designed for MuJoCo physics simulation. This model includes:

- **Full-body humanoid structure** with head, torso, arms, and legs
- **Advanced head mechanism** with Stewart platform (parallel manipulator) system
- **Detailed electronics** including ESP32 controllers, motor drivers, sensors
- **Realistic materials and textures** for enhanced visual simulation
- **Complete simulation environment** with skybox, lighting, and surfaces

## Features

### Robot Components
- **Head**: Stewart platform with 6-DOF motion, cameras, ultrasonic sensors, antennas
- **Torso**: Battery systems, power management, motor controllers
- **Arms**: 7-DOF articulated arms with RMD-X8 Pro motors and servo systems
- **Legs**: Complex leg mechanisms with hip, knee, and ankle joints
- **Pelvic Girdle**: Central mounting system connecting torso to legs

### Electronics & Sensors
- ESP32 microcontrollers
- DRV8825 stepper motor drivers
- Ultrasonic sensors
- Power converters and battery management
- Various communication antennas

### Simulation Environment
- Realistic materials (metal, glass, marble)
- Dynamic lighting with skybox textures
- Textured surfaces and environments
- Customizable simulation parameters

## Directory Structure

```
dropbear_mjcf/
├── dropbear_mjcf.xml          # Main MJCF model file
├── assets/                    # Simulation assets and 
│   └── simulate_alfNGFSkO3.mp4         # Simulation videos
├── meshes/                    # 3D mesh files (.stl)
├── textures/                  # Texture files for materials
├── README.md                  # This file
└── LICENSE                    # License information
```

## Requirements

### Software Dependencies
- **MuJoCo** 2.1.0 or later
- **Python** 3.8+
- **mujoco** Python bindings (`pip install mujoco`)

### Optional Dependencies
- **mujoco-viewer** for interactive visualization
- **numpy** for numerical operations
- **OpenCV** for image processing (if using camera sensors)

## Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/Hyperspawn/dropbear_mjcf.git
   cd dropbear_mjcf
   ```

2. **Install MuJoCo**:
   ```bash
   pip install mujoco
   ```

3. **Install optional dependencies**:
   ```bash
   pip install mujoco-viewer numpy opencv-python
   ```

## Usage

### Basic Simulation

Run a basic simulation of the Dropbear robot:

```python
import mujoco
import mujoco.viewer

# Load the model
model = mujoco.MjModel.from_xml_path('dropbear_mjcf.xml')
data = mujoco.MjData(model)

# Launch interactive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

### Headless Simulation

For headless simulation without graphics:

```python
import mujoco
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('dropbear_mjcf.xml')
data = mujoco.MjData(model)

# Run simulation steps
for i in range(1000):
    mujoco.mj_step(model, data)
    
    # Access robot state
    joint_positions = data.qpos
    joint_velocities = data.qvel
    
    # Apply control inputs
    # data.ctrl[:] = your_control_inputs
```

### Camera Rendering

Render camera views from the robot's sensors:

```python
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path('dropbear_mjcf.xml')
data = mujoco.MjData(model)

# Setup camera
camera = mujoco.MjvCamera()
camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
camera.fixedcamid = 0  # Use camera ID from model

# Render
renderer = mujoco.Renderer(model, height=480, width=640)
renderer.update_scene(data, camera)
image = renderer.render()
```

## Model Configuration

### Simulation Parameters
- **Timestep**: 0.001 seconds (1 kHz)
- **Solver**: PGS (Projected Gauss-Seidel)
- **Gravity**: -9.81 m/s²
- **Iterations**: 50

### Joint Configuration
- **Range**: -π to π radians (configurable per joint)
- **Damping**: 0.01 (default)
- **Armature**: 0.01 (default)
- **Friction Loss**: 0.01 (default)

### Contact Properties
- **Friction**: μ = 0.9, 0.2, 0.2 (sliding, torsional, rolling)
- **Solver Reference**: 0.001, 2 (stiffness, damping)

## Advanced Usage

### Custom Control

Implement custom control policies:

```python
def pd_controller(target_pos, current_pos, target_vel, current_vel, kp, kd):
    return kp * (target_pos - current_pos) + kd * (target_vel - current_vel)

# In simulation loop
data.ctrl[:] = pd_controller(target_positions, data.qpos, 
                            target_velocities, data.qvel, 
                            kp_gains, kd_gains)
```

### Sensor Data Access

Access sensor information:

```python
# Get sensor data
sensor_data = data.sensordata
touch_sensors = sensor_data[:num_touch_sensors]
imu_data = sensor_data[num_touch_sensors:num_touch_sensors+6]
```

## Troubleshooting

### Common Issues

1. **Model fails to load**:
   - Check that all mesh files exist in the `meshes/` directory
   - Verify texture files are present in `textures/`
   - Ensure file paths use forward slashes

2. **Simulation unstable**:
   - Reduce timestep in XML: `<option timestep="0.0005"/>`
   - Increase solver iterations: `<option iterations="100"/>`
   - Check for mesh collisions or unrealistic joint limits

3. **Performance issues**:
   - Use optimized meshes (some are in `meshes/unoptimized/`)
   - Reduce texture resolution if needed
   - Consider using simplified collision meshes

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Commit your changes (`git commit -am 'Add new feature'`)
4. Push to the branch (`git push origin feature/improvement`)
5. Create a Pull Request

## License

This project is licensed under the terms specified in the LICENSE file.

## Acknowledgments

- Built for MuJoCo physics simulation
- Developed by Hyperspawn
- Robot design based on humanoid robotics principles
- Stewart platform implementation for advanced head control

## Technical Specifications

### Degrees of Freedom
- **Head**: 6-DOF (Stewart platform)
- **Arms**: 7-DOF each (shoulder: 3-DOF, elbow: 1-DOF, wrist: 3-DOF)
- **Legs**: 6-DOF each (hip: 3-DOF, knee: 1-DOF, ankle: 2-DOF)
- **Torso**: 3-DOF (waist rotation and bending)

### Actuators
- **Head**: 6x stepper motors with lead screws
- **Arms**: RMD-X8 Pro servo motors
- **Legs**: High-torque servo motors with gear reduction
- **Torso**: RMD-X10 motors for waist articulation

### Sensors
- **Vision**: Intel RealSense cameras
- **Proximity**: Ultrasonic sensors
- **Inertial**: IMU systems
- **Force**: Contact sensors in feet and hands

For more detailed technical information, refer to the MJCF model file and component mesh files.
