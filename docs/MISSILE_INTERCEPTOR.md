# Missile Interceptor Simulation Documentation

## Overview

The Missile Interceptor Simulation implements a physics-based model of missile interception scenarios using Proportional Navigation (PN) guidance. This simulation demonstrates the effectiveness of the PN guidance law under various engagement scenarios.

## Implementation

### Core Components

1. **Interceptor**: Implements the interceptor missile with PN guidance
   - Uses a navigation constant (N') to scale the guidance commands
   - Applies acceleration constraints (max acceleration and jerk limits)
   - Uses 4th order Runge-Kutta integration for accurate physics

2. **Target**: Represents the target being intercepted
   - Can implement various motion profiles (constant velocity, maneuvering)
   - Position and velocity are updated using physics integration

3. **Simulation Scenarios**: Multiple scenarios to test PN guidance
   - Head-on: Interceptor and target moving directly toward each other
   - Crossing Paths: Interceptor and target on perpendicular trajectories
   - Pursuit: Interceptor chasing a target moving away
   - High-Speed: Interceptor attempting to catch a fast-moving target
   - Maneuvering: Interceptor tracking a target with changing velocity

### Physics

The simulation uses 4th order Runge-Kutta integration to simulate accurate physics. This method provides good precision for numerical integration of the equations of motion, considering both position and velocity.

### Guidance Law

The Proportional Navigation guidance law is implemented as:

```
a_c = N' × λ_dot × V_c
```

Where:
- a_c: Commanded acceleration (perpendicular to the line of sight)
- N': Navigation constant (typically 3-5)
- λ_dot: Line of sight rate
- V_c: Closing velocity

## Neural Network Guidance

In addition to the classic PN guidance, the simulation includes a neural network-based guidance system. The neural network is trained to predict optimal guidance commands based on the relative positions and velocities of the interceptor and target.

## Usage

### Running Scenarios

1. Use `run_interceptor_test.bat` to run through all test scenarios
2. Use `run_interceptor.bat` to run the neural network guided interceptor
3. Use `run_prototype.bat` to run a simplified prototype implementation

### Tuning Parameters

The following parameters can be adjusted in `src/missile_interceptor.cpp`:

- `nav_constant_`: Navigation constant for PN guidance (default: 3.0)
- `max_interceptor_accel_`: Maximum acceleration constraint (default: 50.0 units/s²)
- `max_interceptor_jerk_`: Maximum jerk constraint (default: 20.0 units/s³)
- `time_step_`: Simulation time step (default: 0.1 seconds)
- `intercept_radius_`: Success radius for interception (default: 2.0 units)

## Visualization

The simulation provides real-time visualization in the console window, showing:
- Current positions of interceptor and target
- Velocity vectors
- Acceleration vectors
- Line of sight
- Interception statistics (miss distance, time-to-intercept)

## Future Improvements

Potential enhancements to the simulation include:
- Graphical visualization using a proper rendering library
- Advanced guidance laws (Augmented PN, Optimal guidance)
- More complex target maneuvers
- 3D simulation environment 