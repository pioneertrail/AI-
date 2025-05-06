# AI++ Project

This project aims to build simple Machine Learning components (like a Neural Network) from scratch in C++17 to understand the fundamentals.

## Current Features

### 2D Dodger Game

A simple console-based game where the Player ('P') tries to reach the Target ('T') on the right side of a 2D grid while avoiding the Enemy ('E').

The Enemy AI uses a simple neural network trained to intercept the player.

### Missile Interceptor Simulation

A physics-based simulation of missile interception scenarios using Proportional Navigation (PN) guidance.

Key features:
- Realistic physics using 4th order Runge-Kutta integration
- Multiple test scenarios (head-on, crossing paths, pursuit, high-speed, maneuvering)
- Proportional Navigation guidance implementation with acceleration and jerk constraints
- Real-time visualization of interception attempts

#### Running the Simulation

You can use the following batch files to build and run different aspects of the simulation:

```bash
# Build and run the main missile simulation
.\compile_vs.bat

# Run the interceptor test scenarios
.\run_interceptor_test.bat

# Run the interceptor with trained neural network guidance
.\run_interceptor.bat

# Run the prototype interceptor implementation
.\run_prototype.bat
```

#### Simulation Scenarios

The simulation includes several test scenarios:
- **Head-on Collision**: Interceptor and target moving directly toward each other
- **Crossing Paths**: Interceptor and target moving on perpendicular trajectories
- **Pursuit**: Interceptor chasing a target moving away
- **High-Speed Target**: Interceptor attempting to catch a fast-moving target
- **Maneuvering Target**: Interceptor tracking a target with changing velocity

Use keyboard input to switch between scenarios during simulation.

For detailed documentation on the missile interceptor implementation, see [docs/MISSILE_INTERCEPTOR.md](docs/MISSILE_INTERCEPTOR.md).

## Building

Use the Visual Studio Developer Command Prompt or ensure `cl.exe` is in your path.

### Main Application

Run the compile script: `.\compile_vs.bat`

This script compiles all necessary `.cpp` files and creates `ml_app.exe`.

### Missile Simulation

Run the missile simulation compile script: `.\compile_missile_sim.bat`

This builds the missile interceptor simulation and creates `missile_sim.exe`.

## Running

The main application has two modes:

1.  **Training Mode:** Trains the AI model and saves it to `dodger_model_2d.bin`.
    ```bash
    .\ml_app.exe train
    ```
    *Note: The `compile_vs.bat` script is currently configured to automatically run this training step after a successful compilation.*

2.  **Gameplay Mode:** Loads the pre-trained `dodger_model_2d.bin` and starts the interactive game.
    ```bash
    .\ml_app.exe
    ```

## Development

See `DEVELOPMENT_GUIDELINES.md` for coding standards. 