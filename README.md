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

To run the simulation:
```bash
.\compile_missile_sim.bat
```

This builds and runs `missile_sim.exe` using the Visual Studio compiler.

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