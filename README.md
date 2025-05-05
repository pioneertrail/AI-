# AI++ Project

This project aims to build simple Machine Learning components (like a Neural Network) from scratch in C++17 to understand the fundamentals.

## Current Feature: 2D Dodger Game

A simple console-based game where the Player ('P') tries to reach the Target ('T') on the right side of a 2D grid while avoiding the Enemy ('E').

The Enemy AI uses a simple neural network trained to intercept the player.

## Building

Use the Visual Studio Developer Command Prompt or ensure `cl.exe` is in your path.

1.  Run the compile script: `.\compile_vs.bat`

This script compiles all necessary `.cpp` files and creates `ml_app.exe`.

## Running

The application has two modes:

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