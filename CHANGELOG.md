# Changelog

All notable changes to this project will be documented in this file.

## [1.0.0] - 2023-12-18

### Added
- Missile Interceptor Simulation with Proportional Navigation guidance
  - Implemented in a modular design with proper header/source separation
  - Added multiple interception test scenarios (head-on, crossing paths, pursuit, etc.)
  - Added realistic physics using 4th order Runge-Kutta integration
  - Added compile_missile_sim.bat for building the simulation

### Changed
- Updated README.md with information about the missile interceptor simulation
- Improved code documentation with detailed comments and docstrings

### Removed
- Removed monolithic test_scenarios.cpp in favor of modular design

## [Unreleased] 