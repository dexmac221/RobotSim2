# Legacy Python Implementation

⚠️ **This directory contains an earlier iteration of RobotSim2.**

## Status: Archived

This is the initial Python reimplementation that was created during the early stages of the project. It has been superseded by the main `robot_sim_py/` package, which includes:

- Gemini Robotics integration
- OpenGL-based interactive viewer
- Configurable DOF (1-6 joints)
- Advanced camera projection
- Comprehensive test suite

## Differences from Main Package

This legacy version:
- Uses Denavit-Hartenberg parameters exclusively
- Has matplotlib-based visualization only
- Lacks AI/Gemini integration
- Has simpler control schemes
- Is not actively maintained

## For Active Development

Please use the main package at the repository root:
- **Package**: `/robot_sim_py/`
- **Tests**: `/tests/`
- **Documentation**: `/README.md`

## Preservation Reason

This directory is preserved for:
1. Historical reference
2. Comparison of different kinematic approaches (DH vs. explicit transforms)
3. Alternative matplotlib-based visualization examples

If you need the functionality from this legacy version, consider porting specific features to the main package.
