# Contributing to RobotSim2

Thank you for your interest in contributing to RobotSim2! This document provides guidelines and instructions for contributing to the project.

## 🚀 Getting Started

### Prerequisites

- Python 3.9 or higher
- Git
- Basic understanding of robotics kinematics (helpful but not required)
- OpenGL-capable display for running the interactive viewer

### Development Setup

1. **Clone the repository**
   ```bash
   git clone https://github.com/dexmac221/RobotSim2.git
   cd RobotSim2
   ```

2. **Create a virtual environment**
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install in development mode with dev dependencies**
   ```bash
   pip install -e .[dev]
   ```

4. **Verify installation**
   ```bash
   pytest
   robot-sim-demo --duration 2
   ```

## 🧪 Running Tests

Before submitting any changes, ensure all tests pass:

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=robot_sim_py --cov-report=html

# Run specific test file
pytest tests/test_kinematics.py

# Run with verbose output
pytest -v
```

All tests must pass before a PR can be merged.

## 🎨 Code Style

We follow standard Python conventions:

### Formatting
- Use **Black** for code formatting (line length: 100)
  ```bash
  black robot_sim_py/ tests/
  ```

### Linting
- Use **Ruff** for fast linting
  ```bash
  ruff check robot_sim_py/ tests/
  ```

### Type Checking
- Use **mypy** for static type checking
  ```bash
  mypy robot_sim_py/
  ```

### General Guidelines
- Use type hints for all function signatures
- Write docstrings for public APIs
- Keep functions focused and under ~50 lines when possible
- Use descriptive variable names
- Prefer explicit over implicit

## 📝 Commit Messages

Use clear, descriptive commit messages:

```
Add orientation search for edge workspace targets

- Implement find_feasible_orientation() helper
- Add yaw/pitch candidate generation
- Update tests to cover new reachability cases
```

Format: `<type>: <short description>`

Types:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `test`: Test additions or modifications
- `refactor`: Code refactoring
- `perf`: Performance improvements
- `chore`: Maintenance tasks

## 🔧 Pull Request Process

1. **Create a feature branch**
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. **Make your changes**
   - Write clean, well-documented code
   - Add tests for new functionality
   - Update documentation as needed

3. **Run quality checks**
   ```bash
   pytest
   black robot_sim_py/ tests/
   ruff check robot_sim_py/ tests/
   mypy robot_sim_py/
   ```

4. **Commit your changes**
   ```bash
   git add .
   git commit -m "feat: description of your changes"
   ```

5. **Push to your fork**
   ```bash
   git push origin feature/your-feature-name
   ```

6. **Open a Pull Request**
   - Provide a clear description of changes
   - Reference any related issues
   - Explain motivation and context

## 🐛 Reporting Bugs

When reporting bugs, please include:

1. **Description**: Clear description of the bug
2. **Steps to Reproduce**: Minimal steps to reproduce the issue
3. **Expected Behavior**: What you expected to happen
4. **Actual Behavior**: What actually happened
5. **Environment**:
   - Python version
   - Operating system
   - Relevant package versions
6. **Additional Context**: Screenshots, logs, etc.

## 💡 Suggesting Enhancements

We welcome enhancement suggestions! Please:

1. Check existing issues to avoid duplicates
2. Clearly describe the enhancement
3. Explain the use case and motivation
4. Provide examples if applicable

## 🏗️ Project Structure

```
robot_sim_py/
├── __init__.py          # Package initialization
├── app.py               # CLI demo entry point
├── ogl_app.py          # OpenGL interactive viewer
├── robot.py            # Core kinematics (FK, IK, Jacobians)
├── controllers.py      # Manual and trajectory controllers
├── trajectory.py       # Trajectory interpolation
├── gemini_agent.py     # Gemini Robotics integration
├── camera.py           # Camera projection utilities
├── scene.py            # Scene constants and workspace bounds
├── reachability.py     # Orientation search helpers
├── viewer.py           # Matplotlib visualization
└── gemini_image_test.py # Standalone Gemini testing tool

tests/
├── test_kinematics.py               # FK/IK/Jacobian tests
├── test_gemini_agent.py             # Gemini integration tests
├── test_gemini_robotics_prompt.py   # Live API tests
├── test_gemini_api_key.py          # API key validation
└── test_free_wrist_reachability.py # Orientation search tests
```

## 🎯 Areas for Contribution

We're particularly interested in contributions in these areas:

### High Priority
- **Physics integration**: Add collision detection, dynamics simulation
- **Additional robots**: Support for other arm configurations
- **Performance**: Optimize IK solver, rendering pipeline
- **Documentation**: Tutorials, examples, API docs

### Medium Priority
- **WebGL viewer**: Browser-based visualization
- **Trajectory primitives**: More motion types (circular, spline, etc.)
- **Dataset generation**: Tools for creating training data
- **VLA integration**: Examples with visual-language-action models

### Low Priority
- **Mobile support**: Touch controls for viewer
- **Multi-arm coordination**: Support multiple robots
- **ROS integration**: ROS2 bridge
- **Additional sensors**: Virtual sensors (force, proximity)

## 🧩 Code Review Guidelines

When reviewing PRs, we look for:

- ✅ Code correctness and functionality
- ✅ Test coverage for new code
- ✅ Clear, maintainable code style
- ✅ Appropriate documentation
- ✅ No breaking changes (or justified breaking changes)
- ✅ Performance considerations

## 📜 License

By contributing, you agree that your contributions will be licensed under the Apache License 2.0.

## 🤝 Code of Conduct

- Be respectful and inclusive
- Focus on constructive feedback
- Help others learn and grow
- Assume good intentions

## ❓ Questions?

If you have questions:
- Open a GitHub Discussion
- Comment on relevant issues
- Reach out via GitHub

---

Thank you for contributing to RobotSim2! 🤖✨
