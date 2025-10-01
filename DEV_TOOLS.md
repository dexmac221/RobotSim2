# Quick Reference: Development Tools Setup

This guide helps you get started with the new development tools added to RobotSim2.

## 🔧 Installation

Install all development dependencies:

```bash
pip install -e .[dev]
```

This installs:
- `pytest` - Testing framework
- `black` - Code formatter
- `mypy` - Static type checker
- `ruff` - Fast linter

## 🎨 Code Formatting with Black

Black automatically formats your Python code to a consistent style.

```bash
# Format all files in place
black robot_sim_py/ tests/

# Check what would be changed (dry run)
black --check robot_sim_py/ tests/

# Show diffs without making changes
black --diff robot_sim_py/ tests/

# Format a single file
black robot_sim_py/robot.py
```

**Configuration**: See `[tool.black]` in `pyproject.toml`
- Line length: 100 characters
- Target: Python 3.9+

## 🔍 Linting with Ruff

Ruff catches common errors and code smells very quickly.

```bash
# Check for issues
ruff check robot_sim_py/ tests/

# Auto-fix issues where possible
ruff check --fix robot_sim_py/ tests/

# Check a single file
ruff check robot_sim_py/robot.py

# See all available rules
ruff linter
```

**Configuration**: See `[tool.ruff]` in `pyproject.toml`
- Checks: pycodestyle, pyflakes, isort, flake8-bugbear, pyupgrade
- Line length: 100 characters

## 🔬 Type Checking with mypy

Mypy validates type hints and catches type-related bugs.

```bash
# Check the entire package
mypy robot_sim_py/

# Check specific files
mypy robot_sim_py/robot.py robot_sim_py/controllers.py

# More strict checking
mypy --strict robot_sim_py/
```

**Configuration**: See `[tool.mypy]` in `pyproject.toml`
- Checks untyped definitions
- Warns on redundant casts and unused ignores
- Ignores missing imports for external libraries (OpenGL, etc.)

## 🧪 Running Tests

```bash
# Run all tests
pytest

# Run with verbose output
pytest -v

# Run with coverage report
pytest --cov=robot_sim_py --cov-report=html

# Run specific test file
pytest tests/test_kinematics.py

# Run tests matching a pattern
pytest -k "gemini"
```

## 🔄 Complete Quality Check Workflow

Run all checks before committing:

```bash
# 1. Format code
black robot_sim_py/ tests/

# 2. Run linter
ruff check --fix robot_sim_py/ tests/

# 3. Type check
mypy robot_sim_py/

# 4. Run tests
pytest

# 5. Check syntax
python -m compileall robot_sim_py/ tests/
```

## 📝 Pre-commit Hook (Optional)

Create `.git/hooks/pre-commit`:

```bash
#!/bin/bash
# Run quality checks before commit

echo "Running Black..."
black --check robot_sim_py/ tests/ || exit 1

echo "Running Ruff..."
ruff check robot_sim_py/ tests/ || exit 1

echo "Running tests..."
pytest || exit 1

echo "✓ All checks passed!"
```

Make it executable:
```bash
chmod +x .git/hooks/pre-commit
```

## 🚀 VS Code Integration (Optional)

Add to `.vscode/settings.json`:

```json
{
  "python.formatting.provider": "black",
  "python.linting.enabled": true,
  "python.linting.ruffEnabled": true,
  "python.linting.mypyEnabled": true,
  "editor.formatOnSave": true,
  "editor.codeActionsOnSave": {
    "source.organizeImports": true
  }
}
```

## 📦 CI/CD

The GitHub Actions workflow (`.github/workflows/ci.yml`) automatically runs:
- Tests on Python 3.9, 3.10, 3.11
- Tests on Ubuntu, macOS, Windows
- Black formatting check
- Ruff linting
- mypy type checking
- Syntax compilation

No setup needed - it runs automatically on push/PR!

## 🆘 Troubleshooting

### Black and Ruff disagree on formatting
This shouldn't happen with the current config, but if it does:
```bash
black robot_sim_py/ tests/  # Black has final say
```

### mypy reports errors in external libraries
They're already configured to be ignored. If you see new ones:
```python
# Add to pyproject.toml [tool.mypy] overrides
[[tool.mypy.overrides]]
module = ["problematic_module.*"]
ignore_missing_imports = true
```

### Tests fail after formatting
This shouldn't happen. If it does:
```bash
git diff  # See what changed
git restore <file>  # Restore if needed
```

## 📚 More Information

- Black: https://black.readthedocs.io/
- Ruff: https://docs.astral.sh/ruff/
- mypy: https://mypy.readthedocs.io/
- pytest: https://docs.pytest.org/

---

**Pro Tip**: Run `black` first, then `ruff --fix`, then `mypy`. This order minimizes conflicts.
