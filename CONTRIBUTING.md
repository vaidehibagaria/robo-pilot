# Contributing to RCM Robot Control Framework

Thank you for your interest in contributing to the RCM Robot Control Framework! This document provides guidelines and information for contributors.

## 🤝 How to Contribute

### Reporting Issues

- Use the GitHub issue tracker to report bugs or request features
- Include detailed information about your environment and the issue
- For bugs, include steps to reproduce the problem

### Suggesting Enhancements

- Open an issue with the "enhancement" label
- Describe the proposed feature and its benefits
- Consider the impact on existing functionality

### Code Contributions

1. **Fork the repository**
2. **Create a feature branch**:
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. **Make your changes** following our coding standards
4. **Add tests** for new functionality
5. **Update documentation** as needed
6. **Commit your changes**:
   ```bash
   git commit -m "Add: brief description of changes"
   ```
7. **Push to your fork**:
   ```bash
   git push origin feature/your-feature-name
   ```
8. **Create a Pull Request**

## 📋 Development Setup

### Prerequisites

- Python 3.8+
- Docker (for containerized development)
- Git

### Local Development

```bash
# Clone your fork
git clone https://github.com/yourusername/rcm-robot-control.git
cd rcm-robot-control

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Install development dependencies
pip install -r requirements-dev.txt

# Run tests
python -m pytest tests/

# Run linting
flake8 src/
black src/
```

### Docker Development

```bash
# Build development image
docker-compose -f docker-compose.dev.yml up --build

# Run tests in container
docker-compose exec rcm-framework python -m pytest tests/
```

## 📝 Coding Standards

### Python Code

- Follow PEP 8 style guidelines
- Use type hints where appropriate
- Write docstrings for all public functions and classes
- Keep functions focused and small
- Use meaningful variable and function names

### Example:

```python
def process_robot_command(command: str, robot_id: str) -> Dict[str, Any]:
    """
    Process a natural language command for a specific robot.
    
    Args:
        command: Natural language command string
        robot_id: Identifier for the target robot
        
    Returns:
        Dictionary containing execution result and status
        
    Raises:
        ValueError: If command is invalid or robot not found
    """
    # Implementation here
    pass
```

### Commit Messages

Use conventional commit format:

- `feat:` New features
- `fix:` Bug fixes
- `docs:` Documentation changes
- `style:` Code style changes
- `refactor:` Code refactoring
- `test:` Adding or updating tests
- `chore:` Maintenance tasks

Examples:
```
feat: add support for custom robot URDFs
fix: resolve coordinate tracking bug in path visualization
docs: update API documentation for new endpoints
```

## 🧪 Testing

### Running Tests

```bash
# Run all tests
python -m pytest

# Run specific test file
python -m pytest tests/test_tool_generator.py

# Run with coverage
python -m pytest --cov=src tests/

# Run integration tests
python -m pytest tests/integration/
```

### Writing Tests

- Write unit tests for individual functions
- Write integration tests for component interactions
- Aim for high test coverage (>80%)
- Use descriptive test names
- Test both success and failure cases

### Example Test:

```python
def test_drive_straight_command():
    """Test that drive_straight command is processed correctly."""
    # Arrange
    command = "move forward 2 meters"
    expected_distance = 2.0
    
    # Act
    result = process_command(command)
    
    # Assert
    assert result["action"] == "drive_straight"
    assert result["distance"] == expected_distance
    assert result["success"] is True
```

## 📚 Documentation

### Code Documentation

- Write clear docstrings for all public APIs
- Include examples in docstrings where helpful
- Keep README.md updated with new features
- Update API documentation for new endpoints

### User Documentation

- Update user guides for new features
- Add troubleshooting information
- Include example use cases
- Keep installation instructions current

## 🚀 Release Process

### Version Numbering

We use [Semantic Versioning](https://semver.org/):
- `MAJOR.MINOR.PATCH` (e.g., 1.2.3)
- MAJOR: Breaking changes
- MINOR: New features (backward compatible)
- PATCH: Bug fixes (backward compatible)

### Release Checklist

- [ ] All tests passing
- [ ] Documentation updated
- [ ] Version number updated
- [ ] CHANGELOG.md updated
- [ ] Release notes prepared

## 🐛 Bug Reports

When reporting bugs, please include:

1. **Environment information**:
   - OS and version
   - Python version
   - Docker version (if applicable)

2. **Steps to reproduce**:
   - Clear, numbered steps
   - Expected vs actual behavior
   - Error messages or logs

3. **Additional context**:
   - Screenshots if applicable
   - Related issues
   - Workarounds if any

## 💡 Feature Requests

When suggesting features:

1. **Describe the problem** you're trying to solve
2. **Explain your proposed solution**
3. **Consider alternatives** you've thought about
4. **Describe the impact** on existing users

## 📞 Getting Help

- 💬 [GitHub Discussions](https://github.com/yourusername/rcm-robot-control/discussions)
- 🐛 [GitHub Issues](https://github.com/yourusername/rcm-robot-control/issues)
- 📧 Email: [your-email@example.com]

## 🙏 Recognition

Contributors will be recognized in:
- CONTRIBUTORS.md file
- Release notes
- Project documentation

Thank you for contributing to the RCM Robot Control Framework! 🚀

