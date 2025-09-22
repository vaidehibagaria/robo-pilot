# Changelog

All notable changes to the RCM Robot Control Framework will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Docker containerization for easy deployment
- GitHub Actions CI/CD pipeline
- Comprehensive documentation and examples
- Support for multiple robot types (TurtleBot, UR5, Panda)
- Real-time position tracking and path visualization
- Interactive command-line interface
- REST API endpoints for programmatic access

## [1.0.0] - 2024-01-XX

### Added
- Initial release of RCM Robot Control Framework
- Robot Capability Manifest (RCM) system for safe robot control
- GPT integration for natural language robot commands
- Dynamic tool generation based on robot capabilities
- Safety constraint enforcement (joint limits, workspace bounds)
- Multi-robot support with RCM server
- URDF to RCM conversion tool
- Real-time coordinate tracking and path visualization
- Interactive commands (position, path, reset)
- Fallback support for demo mode without GPT
- Comprehensive error handling and logging

### Features
- **Natural Language Control**: Control robots using plain English commands
- **Safety First**: Built-in constraint enforcement prevents dangerous commands
- **No-Code Setup**: Works with any robot via Docker containerization
- **Multi-Model Support**: Compatible with GPT-4, Claude, and other OpenAI-compatible APIs
- **Real-time Tracking**: Live position monitoring and movement history
- **Extensible**: Easy to add new robot types and capabilities

### Supported Robots
- TurtleBot3 (Burger, Waffle, Waffle Pi)
- UR5/UR10 robotic arms
- Panda robotic arm
- Custom robots via URDF conversion

### API Endpoints
- `GET /robots` - List available robots
- `GET /robots/{robot_id}/rcm` - Get complete RCM
- `GET /robots/{robot_id}/capabilities` - Get robot capabilities
- `GET /robots/{robot_id}/constraints` - Get safety constraints
- `POST /robots/{robot_id}/rcm` - Upload RCM

### Docker Support
- Complete containerization with all dependencies
- One-command deployment with `docker-compose up`
- Automatic RCM generation and setup
- Environment variable configuration

### Documentation
- Comprehensive README with quick start guide
- API documentation with examples
- Contributing guidelines
- MIT License

## [0.9.0] - 2024-01-XX (Pre-release)

### Added
- Core RCM framework implementation
- Basic GPT integration
- Tool generator with state tracking
- Agent bridge for LLM communication
- RCM server with FastAPI
- URDF to RCM conversion
- Basic robot state tracking

### Changed
- Improved error handling and logging
- Enhanced coordinate tracking accuracy
- Better path visualization

### Fixed
- Fixed coordinate tracking bugs
- Resolved tool generation issues
- Improved error messages

## [0.8.0] - 2024-01-XX (Alpha)

### Added
- Initial prototype implementation
- Basic RCM structure
- Simple tool generation
- Mock robot control

### Known Issues
- Limited robot support
- Basic error handling
- No Docker support
- Limited documentation

---

## Version History

- **1.0.0**: First stable release with full Docker support
- **0.9.0**: Pre-release with core functionality
- **0.8.0**: Alpha release with basic features

## Migration Guide

### From 0.9.0 to 1.0.0

1. **Docker Support**: The framework now supports Docker deployment. Use `docker-compose up` for easy setup.

2. **Environment Variables**: Configuration is now handled via `.env` file or environment variables.

3. **API Changes**: The RCM server API has been updated with new endpoints.

4. **New Dependencies**: Additional dependencies have been added for Docker support.

### From 0.8.0 to 0.9.0

1. **RCM Structure**: The RCM JSON structure has been updated with new fields.

2. **Tool Generation**: The tool generation system has been completely rewritten.

3. **State Tracking**: New coordinate tracking system has been added.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on contributing to this project.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

