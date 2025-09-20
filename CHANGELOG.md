# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Initial release of ros2_tui
- ROS2 topics monitoring with real-time Hz/Delay measurements  
- ROS2 parameters management with set/dump/load operations
- Terminal User Interface with keyboard navigation
- Hierarchical organization of topics and parameters
- Search and filtering capabilities
- Smart parameter type validation and conversion
- Auto-expiring status messages (15 seconds)
- Uniform popup components for consistent UI
- Abstract tree structure for extensibility
- VS Code debug configuration
- GitHub Actions CI/CD workflows
- PyPI and crates.io publishing setup

### Features

#### Topics Monitor (`ros2-topics`)
- Real-time topic monitoring with continuous streaming
- Interactive keyboard navigation
- Expandable/collapsible topic groups
- Search mode with dynamic filtering
- Loading animations and status indicators
- Optimized performance with shared Tokio runtime

#### Parameters Manager (`ros2-params`)
- Complete parameter management (view/set/dump/load)
- YAML-based bulk operations for entire nodes
- Smart type validation with ROS2 feedback
- Auto-formatting of double parameters with decimal points
- Warning popups for invalid operations
- Cursor-enabled text input with highlighting

## [0.1.0] - 2024-XX-XX

### Added
- Initial public release