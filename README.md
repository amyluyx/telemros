# UWFSAE Jetson ROS2 Stack

Driverless Formula Student autonomous stack running ROS 2 Jazzy on NVIDIA Jetson Orin AGX.

## Overview

This repository provides a Docker-based development workflow for building and testing driverless vehicle software. The same codebase runs on both local development machines and the NVIDIA Jetson Orin AGX deployment target.

**Key Features:**
- Containerized development with Docker 🐳
- ROS 2 Jazzy with pre-configured environment
- Identical environments across development and deployment

## Getting Started

**New to the project?** → See [**docs/SETUP.md**](docs/SETUP.md) for complete setup instructions.

**Quick reference:**
```bash
# Build container
cd docker && docker build -t jetson-ros2:latest -f Dockerfile ..

# Start container (development machines)
docker-compose up -d

# Enter container
docker-compose exec driverless bash

# Build workspace (inside container)
cd /workspace/ros2_ws && colcon build && source install/setup.bash

# For Jetson with GPU:
# docker-compose -f docker-compose.yml -f docker-compose.jetson.yml up -d
```

## (Temporary) Project Structure
```
jetson-ros2/
├── docker/                    # Docker configuration
│   ├── Dockerfile             # ROS 2 Jazzy container
│   └── docker-compose.yml     # Compose configuration
├── ros2_ws/                   # ROS 2 workspace
│   ├── src/
│   │   └── demo_pkg/          # Demo package
│   │       ├── demo_pkg/      # Python package
│   │       │   ├── demo_orchestrator_node.py
│   │       │   ├── demo_perception_node.py
│   │       │   ├── demo_planning_node.py
│   │       │   └── demo_control_node.py
│   │       ├── package.xml    # ROS 2 package manifest
│   │       ├── setup.py       # Python package setup
│   │       └── test/          # Unit tests
│   ├── build/                 # Build artifacts (gitignored)
│   ├── install/               # Install space (gitignored)
│   └── log/                   # Build logs (gitignored)
├── docs/                      # Documentation
│   └── SETUP.md               # Setup instructions
├── scripts/                   # Utility scripts
└── README.md                  # This file
```

## Demo Architecture

The autonomous system follows a standard robotics pipeline:

```
┌─────────────────┐
│  Orchestrator   │  ← Coordinates system state
└────────┬────────┘
         │
    ┌────┴────┬─────────┬─────────┐
    │         │         │         │
┌───▼────┐ ┌─▼──────┐ ┌▼──────┐ ┌▼────────┐
│Perception│ │Planning│ │Control│ │ (More) │
└──────────┘ └────────┘ └───────┘ └─────────┘
```

**Current Demo Nodes:**
- `demo_orchestrator_node` - System coordinator and state machine
- `demo_perception_node` - Sensor data processing (simulated)
- `demo_planning_node` - Path planning and decision making
- `demo_control_node` - Vehicle control commands

All nodes communicate via ROS 2 topics and follow the Jazzy API.

## Development Workflow

1. **Edit code** on your local machine in `ros2_ws/src/`
2. **Rebuild** inside container: `colcon build --packages-select <pkg>`
3. **Test immediately** - changes are live via volume mounting

## Docker Environment

The same Dockerfile builds identical environments for development and Jetson deployment - same OS, ROS 2, Python, and dependencies. On Jetson, we add GPU runtime and hardware access via an override file.

**Development (Mac/Windows/Linux):**
```bash
cd docker
docker-compose up -d
docker-compose exec driverless bash
```

**Jetson (adds GPU/hardware):**
```bash
cd docker
docker-compose -f docker-compose.yml -f docker-compose.jetson.yml up -d
docker-compose exec driverless bash
```

Use `docker-compose` to manage containers - it allows multiple terminals to enter the same container, essential for running multiple nodes.

See [docs/SETUP.md](docs/SETUP.md) for complete instructions.
