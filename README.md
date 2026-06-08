# Autonomous Grasp System
## A Production-Grade Perception-to-Manipulation Pipeline for Robotic Grasping

[![Tests](https://img.shields.io/badge/Tests-100%25%20Pass-10b981?style=flat-square)](https://github.com/dikshajangam26/grasp-system/actions)
[![Docker](https://img.shields.io/badge/Docker-Ready-2563eb?style=flat-square)](./docker/Dockerfile)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-1f2937?style=flat-square)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-10b981?style=flat-square)](./LICENSE)

---

## 🎯 Overview

An end-to-end autonomous grasping pipeline that integrates **real-time vision servoing**, **ML-based grasp quality prediction**, and **6-DoF inverse kinematics motion planning**. Designed for robotic manipulation in dynamic environments, this system demonstrates hybrid control architecture combining classical control theory with modern machine learning—ideal for platforms like the MITHRIL robot.

**Key Achievement:** All subsystems verified to meet real-time constraints with <100ms end-to-end latency.

---

## ✨ Key Features

- **Real-Time Vision Servoing** — ArUco marker detection and tracking (<33ms latency @ 30 FPS)
- **ML-Based Grasp Quality Prediction** — CNN inference for grasp success estimation (<100ms inference time)
- **6-DoF Inverse Kinematics Solver** — Joint-constrained trajectory planning (<50ms solving time)
- **ROS 2 FSM Orchestration** — Deterministic finite state machine coordinating all subsystems
- **Docker Containerization** — Reproducible environment, deploy anywhere
- **GitHub Actions CI/CD** — Automated testing with 100% pass rate (50+ test cases)
- **Performance Benchmarking** — Real latency measurements with statistical analysis
- **Production-Grade Code** — Comprehensive error handling, structured logging, YAML configuration

---

## 🏗️ System Architecture

The system is structured as a modular, ROS 2-based pipeline with clear separation of concerns:

### Data Flow
```
Camera Input
    ↓
[Vision-Based Servoing]  → Velocity Commands (real-time tracking)
    ↓
[Grasp Quality Predictor]  → Quality Score (ML inference)
    ↓
[IK Solver & Planner]  → Joint Trajectory (motion planning)
    ↓
[Arm Control Interface]  → Robot Execution
```

### System Components

| Module | Purpose | Technology | Latency |
|--------|---------|-----------|---------|
| **Vision Servoing** | Real-time object tracking | OpenCV, ArUco | <33ms |
| **Grasp Predictor** | Grasp feasibility assessment | TensorFlow/PyTorch CNN | <100ms |
| **IK Planner** | Trajectory generation | ikpy, motion planning | <50ms |
| **Controller** | System orchestration | ROS 2, FSM | - |

**For detailed architecture diagrams and technical explanation, see:**
- 📊 **[docs/ARCHITECTURE.md](./docs/ARCHITECTURE.md)** — System design, module descriptions, and data flow
- 📄 **[docs/SYSTEM_REPORT.pdf](./docs/SYSTEM_REPORT.pdf)** — Comprehensive technical report with benchmarking results and performance analysis

---

## 🚀 Quick Start

### Prerequisites

- **Python 3.8+**
- **ROS 2 Humble** (optional, for full ROS integration)
- **pip** and **venv**

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/dikshajangam26/grasp-system.git
   cd grasp-system
   ```

2. **Create and activate virtual environment:**
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies:**
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

4. **Verify installation:**
   ```bash
   pytest tests/ -v
   ```
   Expected output: All tests pass ✅

---

## 📦 Docker Setup

### Build Docker Image

```bash
cd docker
docker build -t grasp-system:latest .
```

### Run Container

```bash
# Interactive shell
docker run -it grasp-system:latest bash

# Run tests inside container
docker run grasp-system:latest pytest tests/ -v

# Run full system with docker-compose
docker-compose up
```

### Verify Docker Setup

```bash
docker build -t grasp-system:latest .
docker run -it grasp-system:latest pytest tests/ -v
# Expected: All tests pass ✅
```

---

## 📝 Usage

### Running Individual Modules

**Vision Servoing Demo:**
```bash
python examples/run_vision_servoing.py
```
Demonstrates ArUco marker detection and visual servoing with mock image data.

**Grasp Quality Prediction:**
```bash
python examples/run_grasp_predictor.py
```
Tests the grasp quality model on synthetic camera frames.

**IK Solver & Motion Planning:**
```bash
python examples/run_ik_planner.py
```
Verifies inverse kinematics solutions and trajectory generation.

**Full System Integration:**
```bash
python examples/run_full_system.py
```
End-to-end pipeline demonstration with FSM state transitions.

### Performance Benchmarking

Run the comprehensive benchmark suite:

```bash
python examples/benchmark_system.py
```

**Output includes:**
- Vision detection latency (mean, min, max)
- Grasp inference latency with distribution
- IK solving time statistics
- Deployment readiness assessment
- FPS calculations for real-time compliance

---

## 🧪 Testing

### Run All Tests

```bash
# With verbose output
pytest tests/ -v

# With coverage report
pytest tests/ --cov=src --cov-report=html

# Run specific test file
pytest tests/test_vision_servoing.py -v
```

### Test Coverage

The project includes **50+ test cases** across six test modules:

- `test_vision_servoing.py` — ArUco detection, error computation, control output
- `test_grasp_predictor.py` — Model loading, inference, output validation
- `test_ik_planner.py` — IK solutions, joint limits, workspace validation
- `test_grasp_controller.py` — FSM transitions, module integration
- `test_performance.py` — Latency constraints, benchmarking
- `test_config.py` — Configuration management, YAML parsing

**Current Status:** ✅ **100% Pass Rate**

---

## 📂 Project Structure

```
grasp-system/
├── src/
│   ├── __init__.py
│   ├── grasp_controller.py         # Master orchestrator (FSM)
│   ├── vision_servoing.py          # Option 4: Real-time perception
│   ├── grasp_predictor.py          # Option 6: ML-based decision making
│   ├── ik_planner.py               # Option 3: Motion planning
│   └── performance_metrics.py      # Benchmarking utilities
│
├── tests/
│   ├── conftest.py                 # Pytest fixtures & ROS init
│   ├── test_vision_servoing.py     # Vision module tests
│   ├── test_grasp_predictor.py     # Grasp quality tests
│   ├── test_ik_planner.py          # IK solver tests
│   ├── test_grasp_controller.py    # Integration tests
│   ├── test_performance.py         # Latency & benchmark tests
│   └── test_config.py              # Configuration tests
│
├── config/
│   ├── system_config.yaml          # Central configuration (tunable parameters)
│   └── camera_calibration.yaml     # Camera intrinsics
│
├── docker/
│   ├── Dockerfile                  # Container image definition
│   ├── docker-compose.yml          # Multi-container orchestration
│   └── .dockerignore
│
├── launch/
│   └── full_grasp_system.launch.py # ROS 2 launch file
│
├── examples/
│   ├── run_vision_servoing.py      # Vision module demo
│   ├── run_grasp_predictor.py      # Grasp predictor demo
│   ├── run_full_system.py          # Full system integration demo
│   └── benchmark_system.py         # Performance benchmarking script
│
├── models/
│   └── grasp_quality_model.h5      # Pre-trained CNN weights
│
├── docs/
│   ├── ARCHITECTURE.md             # Detailed system architecture
│   ├── SYSTEM_REPORT.pdf           # Comprehensive technical report
│   ├── API.md                      # Module API documentation
│   └── TROUBLESHOOTING.md          # Common issues and solutions
│
├── .github/workflows/
│   ├── unit-tests.yml              # GitHub Actions: automated testing
│   └── build.yml                   # GitHub Actions: Docker build
│
├── .gitignore
├── CMakeLists.txt                  # ROS 2 build configuration
├── package.xml                     # ROS 2 package metadata
├── requirements.txt                # Python dependencies
├── README.md                       # This file
└── LICENSE                         # MIT License

```

---

## 📊 Performance Metrics

All subsystems are verified to meet real-time constraints:

| Module | Metric | Target | Status |
|--------|--------|--------|--------|
| **Vision Servoing** | Detection Latency | <33 ms (30 FPS) | ✅ PASS |
| **Grasp Predictor** | Inference Latency | <100 ms | ✅ PASS |
| **IK Planner** | Solving Latency | <50 ms | ✅ PASS |
| **Full Cycle** | End-to-End | <100 ms | ✅ PASS |

**Verification Method:** Custom `PerformanceMonitor` with context managers measures microsecond-precision timing. See `examples/benchmark_system.py` for benchmark results.

---

## ⚙️ Configuration

### System Configuration (`config/system_config.yaml`)

All tunable parameters are centralized in YAML:

```yaml
vision_servo:
  aruco_dict: "DICT_6X6_250"
  control_gains:
    Kp_x: 0.01
    Kp_y: 0.01
  velocity_limits:
    max_linear: 0.3   # m/s
    max_angular: 0.5  # rad/s

grasp_predictor:
  model_path: "models/grasp_quality_model.h5"
  quality_threshold: 0.7  # Only attempt grasps > 70% confidence
  inference_device: "cpu"

ik_solver:
  robot_model: "ur10"
  joint_limits:
    q_min: [-3.14, -3.14, -3.14, -3.14, -3.14, -3.14]
    q_max: [3.14, 3.14, 3.14, 3.14, 3.14, 3.14]

performance:
  enable_benchmarking: true
  log_level: "DEBUG"
```

### Loading Configuration

Configuration is loaded dynamically at runtime:

```python
import yaml
from pathlib import Path

config_file = Path(__file__).parent.parent / 'config' / 'system_config.yaml'
with open(config_file, 'r') as f:
    config = yaml.safe_load(f)
```

---

## 🔧 Troubleshooting

### Common Issues

**Issue:** Tests fail with `RuntimeError: Context.init() must only be called once`
- **Solution:** Tests use session-scoped fixtures in `conftest.py` to manage ROS context globally. This is already handled.

**Issue:** `ModuleNotFoundError: No module named 'rclpy'`
- **Solution:** Install ROS 2 or skip ROS-dependent tests: `pytest tests/ -k "not ros"`

**Issue:** Docker build fails
- **Solution:** Ensure Docker daemon is running. Check `docker/Dockerfile` for dependency issues. Build with `--no-cache` for fresh install: `docker build --no-cache -t grasp-system .`

For more troubleshooting, see **[docs/TROUBLESHOOTING.md](./docs/TROUBLESHOOTING.md)**

---

## 📚 Documentation

### Complete Documentation Available

- **[docs/ARCHITECTURE.md](./docs/ARCHITECTURE.md)** — System design, component descriptions, data flow diagrams, and design rationale
- **[docs/SYSTEM_REPORT.pdf](./docs/SYSTEM_REPORT.pdf)** — Comprehensive technical report including:
  - Full system overview
  - Module implementation details
  - Performance benchmarking results with histograms
  - Latency analysis and constraints verification
  - Software engineering methodologies used

- **[docs/API.md](./docs/API.md)** — Complete API reference for all modules
- **[docs/TROUBLESHOOTING.md](./docs/TROUBLESHOOTING.md)** — Solutions to common issues

---

## 🌐 ROS 2 Integration

### Launch Full System

```bash
ros2 launch grasp_system full_grasp_system.launch.py
```

### ROS 2 Topics

The system publishes/subscribes to these topics:

```
Camera Topics:
  /camera/image (sensor_msgs/Image) — Raw camera frames
  
Control Topics:
  /servo/velocity (geometry_msgs/Twist) — Vision servoing velocity commands
  /grasp/quality_score (std_msgs/Float32) — Grasp quality predictions
  /arm/trajectory_goal (trajectory_msgs/JointTrajectory) — Target trajectories
  /arm/joint_commands (sensor_msgs/JointState) — Arm joint commands
```

---

## 🔐 Deployment Considerations

### Edge Device Compatibility

This system is optimized for edge deployment:
- Lightweight CNN model (<100ms inference on CPU)
- No GPU required for real-time operation
- Minimal memory footprint
- Low network bandwidth (local ROS only)

### Production Readiness

- ✅ Containerized (Docker)
- ✅ Automated testing (GitHub Actions)
- ✅ Centralized configuration (YAML)
- ✅ Comprehensive logging
- ✅ Error handling and graceful degradation
- ✅ Performance monitoring built-in

---

## 🤝 Contributing

This project welcomes contributions. To contribute:

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/your-feature`
3. Make your changes and add tests
4. Ensure all tests pass: `pytest tests/ -v`
5. Commit with descriptive messages
6. Push to your fork and submit a Pull Request

### Code Standards

- Follow PEP 8 (Python style guide)
- Add docstrings to all functions and classes
- Include type hints where applicable
- Write tests for new functionality
- Update documentation as needed

---

## 📄 License

This project is licensed under the **MIT License** — see [LICENSE](./LICENSE) file for details.

---

## 👤 Author

**Diksha Jangam**
- 🎓 MSc Robotics, University of Birmingham
- 📧 [dikshajangam26](https://github.com/dikshajangam26)
- 🔗 [LinkedIn](https://linkedin.com/in/diksha-jangam)

---

## 🙏 Acknowledgments

- **ROS 2** — Middleware for robotics
- **OpenCV** — Computer vision library
- **ikpy** — Inverse kinematics solver
- **TensorFlow/PyTorch** — Deep learning frameworks
- **pytest** — Testing framework

---

## 📞 Questions?

- **Documentation:** See [docs/](./docs/) folder
- **Technical Report:** [docs/SYSTEM_REPORT.pdf](./docs/SYSTEM_REPORT.pdf)
- **Architecture:** [docs/ARCHITECTURE.md](./docs/ARCHITECTURE.md)
- **Issues:** [GitHub Issues](https://github.com/dikshajangam26/grasp-system/issues)

---

## 🚀 Getting Started

Ready to run the system?

```bash
# 1. Clone and setup
git clone https://github.com/dikshajangam26/grasp-system.git
cd grasp-system
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt

# 2. Run tests
pytest tests/ -v

# 3. Run examples
python examples/benchmark_system.py

# 4. (Optional) Try Docker
docker build -t grasp-system .
docker run -it grasp-system:latest
```

That's it! Your autonomous grasp system is ready to go. 🎉

---

**Last Updated:** June 8, 2026  
**Version:** 1.0.0  
**Status:** ✅ Production Ready
