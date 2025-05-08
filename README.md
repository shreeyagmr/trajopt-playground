# Trajopt Playground

## Overview

TrajOpt Playground is a standalone C++ application for experimenting with trajectory optimization using Tesseract's TrajOpt and TrajOptIfopt planners. The project focuses on planning motions for a Fanuc M710 robot mounted on a 7th axis (linear rail), using input waypoints from a CSV file.

This playground enables testing and comparison of:
* TrajOpt vs TrajOptIfopt performance on complex trajectories
* Different seeding strategies and their impact on convergence
* Optimization behavior with varying numbers of waypoints
* Effects of different constraint formulations on solution quality

## Project Structure

```
trajopt-playground/
├── CMakeLists.txt
├── include/
│   └── utils.hpp
├── src/
│   └── main.cpp
├── config/
│   └── robots/
│       └── fanuc_m710/
│           ├── robot_actual_roe.urdf
│           ├── robot_actual_roe.srdf
│           ├── kinematics_plugin_config.yaml
│           ├── contact_manager_config.yaml
│           └── meshes/
├── data/
│   ├── input.csv
│   └── descartes_solution.json
└── output/
```

## Building the Project

```bash
mkdir -p build && cd build
cmake ..
make
```

After building, the executable will be available at `./build/trajopt_playground`.

## Running the Planner

Execute the TrajOpt Playground application:

```bash
./build/trajopt_playground [OPTIONS]
```

### Available Command-line Arguments

| Argument               | Description                                                                                             |
| ---------------------- | ------------------------------------------------------------------------------------------------------- |
| `--use_ifopt`          | Use TrajOptIfoptMotionPlanner instead of TrajOptMotionPlanner (default: false)                          |
| `--points N`           | Consider only the first N points from the toolpath (default: all points)                                |
| `--seed_method METHOD` | Set the seeding method for the planner:                                                                 |
|                        | `none` (default): No explicit seeding                                                                   |
|                        | `waypoint`: Seed each waypoint individually (requires Descartes solution)                               |
|                        | `pci`: Manually construct PCI with seed (requires Descartes solution, only works with standard TrajOpt) |
| `--help` or `-h`       | Display help message with available options                                                             |

### Example Usage

```bash
# Run with default settings (TrajOpt, all points, no seeding)
./build/trajopt_playground

# Use TrajOptIfopt planner
./build/trajopt_playground --use_ifopt

# Plan for only the first 20 points
./build/trajopt_playground --points 20

# Use waypoint seeding with standard TrajOpt
./build/trajopt_playground --seed_method waypoint

# Use ProblemConstructionInfo (PCI) seeding with standard TrajOpt for first 10 points
./build/trajopt_playground --points 10 --seed_method pci

# Use waypoint seeding with TrajOptIfopt
./build/trajopt_playground --use_ifopt --seed_method waypoint
```

## Dependencies

* Tesseract
* Eigen3
* nlohmann_json
* console_bridge

## Notes

This is a standalone application designed for experimentation and research. For integration with ROS2 or other robotics frameworks, additional configuration may be required.

