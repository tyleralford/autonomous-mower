# Mower Workspace

This is the ROS 2 workspace for the autonomous mower project.

## Building the Workspace

To build all packages in this workspace:

```bash
cd /path/to/mower_ws
colcon build
```

## Sourcing the Workspace

After building, source the workspace to use the packages:

```bash
source install/setup.bash
```

## Package Structure

- `src/` - Source code for all ROS 2 packages
- `build/` - Build artifacts (auto-generated)
- `install/` - Installation files (auto-generated)
- `log/` - Build logs (auto-generated)

## Usage

Instructions for running the simulation will be added as packages are developed.
