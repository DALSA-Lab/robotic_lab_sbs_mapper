# C++ Tools

The available tools are:
- [Detect Markers](./detect_markers/README.md)
- [Calibrate Hand-eye](./calib_hand_eye/README.md)
- [Capture Images](./capture_images/README.md)
- [Compute Poses](./compute_pose/README.md)
- [Compute Poses (all options)](./compute_pose_all_options/README.md)

# How to build
Build is done using CMake. A global CMake file exists, from which it is possible to build all tools:
```bash
cmake --build build
```

Or, to target a specific tool, use the `--target <tool>` argument:
```bash
cmake --build build --target calib
```

# How to run
All tool binares are found in the `/build` under their respective directories. Tools might require arguments to be included. See each of the tools respective READMEs for more information. 