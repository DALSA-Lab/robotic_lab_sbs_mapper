# C++ Tools

The available tools are:
- Detect Markers
- Calibrate Hand-eye
- Capture Images
- Compute Poses
- Compute Poses (all options)

# How to build
Build is done using CMake. A global CMAKE file exists, from which it is possible to build all tools:
```bash
cmake --build build
```

Or, to target a specific tool, use the `--target <tool>` argument:
```bash
cmake --build build --target calib
```

# How to run
All tool binares are found in the `/build` under their respective directories. Tools might require arguments to be included. See each of the tools respective READMEs for more information. 