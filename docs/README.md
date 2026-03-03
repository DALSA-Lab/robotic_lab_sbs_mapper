# Docs
This directory contain the documentation source, from with the online documentation is built. 

It contains a few working examples, located in [usage/examples](./usage/examples/).

The `doxygen` folder is only relevant when building the documentation locally, or when built by the [gh-pages](https://github.com/DALSA-Lab/robotic_lab_sbs_mapper/tree/gh-pages) branch. 

## Structure
```
.
├── architecture
│   └── components
├── code
│   ├── cpp_tools
│   │   ├── calib_hand_eye
│   │   ├── capture_images
│   │   ├── compute_pose
│   │   ├── compute_pose_all_options
│   │   └── detect_markers
│   └── py_pkgs
├── doxygen
│   ├── html
│   │   └── search
│   └── xml
└── usage
    └── examples
        ├── calibration
        └── plate_detection
```