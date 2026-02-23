# Calibrate Hand-eye

A simple program to compute the distortion coefficients, intrinsic and extrinsic parameters. These parameters are exported to a YAML file. When running the program a numeric presentation of how the camera is oriented with respect to the TCP is displayed, along with other camera calibration metrics, such as the reprojection error.

## Build Instructions

```
mkdir build
cd build
cmake ..
```

## Run Instructions
The binary takes the following arguments:
- 1
- 2
- 3
```
./build/out -1 -2 -3
```


