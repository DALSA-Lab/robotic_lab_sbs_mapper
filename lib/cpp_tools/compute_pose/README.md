# Compute Pose

A script to compute the 6DOF pose of ArUco markers in a collection of images. The program takes various input parameters related to the detection algorithm, camera parameters, etc. Output is a CSV file for each detected marker, containing translation and orientation wrt. to the camera optical frame.

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


