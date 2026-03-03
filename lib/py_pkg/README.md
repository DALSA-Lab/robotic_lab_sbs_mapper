# MapIt Python Module

This is the main library/module for carrying out camera calibration, ArUco detection and localisation. The module is split into the following sourcefiles:

```
py_pkg
├── pyproject.toml
├── README.md
├── src
│   ├── mapit
│   │   ├── calib.py
│   │   ├── core.py
│   │   ├── detector.py
│   │   ├── __init__.py
│   │   ├── models.py
│   │   └── utils.py
└── tests
    ├── __init__.py
    └── test_main.py
```

The Aruco detection rely on the `CalibratedCamera` class, defined in [calib.py](./src/mapit/calib.py). A new `CalibratedCamera` can be instantiated either by carrying out a calibration, through manual definition or by loading a YAML config. 

To be hardware agnostic, the `CalibratedCamera` object implements a `frame_grabber` callable, which must be configured by the user. A minimal example would be using OpenCV's VideoCapture class:

```python
# Camera initialisation
cam = cv2.VideoCapture(2) # ID depends on the video source
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
def frame_grabber(camera):
    _, frame = camera.read()
    return frame

# create new CalibratedCamera object
camera = CalibratedCamera.new_from_config(
    "calibrated_camera.yml", lambda: frame_grabber(cam)
)
```

The ArUco detector depends on this `CalibratedCamera` object, as it invokes the `frame_grabber` function and uses the intrinsic parameters to estimate ArUco marker pose.