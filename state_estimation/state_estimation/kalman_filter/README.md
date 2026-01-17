# Kalman Filter

## Overview

Kalman filter (KF) estimator library, with the following implementations:
- Linear Kalman Filter (LKF)
- Extended Kalman Filter (EKF)
- Unscent Kalman Filter (UKF)


## Setup

### Dependencies
- Eigen
```sh
sudo apt install libeigen3-dev
```

- SDL2
```sh
sudo apt install libsdl2-dev libsdl2-ttf-dev
```

- symengine
```sh
sudo apt install cmake libgmp-dev
git clone https://github.com/symengine/symengine.git
mkdir build && cd build
cmake ..
make
make install
```

- AKFSF-Simulation
```sh
git clone https://github.com/technitute/AKFSF-Simulation-CPP
cd AKFSF-Simulation-CPP
mkdir build
cd build
cmake ..
make
```


## Testing

### Unit tests
```sh
./build/kalman_filter/test_linear_kalman_filter
```

Coverage
```sh
lcov --directory ./build/kalman_filter/ --capture --output-file coverage.info
lcov --remove coverage.info '*/external/*' '*/ros2/*' --output-file coverage_filtered.info
genhtml coverage_filtered.info --output-directory coverage_report
```





## Reference
