# Kalman Filter



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

