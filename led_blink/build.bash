#!/bin/bash

cmake -S . -B build          # configure

cmake --build build          # build

# cmake --build build -j 8     # parallel

# cmake -DCMAKE_INSTALL_PREFIX=$HOME/.local -S . -B build
# cmake --install build        # build + install

# Install
# cmake --build build --target install

# Clean
# cmake --build build --target clean

# Test
# cmake --build build --target test
