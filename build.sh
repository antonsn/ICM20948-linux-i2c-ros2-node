#!/bin/bash

# Quick build script for ICM20948 project
echo "Building ICM20948 project..."

# Create build directory if it doesn't exist
if [ ! -d "build" ]; then
    mkdir build
fi

# Navigate to build directory
cd build

# Run cmake if needed
if [ ! -f "Makefile" ]; then
    echo "Running CMake configuration..."
    cmake ..
fi

# Build with parallel jobs
echo "Building with make -j4..."
make -j4

echo "Build complete!"

