#!/bin/bash

# initialize and update submodules
git submodule init && git submodule update

# Build and run clang tidy
cmake -B build -G Ninja
cmake --build build
