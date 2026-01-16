#!/bin/bash
source /opt/ros/humble/setup.bash

# 自动计算多线程
CORES=$(nproc)
AVAILABLE_MEM_GB=$(free -g | awk '/^Mem:/{print $7}')
MEM_LIMITED_JOBS=$((AVAILABLE_MEM_GB / 2))
PARALLEL_JOBS=$((CORES < MEM_LIMITED_JOBS ? CORES : MEM_LIMITED_JOBS))
PARALLEL_JOBS=$((PARALLEL_JOBS < 4 ? 4 : PARALLEL_JOBS))
echo "==> CPU cores: $CORES, Available memory: ${AVAILABLE_MEM_GB}GB"
echo "==> Building with $PARALLEL_JOBS parallel workers"

colcon --log-level error build \
    --symlink-install \
    --parallel-workers $PARALLEL_JOBS \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_FLAGS="-O3 -pipe -Wno-pragmas -Wno-unknown-pragmas" \
        -DBUILD_TESTING=OFF \
    -Wno-dev

source install/setup.bash
echo "==> Build completed!"
