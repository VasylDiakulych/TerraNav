#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"

git submodule update --init --recursive

cmake -B build -DCMAKE_BUILD_TYPE=Release -DSANITIZE=OFF --log-level=ERROR
cmake --build build --target viz
exec ./build/viz "$@"
