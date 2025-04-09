#!/bin/bash

# 현재 스크립트의 디렉토리 경로
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="$(cd "${PARENT_DIR}/controller/canine_console/build/" && pwd)"

if [ -d "${BUILD_DIR}" ]; then
    echo "CANINE build folder path : $BUILD_DIR"
else
    echo "Build folder is not found."
    echo "Make build folder."
    cd ${PARENT_DIR}
    mkdir "controller/canine_console/build"
    BUILD_DIR="$(cd "${PARENT_DIR}/controller/canine_console/build/" && pwd)"
fi

cd ${BUILD_DIR}
cmake .. -DCMAKE_BUILD_TYPE=Release
make
./canine_console
