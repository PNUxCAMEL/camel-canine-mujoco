#!/bin/bash

# 현재 스크립트의 디렉토리 경로
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="$(cd "${PARENT_DIR}/controller/canine_controller/" && pwd)"

cd ${BUILD_DIR}
./canine_simulation
