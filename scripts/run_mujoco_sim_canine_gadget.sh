#!/bin/bash

# 현재 스크립트의 디렉토리 경로
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PYTHON_SCRIPT_DIR="$(cd "${PARENT_DIR}/python/" && pwd)"

cd ${PYTHON_SCRIPT_DIR}
python3 main.py --robot 1