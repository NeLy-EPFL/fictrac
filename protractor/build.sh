#!/bin/bash
# Build script for protractor tool

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
BIN_DIR="$SCRIPT_DIR/../bin"
BIN_DIR="$(cd "$BIN_DIR" && pwd)"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake ..
make

# Copy to bin folder
if [ -f "protractor" ]; then
    cp protractor "$BIN_DIR/"
    echo "Built and copied to $BIN_DIR/protractor"
fi

