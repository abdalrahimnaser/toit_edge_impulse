#!/bin/bash

# This script fixes the "Missing LSM6DS3.h" error by:
# 1. Downloading the required library
# 2. Extracting it
# 3. Copying it to the appropriate location
# 4. Adding the necessary lines to CMakeLists.txt

# Configuration
LIBRARY_URL="https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3/archive/refs/tags/v2.0.4.zip"
DOWNLOAD_DIR="/tmp"
ZIP_FILE="$DOWNLOAD_DIR/lsm6ds3_v2.0.4.zip"
EXTRACT_DIR="$DOWNLOAD_DIR/lsm6ds3_extract"
TARGET_DIR="build-root/managed_components/espressif__arduino-esp32/libraries/Seeed_Arduino_LSM6DS3-2.0.4"
CMAKE_FILE="build-root/managed_components/espressif__arduino-esp32/CMakeLists.txt"

echo "Starting LSM6DS3 library installation..."

# Check if the target directory already exists
if [ -d "$TARGET_DIR" ] && [ -f "$TARGET_DIR/src/LSM6DS3.h" ]; then
  echo "LSM6DS3 library is already installed at $TARGET_DIR."
else
  # Create directories if they don't exist
  mkdir -p "$DOWNLOAD_DIR"
  mkdir -p "$EXTRACT_DIR"
  mkdir -p "$TARGET_DIR"

  # Step 1: Download the library
  echo "Downloading LSM6DS3 library from $LIBRARY_URL..."
  if command -v curl &> /dev/null; then
    curl -L "$LIBRARY_URL" -o "$ZIP_FILE"
  elif command -v wget &> /dev/null; then
    wget "$LIBRARY_URL" -O "$ZIP_FILE"
  else
    echo "Error: Neither curl nor wget is installed. Please install one of them and try again."
    exit 1
  fi

  if [ ! -f "$ZIP_FILE" ]; then
    echo "Error: Failed to download the library."
    exit 1
  fi

  # Step 2: Extract the library
  echo "Extracting library..."
  if command -v unzip &> /dev/null; then
    unzip -q "$ZIP_FILE" -d "$EXTRACT_DIR"
  else
    echo "Error: unzip is not installed. Please install it and try again."
    exit 1
  fi

  # Step 3: Copy the library to the target directory
  echo "Copying library to $TARGET_DIR..."
  cp -r "$EXTRACT_DIR"/*/* "$TARGET_DIR"

  # Clean up
  echo "Cleaning up temporary files..."
  rm -rf "$ZIP_FILE" "$EXTRACT_DIR"

  echo "LSM6DS3 library installation completed."
fi

# Step 4: Modify the CMakeLists.txt to include the new library
if [ -f "$CMAKE_FILE" ]; then
  echo "Modifying $CMAKE_FILE to include the LSM6DS3 library..."
  
  # Check if the library is already included
  if grep -q "libraries/Seeed_Arduino_LSM6DS3-2.0.4/LSM6DS3.cpp" "$CMAKE_FILE" && \
     grep -q "list(APPEND ARDUINO_LIBRARIES_INCLUDEDIRS libraries/Seeed_Arduino_LSM6DS3-2.0.4)" "$CMAKE_FILE"; then
    echo "LSM6DS3 library is already included in $CMAKE_FILE."
  else
    # Create a backup of the original file
    cp "$CMAKE_FILE" "${CMAKE_FILE}.bak"
    
    # Add the LSM6DS3 library source to the BLE_SRCS section
    if ! grep -q "libraries/Seeed_Arduino_LSM6DS3-2.0.4/LSM6DS3.cpp" "$CMAKE_FILE"; then
      sed -i '/set(ARDUINO_LIBRARY_BLE_SRCS/,/)/s/)$/\n  libraries\/Seeed_Arduino_LSM6DS3-2.0.4\/LSM6DS3.cpp)/' "$CMAKE_FILE"
      echo "Added LSM6DS3.cpp to library sources."
    fi
    
    # Add the include directory
    if ! grep -q "list(APPEND ARDUINO_LIBRARIES_INCLUDEDIRS libraries/Seeed_Arduino_LSM6DS3-2.0.4)" "$CMAKE_FILE"; then
      sed -i '/list(APPEND ARDUINO_LIBRARIES_INCLUDEDIRS/a list(APPEND ARDUINO_LIBRARIES_INCLUDEDIRS libraries/Seeed_Arduino_LSM6DS3-2.0.4)' "$CMAKE_FILE"
      echo "Added LSM6DS3 include directory."
    fi
  fi
else
  echo "Warning: $CMAKE_FILE not found. Cannot modify to include the LSM6DS3 library."
fi

# Step 5: Fix the I2C pin initialization in LSM6DS3.cpp (Fix #4 from README)
LSM6DS3_CPP_FILE="$TARGET_DIR/LSM6DS3.cpp"
if [ -f "$LSM6DS3_CPP_FILE" ]; then
  echo "Checking I2C pin initialization in $LSM6DS3_CPP_FILE..."
  
  # Check if the fix is already applied
  if grep -q "Wire.begin(0,1)" "$LSM6DS3_CPP_FILE"; then
    echo "I2C pin initialization fix already applied."
  else
    # Create a backup of the original file
    cp "$LSM6DS3_CPP_FILE" "${LSM6DS3_CPP_FILE}.bak"
    
    # Replace Wire.begin() with Wire.begin(0,1)
    sed -i 's/Wire.begin();/Wire.begin(0,1);/' "$LSM6DS3_CPP_FILE"
    echo "Applied I2C pin initialization fix."
  fi
else
  echo "Warning: $LSM6DS3_CPP_FILE not found. Cannot apply I2C pin initialization fix."
fi

echo "All fixes completed successfully!" 