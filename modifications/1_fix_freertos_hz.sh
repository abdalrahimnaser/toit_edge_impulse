#!/bin/bash

# This script fixes the CMake Error: CONFIG_FREERTOS_HZ
# by patching the relevant line in CMakeLists.txt.

CMAKE_FILE="build-root/managed_components/espressif__arduino-esp32/CMakeLists.txt"
TARGET_LINE='if(NOT CONFIG_FREERTOS_HZ EQUAL 1000 AND NOT "$ENV{ARDUINO_SKIP_TICK_CHECK}")'
FIX_LINE='if(NOT CONFIG_FREERTOS_HZ EQUAL 100 AND NOT "$ENV{ARDUINO_SKIP_TICK_CHECK}")'

if [ -f "$CMAKE_FILE" ]; then
  echo "Fixing CONFIG_FREERTOS_HZ in $CMAKE_FILE..."
  
  # First try the exact line match approach
  if grep -qF "$TARGET_LINE" "$CMAKE_FILE"; then
    sed -i "s|$TARGET_LINE|$FIX_LINE|" "$CMAKE_FILE"
    echo "Applied CONFIG_FREERTOS_HZ fix using exact line matching."
  else
    # If exact match fails, use a more general approach
    sed -i '/CONFIG_FREERTOS_HZ EQUAL 1000/{
      s/CONFIG_FREERTOS_HZ EQUAL 1000/CONFIG_FREERTOS_HZ EQUAL 100/
    }' "$CMAKE_FILE"
    echo "Applied CONFIG_FREERTOS_HZ fix using pattern matching."
  fi
  
  # Also replace any assignment CONFIG_FREERTOS_HZ=1000 with CONFIG_FREERTOS_HZ=100
  if grep -q "CONFIG_FREERTOS_HZ=1000" "$CMAKE_FILE"; then
    sed -i 's/CONFIG_FREERTOS_HZ=1000/CONFIG_FREERTOS_HZ=100/g' "$CMAKE_FILE"
    echo "Applied CONFIG_FREERTOS_HZ assignment fix."
  fi
  
  echo "All CONFIG_FREERTOS_HZ fixes have been applied to $CMAKE_FILE."
else
  echo "Warning: $CMAKE_FILE not found. Skipping CONFIG_FREERTOS_HZ fix."
  echo "This file is typically generated during the build process."
fi
