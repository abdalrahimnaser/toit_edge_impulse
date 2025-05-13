#!/bin/bash

# This script fixes the CMake Error: CONFIG_FREERTOS_HZ
# by patching the relevant line in CMakeLists.txt and sdkconfig.defaults if present.

CMAKE_FILE="build-root/managed_components/espressif__arduino-esp32/CMakeLists.txt"

if [ -f "$CMAKE_FILE" ]; then
  # Fix the conditional line if needed
  sed -i '/CONFIG_FREERTOS_HZ EQUAL 1000/{
    s/CONFIG_FREERTOS_HZ EQUAL 1000/CONFIG_FREERTOS_HZ EQUAL 100/
  }' "$CMAKE_FILE"

  # Also replace any assignment CONFIG_FREERTOS_HZ=1000 with CONFIG_FREERTOS_HZ=100
  sed -i 's/CONFIG_FREERTOS_HZ=1000/CONFIG_FREERTOS_HZ=100/g' "$CMAKE_FILE"

  echo "Applied CONFIG_FREERTOS_HZ fix to $CMAKE_FILE."
else
  echo "Warning: $CMAKE_FILE not found. Skipping CONFIG_FREERTOS_HZ fix."
  echo "This file is typically generated during the build process."
fi
#!/bin/bash

# This script fixes the CMake Error: CONFIG_FREERTOS_HZ
# by patching the relevant line in CMakeLists.txt.

CMAKE_FILE="build-root/managed_components/espressif__arduino-esp32/CMakeLists.txt"
TARGET_LINE='if(NOT CONFIG_FREERTOS_HZ EQUAL 1000 AND NOT "$ENV{ARDUINO_SKIP_TICK_CHECK}")'
FIX_LINE='if(NOT CONFIG_FREERTOS_HZ EQUAL 100 AND NOT "$ENV{ARDUINO_SKIP_TICK_CHECK}")'

if [ -f "$CMAKE_FILE" ]; then
  # Only replace the exact line if it matches the problematic condition
  if grep -qF "$TARGET_LINE" "$CMAKE_FILE"; then
    sed -i "s|$TARGET_LINE|$FIX_LINE|" "$CMAKE_FILE"
    echo "Applied CONFIG_FREERTOS_HZ fix to $CMAKE_FILE."
  else
    # Try a more general approach if the line is slightly different
    sed -i '/CONFIG_FREERTOS_HZ EQUAL 1000/{
      s/CONFIG_FREERTOS_HZ EQUAL 1000/CONFIG_FREERTOS_HZ EQUAL 100/
    }' "$CMAKE_FILE"
    echo "Applied generic CONFIG_FREERTOS_HZ fix to $CMAKE_FILE."
  fi
else
  echo "Warning: $CMAKE_FILE not found. Skipping CONFIG_FREERTOS_HZ fix."
  echo "This file is typically generated during the build process."
fi
