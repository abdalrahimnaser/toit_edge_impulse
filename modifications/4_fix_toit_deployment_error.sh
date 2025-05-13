#!/bin/bash

# This script fixes the "Toit Deployment Fatal Error" by setting 
# throwing_new_allowed to true in toit/src/top.cc

TOP_CC_FILE="toit/src/top.cc"

echo "Fixing Toit Deployment Fatal Error..."

if [ -f "$TOP_CC_FILE" ]; then
  # Check if the fix is already applied
  if grep -q "bool throwing_new_allowed = true;" "$TOP_CC_FILE"; then
    echo "Fix for 'Toit Deployment Fatal Error' already applied to $TOP_CC_FILE."
  else
    # Create a backup of the original file
    cp "$TOP_CC_FILE" "${TOP_CC_FILE}.bak"
    
    # Replace 'bool throwing_new_allowed = false;' with 'bool throwing_new_allowed = true;'
    sed -i 's/bool throwing_new_allowed = false;/bool throwing_new_allowed = true;/g' "$TOP_CC_FILE"
    
    # If the above doesn't match (maybe it's already true or has different formatting),
    # try a more general approach
    if ! grep -q "bool throwing_new_allowed = true;" "$TOP_CC_FILE"; then
      sed -i 's/throwing_new_allowed = false/throwing_new_allowed = true/g' "$TOP_CC_FILE"
    fi
    
    # Verify the fix was applied
    if grep -q "throwing_new_allowed = true" "$TOP_CC_FILE"; then
      echo "Successfully applied fix for 'Toit Deployment Fatal Error' to $TOP_CC_FILE."
    else
      echo "Failed to apply fix. Please check $TOP_CC_FILE manually."
    fi
  fi
else
  echo "Error: $TOP_CC_FILE not found. Cannot apply fix for 'Toit Deployment Fatal Error'."
fi 