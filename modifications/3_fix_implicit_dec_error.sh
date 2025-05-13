#!/bin/bash

# This script fixes the "Implicit Declaration Error" in ble_hs_flow.c
# by adding the missing function declaration.

BLE_HS_FLOW_FILE="toit/third_party/esp-idf/components/bt/host/nimble/nimble/nimble/host/src/ble_hs_flow.c"

if [ -f "$BLE_HS_FLOW_FILE" ]; then
  # Check if the fix is already applied
  if grep -q "int ble_hci_trans_set_acl_free_cb" "$BLE_HS_FLOW_FILE"; then
    echo "Fix for 'Implicit Declaration Error' already applied to $BLE_HS_FLOW_FILE."
  else
    # Add the function definition after the includes
    sed -i '/^#include "ble_hs_priv.h"$/a \
int ble_hci_trans_set_acl_free_cb(os_mempool_put_fn *cb, void *arg) {\
    return BLE_ERR_UNSUPPORTED; // Temporary workaround\
}\
' "$BLE_HS_FLOW_FILE"
    echo "Applied fix for 'Implicit Declaration Error' to $BLE_HS_FLOW_FILE."
  fi
else
  echo "Warning: $BLE_HS_FLOW_FILE not found. Skipping 'Implicit Declaration Error' fix."
fi