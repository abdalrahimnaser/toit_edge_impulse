# Template repository for deploying Edge Impulse C++ Models on Toit-enabled devices


This repository provides a template for integrating Edge Impulse machine learning models with Toit-enabled devices. It allows you to easily deploy and run Edge Impulse models on your IoT hardware using the Toit programming language and runtime.


## Getting Started

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/toit_edge_impulse.git
   cd toit_edge_impulse
   ```

2. Export your Edge Impulse model as a C++ library and place the files in the `build-root/model_folder` directory.

3. Required Patches (TBD): Some patches are currently needed to ensure compatibility between Edge Impulse models and the Toit runtime. This section will be updated with specific instructions for applying these patches.

4. Configuration
* Adjust or remove the C components in the `components` folder.
* add external esp-idf dependencies by modifiying idf_component.yml in components/fall_detection
* Run `make menuconfig` to configure the build.
* Adjust the [ci.yml](.github/workflows/ci.yml) file to match your setup. Typically, you don't need
  to compile on Windows or macOS.

### Build
* Run `make` to build the envelope. It should end up with a `build/esp32c6/firmware.envelope`.


## Makefile targets
- `make` or `make all` - Build the envelope.
- `make init` - Initialize after cloning. See the Setup section above.
- `make menuconfig` - Runs the ESP-IDF menuconfig tool in the build-root. Also creates the `sdkconfig.defaults` file.
- `make diff` - Show the differences between your configuration (sdkconfig and partitions.csv) and the default Toit configuration.
- `make clean` - Remove all build artifacts.

  
## Usage Example - Fall Detection: EI model: https://studio.edgeimpulse.com/public/23068/latest

```toit
import system.external
import io

FUNCTION-ID ::= 0

main:
  echo := external.Client.open "toitlang.org/demo-echo"
  threshold := 0.5
  while 1:
    response := io.Reader (echo.request FUNCTION-ID "fetch result")
    fall_probability := response.little-endian.read-float32

    if fall_probability >= threshold : print "Fall Detected!"

  echo.close
```
![alt text](demo_screenshot.png "Title")

This repository was cloned from this [template](https://github.com/toitlang/template-custom-envelope).

## Patches:
make issues:
----------------------
CMake Error at managed_components/espressif__arduino-esp32/CMakeLists.txt:371 (message):
  esp32-arduino requires CONFIG_FREERTOS_HZ=1000 (currently 100)

go to line 369 and change this: if(NOT CONFIG_FREERTOS_HZ EQUAL 1000 AND NOT "$ENV{ARDUINO_SKIP_TICK_CHECK}")
    # See delay() in cores/esp32/esp32-hal-misc.c.
    message(FATAL_ERROR "esp32-arduino requires CONFIG_FREERTOS_HZ=1000 "
                        "(currently ${CONFIG_FREERTOS_HZ})") to this if(NOT CONFIG_FREERTOS_HZ EQUAL 100 AND NOT "$ENV{ARDUINO_SKIP_TICK_CHECK}")
    # See delay() in cores/esp32/esp32-hal-misc.c.
    message(FATAL_ERROR "esp32-arduino requires CONFIG_FREERTOS_HZ=100 "
                        "(currently ${CONFIG_FREERTOS_HZ})")
--------------------------
/home/abood/fresh_toit_ei/toit_edge_impulse/toit/third_party/esp-idf/components/bt/host/nimble/nimble/nimble/host/src/ble_hs_flow.c:269:5: error: implicit declaration of function 'ble_hci_trans_set_acl_free_cb'; did you mean 'ble_hci_trans_hs_acl_tx'? [-Werror=implicit-function-declaration]
  269 |     ble_hci_trans_set_acl_free_cb(ble_hs_flow_acl_free, NULL);
      |     ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      |     ble_hci_trans_hs_acl_tx
go to that file and add this temporary workaround below includes:
int ble_hci_trans_set_acl_free_cb(os_mempool_put_fn *cb, void *arg) {
    return BLE_ERR_UNSUPPORTED; // Temporary workaround
}
---------------------------
ion.cpp:6:10: fatal error: LSM6DS3.h: No such file or directory
    6 | #include <LSM6DS3.h>
install the library from here: 
modify the .../../... to 
or well tutorial is: https://www.youtube.com/watch?v=7wOpKfeLd7w

----------------------------
if using lightbug's ....., change wire (i2c) pin initialisations in LSM6DS3.cpp to 0,1 , code line 88
-----------------------------

when deploying decisioning.toit via jag run -d ....:
----------------------------------------
fatal: Throwing new[] not allowed: 24
in toit\src\top.cc set throwing_new_allowed in line 81 to true
----------------------------------------

