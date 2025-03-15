# Copyright (C) 2024 Toitware ApS.
# Use of this source code is governed by a Zero-Clause BSD license that can
# be found in the LICENSE file.

cmake_minimum_required(VERSION 3.5)

set(IDF_PATH "$ENV{IDF_PATH}" CACHE FILEPATH "Path to the ESP-IDF directory")
set(TOIT_SDK_DIR "$ENV{TOIT_SDK_DIR}" CACHE FILEPATH "Path to the Toit SDK directory")
set(TOIT_EXTRA_COMPONENTS "$ENV{TOIT_EXTRA_COMPONENTS}" CACHE FILEPATH "Path to Toit components directories")

list(APPEND EXTRA_COMPONENT_DIRS "../toit/toolchains/idf/components" ${TOIT_EXTRA_COMPONENTS})
# list(APPEND EXTRA_COMPONENT_DIRS "./managed_components" ${TOIT_EXTRA_COMPONENTS})

include("${IDF_PATH}/tools/cmake/project.cmake")
project(toit)

include("local.cmake" OPTIONAL)

idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-unused-variable" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-maybe-uninitialized" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=format=" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-unused-but-set-parameter" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=nonnull-compare" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=stringop-truncation" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=uninitialized" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-stringop-truncation" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-deprecated-declarations" APPEND)

toit_postprocess()
