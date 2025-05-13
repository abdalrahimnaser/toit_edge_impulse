set(CMAKE_CXX_STANDARD 17)

get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

set(MODEL_FOLDER "${PROJECT_ROOT}/build-root/model_folder")
set(EI_SDK_FOLDER "${MODEL_FOLDER}/edge-impulse-sdk")

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    add_definitions(-DEI_CLASSIFIER_TFLITE_ENABLE_ESP_NN=1)
    if(${IDF_TARGET} STREQUAL "esp32c6")
        add_definitions(-DEI_CLASSIFIER_TFLITE_ENABLE_ESP_NN_C6=1)
    endif()
endif()

option(DEFINE_DEBUG "Build the project using debugging code" OFF)
if(DEFINE_DEBUG)
    message("Adding Debug flag...")
    add_definitions(-DDEBUG)
    set(CMAKE_BUILD_TYPE Debug)
    message("Build type is " ${CMAKE_BUILD_TYPE})
endif()

set(include_dirs
    "${MODEL_FOLDER}"
    "${MODEL_FOLDER}/tflite-model"
    "${MODEL_FOLDER}/model-parameters"
    "${EI_SDK_FOLDER}"
)

include("${EI_SDK_FOLDER}/cmake/utils.cmake")

RECURSIVE_FIND_FILE_EXCLUDE_DIR(SOURCE_FILES "${EI_SDK_FOLDER}" "CMSIS" "*.cpp")
RECURSIVE_FIND_FILE_EXCLUDE_DIR(MODEL_FILES "${MODEL_FOLDER}/tflite-model" "CMSIS" "*.cpp")
RECURSIVE_FIND_FILE_EXCLUDE_DIR(CC_FILES "${EI_SDK_FOLDER}" "CMSIS" "*.cc")
RECURSIVE_FIND_FILE_EXCLUDE_DIR(S_FILES "${EI_SDK_FOLDER}" "CMSIS" "*.s")
RECURSIVE_FIND_FILE_EXCLUDE_DIR(C_FILES "${EI_SDK_FOLDER}" "CMSIS" "*.c")

list(APPEND SOURCE_FILES ${S_FILES})
list(APPEND SOURCE_FILES ${C_FILES})
list(APPEND SOURCE_FILES ${CC_FILES})
list(APPEND SOURCE_FILES ${MODEL_FILES})

idf_component_register(
    SRCS "toit_envelope.cpp" "fall_detection.cpp" ${SOURCE_FILES}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES toit
    WHOLE_ARCHIVE
)

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)
