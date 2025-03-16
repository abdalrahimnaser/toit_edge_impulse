set (CMAKE_CXX_STANDARD 17)

set(MODEL_FOLDER /home/abood/lb/fix/demo-custom-envelope/build-root/model_folder)
set(EI_SDK_FOLDER /home/abood/lb/fix/demo-custom-envelope/build-root/model_folder/edge-impulse-sdk)

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    # enables generic ESP-NN optimizations by Espressif
    add_definitions(-DEI_CLASSIFIER_TFLITE_ENABLE_ESP_NN=1)
    # conditionally enable ESP32S3 specific optimizations
    if(${IDF_TARGET} STREQUAL "esp32c6")
        add_definitions(-DEI_CLASSIFIER_TFLITE_ENABLE_ESP_NN_C6=1)
    endif()
endif()

OPTION(DEFINE_DEBUG
    "Build the project using debugging code"
    OFF)
IF(DEFINE_DEBUG)
    MESSAGE("Adding Debug flag...")
    ADD_DEFINITIONS(-DDEBUG)
    SET(CMAKE_BUILD_TYPE Debug)
    MESSAGE("Build type is " ${CMAKE_BUILD_TYPE})
ENDIF(DEFINE_DEBUG)

set(include_dirs
    ${MODEL_FOLDER}
    ${MODEL_FOLDER}/tflite-model
    ${MODEL_FOLDER}/model-parameters
    ${EI_SDK_FOLDER}
    )

include(${EI_SDK_FOLDER}/cmake/utils.cmake)
# include($ENV{IDF_PATH}/tools/cmake/project.cmake)




RECURSIVE_FIND_FILE_EXCLUDE_DIR(SOURCE_FILES "/home/abood/lb/fix/demo-custom-envelope/build-root/model_folder/edge-impulse-sdk" "CMSIS" "*.cpp")
RECURSIVE_FIND_FILE_EXCLUDE_DIR(MODEL_FILES "/home/abood/lb/fix/demo-custom-envelope/build-root/model_folder/tflite-model" "CMSIS" "*.cpp")
RECURSIVE_FIND_FILE_EXCLUDE_DIR(CC_FILES "/home/abood/lb/fix/demo-custom-envelope/build-root/model_folder/edge-impulse-sdk" "CMSIS" "*.cc")
RECURSIVE_FIND_FILE_EXCLUDE_DIR(S_FILES "/home/abood/lb/fix/demo-custom-envelope/build-root/model_folder/edge-impulse-sdk" "CMSIS" "*.s")
RECURSIVE_FIND_FILE_EXCLUDE_DIR(C_FILES "/home/abood/lb/fix/demo-custom-envelope/build-root/model_folder/edge-impulse-sdk" "CMSIS" "*.c")

list(APPEND SOURCE_FILES ${S_FILES})
list(APPEND SOURCE_FILES ${C_FILES})
list(APPEND SOURCE_FILES ${CC_FILES})
list(APPEND SOURCE_FILES ${MODEL_FILES})


# idf_component_register(
#   REQUIRES toit
#   SRCS qrcode.c 
#   WHOLE_ARCHIVE
# )


idf_component_register(SRCS "toit_envelope.cpp" "fall_detection.cpp" "${SOURCE_FILES}"
                       INCLUDE_DIRS "${include_dirs}"      
                       REQUIRES toit
                       WHOLE_ARCHIVE                
                       )

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)