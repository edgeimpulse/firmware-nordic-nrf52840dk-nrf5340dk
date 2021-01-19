cmake_minimum_required(VERSION 3.13.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(example-standalone-inferencing-zephyr)

include(utils/cmake/utils.cmake)
# Needed for colorful output
zephyr_compile_options(-fdiagnostics-color=always)

# Use hardware acceleration for DSP and Neural Network code
# You'll need to disable these on non-Arm cores
add_definitions(-DEIDSP_USE_CMSIS_DSP=1
                -DEIDSP_LOAD_CMSIS_DSP_SOURCES=1
                -DEI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN=1
                -DEIDSP_QUANTIZE_FILTERBANK=0
                -DARM_MATH_LOOPUNROLL
                -DEI_SENSOR_AQ_STREAM=FILE
                )

# Edge impulse SDK include directories
set(INCLUDES
    .
    src
    tflite-model
    model-parameters
    edge-impulse-sdk
    edge-impulse-sdk/third_party/ruy
    edge-impulse-sdk/third_party/gemmlowp
    edge-impulse-sdk/third_party/flatbuffers/include
    edge-impulse-sdk/third_party
    edge-impulse-sdk/tensorflow
    edge-impulse-sdk/dsp
    edge-impulse-sdk/classifier
    edge-impulse-sdk/anomaly
    edge-impulse-sdk/CMSIS/NN/Include
    edge-impulse-sdk/CMSIS/DSP/PrivateInclude
    edge-impulse-sdk/CMSIS/DSP/Include
    edge-impulse-sdk/CMSIS/Core/Include
    edge-impulse/ingestion-sdk-c
    edge-impulse/ingestion-sdk-platform/NordicSemi-nrf52
    edge-impulse/ingestion-sdk-platform/NordicSemi-nrf52/sensors
    edge-impulse/mbedtls_hmac_sha256_sw
    edge-impulse/QCBOR/inc
    edge-impulse/QCBOR/src
    edge-impulse/repl
    edge-impulse-sdk/porting
    )
include_directories(${INCLUDES})

#file(GLOB PLATFORM "${CMAKE_CURRENT_LIST_DIR}/ingestion-sdk-platform/eta-compute/*.cpp")
#file(GLOB REPL "${CMAKE_CURRENT_LIST_DIR}/repl/*.cpp")
#file(GLOB INGESTION "${CMAKE_CURRENT_LIST_DIR}/ingestion-sdk-c/*.cpp")
#file(GLOB QCBOR "${CMAKE_CURRENT_LIST_DIR}/QCBOR/src/*.c")
#file(GLOB MBEDTLS "${CMAKE_CURRENT_LIST_DIR}/mbedtls_hmac_sha256_sw/mbedtls/src/*.c")
#file(GLOB EIPORT "${CMAKE_CURRENT_LIST_DIR}/edge-impulse-sdk/porting/YOURTARGET/*.cpp")

RECURSIVE_FIND_FILE(SOURCE_FILES "edge-impulse-sdk" "*.cpp")
RECURSIVE_FIND_FILE(MODEL_FILES "tflite-model" "*.cpp")
RECURSIVE_FIND_FILE(CC_FILES "edge-impulse-sdk" "*.cc")
RECURSIVE_FIND_FILE(S_FILES "edge-impulse-sdk" "*.s")
RECURSIVE_FIND_FILE(C_FILES "edge-impulse-sdk" "*.c")

RECURSIVE_FIND_FILE(PLATFORM_FILES "edge-impulse/ingestion-sdk-platform/NordicSemi-nrf52" "*.cpp")
RECURSIVE_FIND_FILE(REPL_FILES "edge-impulse/repl" "*.cpp")
RECURSIVE_FIND_FILE(INGESTION_FILES "edge-impulse/ingestion-sdk-c/" "*.cpp")
RECURSIVE_FIND_FILE(QCBOR_FILES "edge-impulse/QCBOR/src" "*.c")
RECURSIVE_FIND_FILE(MBEDTLS_FILES "edge-impulse/mbedtls_hmac_sha256_sw/mbedtls/src" "*.c")
RECURSIVE_FIND_FILE(PORTING_FILES "edge-impulse-sdk/porting/zephyr" "*.cpp")


list(APPEND SOURCE_FILES ${S_FILES})
list(APPEND SOURCE_FILES ${C_FILES})
list(APPEND SOURCE_FILES ${CC_FILES})
list(APPEND SOURCE_FILES ${MODEL_FILES})

list(APPEND SOURCE_FILES ${PLATFORM_FILES})
list(APPEND SOURCE_FILES ${REPL_FILES})
list(APPEND SOURCE_FILES ${INGESTION_FILES})
list(APPEND SOURCE_FILES ${QCBOR_FILES})
list(APPEND SOURCE_FILES ${MBEDTLS_FILES})
list(APPEND SOURCE_FILES ${PORTING_FILES})

# add all sources to the project
target_sources(app PRIVATE ${SOURCE_FILES})
target_sources(app PRIVATE src/main.cpp)
