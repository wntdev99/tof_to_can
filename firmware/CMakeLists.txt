cmake_minimum_required(VERSION 3.13)

# Pico SDK 경로 (환경변수 또는 직접 지정)
if(DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
else()
    set(PICO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk")
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(tof_to_can C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ── ST ULD 소스 파일 목록 ─────────────────────────────────
# 각 ULD 디렉토리에 ST 공식 소스를 복사한 후 주석을 해제하십시오.
#
#  VL53L5CX: https://www.st.com/en/embedded-software/stsw-img023.html
#  VL53L4CD: https://www.st.com/en/embedded-software/stsw-img026.html
#  VL53L7CX: https://www.st.com/en/embedded-software/stsw-img039.html
#  VL53L8CX: https://www.st.com/en/embedded-software/stsw-img040.html
#
# 설치 방법:
#   ULD 압축 파일에서 VL53LxCX_ULD_API/Src/*.c 와 Inc/*.h 를
#   firmware/drivers/vl53lXcx/ 에 복사하고 아래 항목을 활성화하십시오.

set(ULD_L5CX_SRCS
    drivers/vl53l5cx/vl53l5cx_api.c
    drivers/vl53l5cx/vl53l5cx_plugin_detection_thresholds.c
    drivers/vl53l5cx/vl53l5cx_plugin_motion_indicator.c
    drivers/vl53l5cx/vl53l5cx_plugin_xtalk.c
)

set(ULD_L4CD_SRCS
    drivers/vl53l4cd/VL53L4CD_api.c
    drivers/vl53l4cd/VL53L4CD_calibration.c
)

set(ULD_L7CX_SRCS
    drivers/vl53l7cx/vl53l7cx_api.c
    drivers/vl53l7cx/vl53l7cx_plugin_detection_thresholds.c
    drivers/vl53l7cx/vl53l7cx_plugin_motion_indicator.c
    drivers/vl53l7cx/vl53l7cx_plugin_xtalk.c
)

set(ULD_L8CX_SRCS
    drivers/vl53l8cx/vl53l8cx_api.c
    drivers/vl53l8cx/vl53l8cx_plugin_detection_thresholds.c
    drivers/vl53l8cx/vl53l8cx_plugin_motion_indicator.c
    drivers/vl53l8cx/vl53l8cx_plugin_xtalk.c
)

# ── 실행 타겟 ─────────────────────────────────────────────
add_executable(tof_to_can
    main.c
    drivers/mcp25625/mcp25625.c
    drivers/st_platform_common.c
    drivers/vl53l5cx/platform.c
    drivers/vl53l5cx/vl53l5cx_driver.c
    drivers/vl53l4cd/platform.c
    drivers/vl53l4cd/vl53l4cd_driver.c
    drivers/vl53l7cx/platform.c
    drivers/vl53l7cx/vl53l7cx_driver.c
    drivers/vl53l8cx/platform.c
    drivers/vl53l8cx/vl53l8cx_driver.c
    drivers/tmf8828/tmf8828_driver.c
    sensor/sensor_manager.c
    can/can_tx.c
    utils/ring_buffer.c
    ${ULD_L5CX_SRCS}
    ${ULD_L4CD_SRCS}
    ${ULD_L7CX_SRCS}
    ${ULD_L8CX_SRCS}
)

target_include_directories(tof_to_can PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    drivers/vl53l5cx
    drivers/vl53l4cd
    drivers/vl53l7cx
    drivers/vl53l8cx
    drivers/tmf8828
)

target_compile_definitions(tof_to_can PRIVATE
    VL53L5CX_NB_TARGET_PER_ZONE=1
    VL53L7CX_NB_TARGET_PER_ZONE=1
    VL53L8CX_NB_TARGET_PER_ZONE=1
    # L5CX_DEBUG_SERIAL   # (검증 완료, 비활성) 시리얼로 8x8 거리 맵 출력
    # L4CD_DEBUG_SERIAL   # (검증 완료, 비활성) 시리얼로 L4CD 단일존 거리/상태 출력
)

target_link_libraries(tof_to_can
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_i2c
    pico_sync
)

# USB CDC 시리얼 활성화
pico_enable_stdio_usb(tof_to_can 1)
pico_enable_stdio_uart(tof_to_can 0)

pico_add_extra_outputs(tof_to_can)
