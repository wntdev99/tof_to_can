#!/bin/bash
# Pico SDK 빌드 스크립트
# 사용법: ./build.sh [clean]

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

# ── Pico SDK 경로 자동 탐색 ────────────────────────────────
if [ -z "$PICO_SDK_PATH" ]; then
    CANDIDATES=(
        "$HOME/pico/pico-sdk"
        "$HOME/pico-sdk"
        "/opt/pico-sdk"
        "$SCRIPT_DIR/pico-sdk"
    )
    for c in "${CANDIDATES[@]}"; do
        if [ -f "$c/external/pico_sdk_import.cmake" ]; then
            export PICO_SDK_PATH="$c"
            echo "[BUILD] PICO_SDK_PATH 자동 설정: $PICO_SDK_PATH"
            break
        fi
    done
fi

if [ -z "$PICO_SDK_PATH" ]; then
    echo "[ERROR] Pico SDK를 찾을 수 없습니다."
    echo "  설치 방법:"
    echo "    cd ~ && mkdir pico && cd pico"
    echo "    git clone https://github.com/raspberrypi/pico-sdk.git --branch master"
    echo "    cd pico-sdk && git submodule update --init"
    echo "  그 후 다시 실행하십시오."
    exit 1
fi

# ── 클린 빌드 ──────────────────────────────────────────────
if [ "$1" = "clean" ]; then
    echo "[BUILD] 클린 빌드..."
    rm -rf "$BUILD_DIR"
fi

# ── cmake 구성 ─────────────────────────────────────────────
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake .. \
    -DPICO_SDK_PATH="$PICO_SDK_PATH" \
    -DCMAKE_BUILD_TYPE=Release \
    -DPICO_BOARD=adafruit_feather_rp2040 \
    2>&1

# ── 빌드 ───────────────────────────────────────────────────
CORES=$(nproc 2>/dev/null || echo 4)
make -j"$CORES" 2>&1

echo ""
echo "[BUILD] 완료: $BUILD_DIR/tof_to_can.uf2"
echo "  RP2040에 플래시:"
echo "  BOOTSEL 버튼을 누른 채로 USB 연결 → /media/\$USER/RPI-RP2/ 에 복사"
