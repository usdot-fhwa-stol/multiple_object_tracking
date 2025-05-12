#!/bin/bash

set -ex

# Install dlib
git clone --depth 1 --branch v19.24.2 https://github.com/davisking/dlib.git /tmp/dlib
cd /tmp/dlib
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF \
    -DDLIB_NO_GUI_SUPPORT=TRUE \
    -DDLIB_JPEG_SUPPORT=FALSE \
    -DDLIB_WEBP_SUPPORT=FALSE \
    -DDLIB_PNG_SUPPORT=FALSE \
    -DDLIB_GIF_SUPPORT=FALSE \
    -DDLIB_USE_MKL_FFT=FALSE \
    -DDLIB_USE_FFMPEG=FALSE \
    -DDLIB_USE_CUDA=FALSE \
    -DLIB_IN_PROJECT_BUILD=FALSE \
    -DBUILD_DOCS=OFF
cmake --build build
cmake --install build
cd /
rm -rf /tmp/dlib

# Install units library
git clone --depth 1 --branch v2.3.3 https://github.com/nholthaus/units.git /tmp/units
cd /tmp/units
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_DOCS=OFF
cmake --build build
cmake --install build
cd /
rm -rf /tmp/units

# Install JSON library
git clone --depth 1 --branch v3.11.2 https://github.com/nlohmann/json.git /tmp/json
cd /tmp/json
cmake -B build  \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF \
    -DJSON_BuildTests=FALSE \
    -DJSON_Install=TRUE \
    -DBUILD_DOCS=OFF
cmake --build build
cmake --install build
cd /
rm -rf /tmp/json

apt update
DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends libboost-container-dev libeigen3-dev
