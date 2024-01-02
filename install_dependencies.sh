#!/bin/bash

set -ex
cd /tmp
git clone --depth 1 --branch 3.4.0 https://gitlab.com/libeigen/eigen.git /tmp/eigen
cd /tmp/eigen
cmake -B build -DCMAKE_BUILD_TYPE=Release -DEIGEN_BUILD_TESTING=OFF -DEIGEN_BUILD_DOC=OFF
cmake --build build
cmake --install build
cd /
rm -rf /tmp/eigen

apt update
DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends libboost-container-dev libdlib-dev nlohmann-json3-dev libgtest-dev libgmock-dev
