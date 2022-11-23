FROM mcr.microsoft.com/devcontainers/base:ubuntu-20.04

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
        clang-format \
        clang-tidy \
        cmake \
        g++ \
        gcc \
        gdb \
        libgtest-dev \
        libboost-container-dev \
        make \
    && rm -rf /var/lib/apt/lists/*

# Install units library
RUN git clone --depth 1 --branch v2.3.3 https://github.com/nholthaus/units.git /tmp/units \
    && cd /tmp/units \
    && cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_DOCS=OFF \
    && cmake --build build \
    && cmake --install build \
    && cd / \
    && rm -rf /tmp/units
