FROM mcr.microsoft.com/devcontainers/base:ubuntu-20.04

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
        clang-format \
        clang-tidy \
        cmake \
        g++ \
        gcc \
        gdb \
        libboost-container-dev \
        libgtest-dev \
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

# Install Eigen3
RUN git clone --depth 1 --branch 3.4.0 https://gitlab.com/libeigen/eigen.git /tmp/eigen \
    && cd /tmp/eigen \
    && cmake -B build -DCMAKE_BUILD_TYPE=Release -DEIGEN_BUILD_TESTING=OFF -DEIGEN_BUILD_DOC=OFF \
    && cmake --build build \
    && cmake --install build \
    && cd / \
    && rm -rf /tmp/eigen

# Install dlib
RUN git clone --depth 1 --branch v19.24.2 https://github.com/davisking/dlib.git /tmp/dlib \
    && cd /tmp/dlib \
    && cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_DOCS=OFF \
    && cmake --build build \
    && cmake --install build \
    && cd / \
    && rm -rf /tmp/dlib
