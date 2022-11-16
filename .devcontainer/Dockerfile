FROM mcr.microsoft.com/devcontainers/base:jammy

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
        clang-format \
        clang-tidy \
        cmake \
        gdb \
    && rm -rf /var/lib/apt/lists/*
