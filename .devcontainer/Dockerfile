FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    git \
    libboost-filesystem-dev \
    libboost-system-dev \
    libeigen3-dev \
    libomp-dev \
    libpcl-dev \
    libtbb-dev \
    libyaml-cpp-dev \
    ninja-build \
    pkg-config \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
RUN useradd -m -s /bin/bash -G sudo developer && \
    echo "developer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer

WORKDIR /workspaces/M-detector

# Set ownership of workspace to the developer user
RUN chown -R developer:developer /workspaces/M-detector

# Switch to the developer user
USER developer
