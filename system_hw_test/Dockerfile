# Base builder ---------------------------
# FROM --platform=$BUILDPLATFORM rust:1.61 AS rust-builder
# docker build --platform=linux/arm64 .
# docker buildx build --platform linux/arm64 .

FROM alpine:latest

CMD ["/bin/sh"]

ARG TARGETPLATFORM=linux/amd64
ARG BINARY=zenoh-bridge-ros2dds

RUN TARGETPLATFORM=linux/amd64 BINARY=zenoh-bridge-ros2dds apk add --no-cache libgcc libstdc++ # buildkit

COPY linux/amd64/* / # buildkit

# ARG TARGETPLATFORM
# ARG BINARY

# RUN apk add --no-cache libgcc libstdc++

# COPY $TARGETPLATFORM/* /
# ENV BINARY $BINARY

# COPY linux/arm64/* / # buildkit

# RUN apt-get update && apt-get install -y \
#    g++-x86-64-linux-gnu libc6-dev-amd64-cross \
#    g++-aarch64-linux-gnu libc6-dev-arm64-cross && \
#    rm -rf /var/lib/apt/lists/*
# RUN rustup target add \
#    x86_64-unknown-linux-gnu aarch64-unknown-linux-gnu
# RUN rustup toolchain install \
#    stable-x86_64-unknown-linux-gnu stable-aarch64-unknown-linux-gnu
# RUN rustup component add rustfmt
# ENV CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_LINKER=x86_64-linux-gnu-gcc \
#    CC_x86_64_unknown_linux_gnu=x86_64-linux-gnu-gcc \
#    CXX_x86_64_unknown_linux_gnu=x86_64-linux-gnu-g++ \
#    CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc \
#    CC_aarch64_unknown_linux_gnu=aarch64-linux-gnu-gcc \
#    CXX_aarch64_unknown_linux_gnu=aarch64-linux-gnu-g++ \
#    CARGO_INCREMENTAL=0
 
# amd64 build ----------------------------
 
# FROM --platform=$BUILDPLATFORM rust-builder AS build-amd64
# WORKDIR /app
# COPY . .
# RUN cargo install --target x86_64-unknown-linux-gnu --path .
# RUN mv ./target/x86_64-unknown-linux-gnu/release/example-app /usr/bin/example-app
 
# arm64 build ----------------------------
 
# FROM --platform=$BUILDPLATFORM rust-builder AS build-arm64
# WORKDIR /app
# COPY . .
# RUN cargo install --target aarch64-unknown-linux-gnu --path .
# RUN mv ./target/aarch64-unknown-linux-gnu/release/example-app /usr/bin/example-app