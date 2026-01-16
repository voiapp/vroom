# Multi-stage build for better optimization
FROM node:18-slim AS vroom-builder

ARG VROOM_RELEASE=v0.0.1
ARG VROOM_EXPRESS_RELEASE=v0.12.0

# Install dependencies in a single layer
# GCC 13 is required for C++20 <format> header support
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    libssl-dev \
    libasio-dev \
    libpthread-stubs0-dev \
    curl \
    software-properties-common \
    && echo "deb http://deb.debian.org/debian testing main" >> /etc/apt/sources.list.d/testing.list \
    && apt-get update \
    && apt-get install -y -t testing g++-13 gcc-13 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100 \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100 \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Clone and build VROOM
WORKDIR /vroom
RUN git clone --depth 1 --branch ${VROOM_RELEASE} https://github.com/voiapp/vroom.git . \
    && git submodule init \
    && git submodule update \
    && cd src \
    && make -j$(nproc) \
    && cd .. \
    && rm -rf .git src obj

# Clone and setup VROOM-Express
WORKDIR /vroom-express
RUN git clone --depth 1 --branch ${VROOM_EXPRESS_RELEASE} https://github.com/VROOM-Project/vroom-express.git . \
    && npm install \
    && npm prune --production \
    && rm -rf .git

# Copy VROOM binary to VROOM-Express
RUN cp /vroom/bin/vroom /vroom-express/

# Final stage
FROM node:18-slim

# Install runtime dependencies
# Need glibc and libstdc++ from testing to match the GCC 13 compiled binary
RUN apt-get update && apt-get install -y \
    libssl3 \
    && echo "deb http://deb.debian.org/debian testing main" >> /etc/apt/sources.list.d/testing.list \
    && apt-get update \
    && apt-get install -y -t testing libc6 libstdc++6 \
    && rm /etc/apt/sources.list.d/testing.list \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Copy built artifacts from builder stage
COPY --from=vroom-builder /vroom-express /vroom-express
WORKDIR /vroom-express

# Copy configuration files
COPY ./build/vroom/vroom_express_config.yml /vroom-express/config.yml
COPY ./build/vroom/vroom_cli_config.yml /conf/config.yml

# Set environment variables
ENV VROOM_ROUTER=osrm
ENV OSRM_HOST=osrm
ENV OSRM_PORT=5000

# Expose port
EXPOSE 3000

# Start VROOM-Express
CMD ["node", "src/index.js"]
