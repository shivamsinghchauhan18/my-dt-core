# parameters
ARG REPO_NAME="dt-core"
ARG DESCRIPTION="Provides high-level autonomy and fleet-coordination capabilities"
ARG MAINTAINER="Andrea F. Daniele (afdaniele@duckietown.com)"
# pick an icon from: https://fontawesome.com/v4.7.0/icons/
ARG ICON="diamond"

# ==================================================>
# ==> Do not change the code below this line
ARG ARCH
ARG DISTRO=daffy
ARG DOCKER_REGISTRY=docker.io
ARG BASE_IMAGE=dt-ros-commons
ARG BASE_TAG=${DISTRO}-${ARCH}
ARG LAUNCHER=default
ARG BUILD_TARGET=production

# Multi-stage build configuration
# Stage 1: Base development environment
FROM ${DOCKER_REGISTRY}/duckietown/${BASE_IMAGE}:${BASE_TAG} as base

# recall all arguments
ARG DISTRO
ARG REPO_NAME
ARG DESCRIPTION
ARG MAINTAINER
ARG ICON
ARG BASE_TAG
ARG BASE_IMAGE
ARG LAUNCHER
# - buildkit
ARG TARGETPLATFORM
ARG TARGETOS
ARG TARGETARCH
ARG TARGETVARIANT

# check build arguments
RUN dt-build-env-check "${REPO_NAME}" "${MAINTAINER}" "${DESCRIPTION}"

# define/create repository path
ARG REPO_PATH="${CATKIN_WS_DIR}/src/${REPO_NAME}"
ARG LAUNCH_PATH="${LAUNCH_DIR}/${REPO_NAME}"
RUN mkdir -p "${REPO_PATH}" "${LAUNCH_PATH}"
WORKDIR "${REPO_PATH}"

# keep some arguments as environment variables
ENV DT_MODULE_TYPE="${REPO_NAME}" \
    DT_MODULE_DESCRIPTION="${DESCRIPTION}" \
    DT_MODULE_ICON="${ICON}" \
    DT_MAINTAINER="${MAINTAINER}" \
    DT_REPO_PATH="${REPO_PATH}" \
    DT_LAUNCH_PATH="${LAUNCH_PATH}" \
    DT_LAUNCHER="${LAUNCHER}"

# install apt dependencies
COPY ./dependencies-apt.txt "${REPO_PATH}/"
RUN dt-apt-install ${REPO_PATH}/dependencies-apt.txt

# install opencv
COPY ./assets/opencv/${TARGETARCH} /tmp/opencv
RUN /tmp/opencv/install.sh && python3 -m pip list | grep opencv

# install python3 dependencies
ARG PIP_INDEX_URL="https://pypi.org/simple/"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
COPY ./dependencies-py3.* "${REPO_PATH}/"
RUN dt-pip3-install "${REPO_PATH}/dependencies-py3.*"

# copy the source code
COPY ./packages "${REPO_PATH}/packages"

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${CATKIN_WS_DIR}/

# install launcher scripts
COPY ./launchers/. "${LAUNCH_PATH}/"
RUN dt-install-launchers "${LAUNCH_PATH}"

# define default command
CMD ["bash", "-c", "dt-launcher-${DT_LAUNCHER}"]

# store module metadata
LABEL org.duckietown.label.module.type="${REPO_NAME}" \
    org.duckietown.label.module.description="${DESCRIPTION}" \
    org.duckietown.label.module.icon="${ICON}" \
    org.duckietown.label.platform.os="${TARGETOS}" \
    org.duckietown.label.platform.architecture="${TARGETARCH}" \
    org.duckietown.label.platform.variant="${TARGETVARIANT}" \
    org.duckietown.label.code.location="${REPO_PATH}" \
    org.duckietown.label.code.version.distro="${DISTRO}" \
    org.duckietown.label.base.image="${BASE_IMAGE}" \
    org.duckietown.label.base.tag="${BASE_TAG}" \
    org.duckietown.label.maintainer="${MAINTAINER}"
# <== Do not change the code above this line
# <==================================================

# Stage 2: Development environment with additional tools
FROM base as development

# Development-specific environment variables
ENV DUCKIETOWN_ROOT="${SOURCE_DIR}"
ENV DUCKIETOWN_DATA="/tmp/duckietown-data"
ENV PYTHONPATH="${PYTHONPATH}:${REPO_PATH}/packages"
ENV BUILD_STAGE="development"

# Install development tools
RUN echo "$(date '+%Y-%m-%d %H:%M:%S') [BUILD] Installing development dependencies..." && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        gdb \
        valgrind \
        htop \
        vim \
        nano \
        git \
        curl \
        wget \
        tree \
        tmux \
        screen \
        iputils-ping \
        net-tools \
        ssh-client \
        rsync && \
    echo "$(date '+%Y-%m-%d %H:%M:%S') [BUILD] Development tools installed successfully" && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# macOS development support - X11 forwarding setup
RUN echo "$(date '+%Y-%m-%d %H:%M:%S') [BUILD] Setting up X11 forwarding for macOS development..." && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        x11-apps \
        x11-utils \
        x11-xserver-utils \
        xauth && \
    echo "$(date '+%Y-%m-%d %H:%M:%S') [BUILD] X11 forwarding setup completed" && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Development configuration
RUN echo 'config echo 1' > .compmake.rc && \
    echo "$(date '+%Y-%m-%d %H:%M:%S') [BUILD] Development configuration applied"

# Stage 3: Testing environment
FROM development as testing

ENV BUILD_STAGE="testing"

# Install testing dependencies
RUN echo "$(date '+%Y-%m-%d %H:%M:%S') [BUILD] Installing testing dependencies..." && \
    pip3 install --no-cache-dir \
        pytest>=6.0.0 \
        pytest-cov>=2.10.0 \
        pytest-xdist>=2.0.0 \
        coverage>=5.0.0 \
        mock>=4.0.0 \
        parameterized>=0.8.0 && \
    echo "$(date '+%Y-%m-%d %H:%M:%S') [BUILD] Testing dependencies installed successfully"

# Copy test configuration
COPY ./packages/*/tests/ "${REPO_PATH}/tests/" 2>/dev/null || echo "No test directories found"

# Stage 4: Production environment (optimized)
FROM base as production

ENV BUILD_STAGE="production"
ENV DUCKIETOWN_ROOT="${SOURCE_DIR}"
ENV DUCKIETOWN_DATA="/tmp/duckietown-data"

# Production optimizations
RUN echo "$(date '+%Y-%m-%d %H:%M:%S') [BUILD] Applying production optimizations..." && \
    # Remove unnecessary packages to reduce image size
    apt-get update && \
    apt-get autoremove -y && \
    apt-get autoclean && \
    rm -rf /var/lib/apt/lists/* && \
    # Clear pip cache
    pip3 cache purge 2>/dev/null || true && \
    # Remove development files
    find /usr/local/lib/python3.*/dist-packages -name "*.pyc" -delete && \
    find /usr/local/lib/python3.*/dist-packages -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true && \
    echo "$(date '+%Y-%m-%d %H:%M:%S') [BUILD] Production optimizations completed"

RUN echo 'config echo 1' > .compmake.rc

# Copy production scripts
COPY scripts/send-fsm-state.sh /usr/local/bin

# Final stage selection based on BUILD_TARGET
FROM ${BUILD_TARGET} as final

# Log final build stage
RUN echo "$(date '+%Y-%m-%d %H:%M:%S') [BUILD] Final build stage: ${BUILD_STAGE}" && \
    echo "$(date '+%Y-%m-%d %H:%M:%S') [BUILD] Build completed successfully" && \
    echo "Build stage: ${BUILD_STAGE}" > /tmp/build_info.txt && \
    echo "Build timestamp: $(date)" >> /tmp/build_info.txt
