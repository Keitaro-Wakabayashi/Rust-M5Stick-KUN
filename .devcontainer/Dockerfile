# M5StickC Plus2 Rust Development Environment with AI
# ESP-IDF + Rust + Claude Code + MCP
#
# This Dockerfile creates a complete development environment for
# M5StickC Plus2 firmware development in Rust with AI assistance

FROM docker.io/espressif/idf:release-v5.5

ARG DEBIAN_FRONTEND=noninteractive
ARG CONTAINER_USER=esp
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install base packages
RUN apt-get update \
    && apt-get install -y -q \
    cmake \
    git \
    hwdata \
    libglib2.0-0 \
    libnuma1 \
    libpixman-1-0 \
    linux-tools-virtual \
    curl \
    wget \
    build-essential \
    pkg-config \
    libssl-dev \
    libudev-dev \
    libusb-1.0-0-dev \
    sudo \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Setup usbip
RUN update-alternatives --install /usr/local/bin/usbip usbip `ls /usr/lib/linux-tools/*/usbip | tail -n1` 20

# Install QEMU for ESP32 simulation
ENV QEMU_REL=esp-develop-20220919
ENV QEMU_SHA256=f6565d3f0d1e463a63a7f81aec94cce62df662bd42fc7606de4b4418ed55f870
ENV QEMU_DIST=qemu-${QEMU_REL}.tar.bz2
ENV QEMU_URL=https://github.com/espressif/qemu/releases/download/${QEMU_REL}/${QEMU_DIST}

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

RUN wget --no-verbose ${QEMU_URL} \
    && echo "${QEMU_SHA256} *${QEMU_DIST}" | sha256sum --check --strict - \
    && tar -xf $QEMU_DIST -C /opt \
    && rm ${QEMU_DIST}

ENV PATH=/opt/qemu/bin:${PATH}

# Install Rust for root user
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y \
    --default-toolchain stable \
    --profile minimal \
    --component rust-src
ENV PATH=/root/.cargo/bin:${PATH}

# Install Rust ESP toolchain essentials (for root)
ENV OPENSSL_NO_VENDOR=1
RUN cargo install cargo-espflash espflash ldproxy --locked || true

# Create user with proper permissions (handle existing UID/GID)
RUN EXISTING_USER=$(getent passwd $USER_UID | cut -d: -f1 || echo "") \
    && if [ -z "$EXISTING_USER" ]; then \
        # No existing user with this UID, create new one
        if ! getent group $USER_GID >/dev/null; then \
            groupadd --gid $USER_GID $CONTAINER_USER; \
        fi; \
        adduser --uid $USER_UID --gid $USER_GID --disabled-password --gecos "" ${CONTAINER_USER}; \
    elif [ "$EXISTING_USER" != "$CONTAINER_USER" ]; then \
        # Existing user with different name, rename it
        usermod -l $CONTAINER_USER $EXISTING_USER; \
        groupmod -n $CONTAINER_USER $(getent group $USER_GID | cut -d: -f1); \
        usermod -d /home/${CONTAINER_USER} -m $CONTAINER_USER; \
    fi \
    && usermod -a -G dialout,plugdev $CONTAINER_USER \
    && echo "${CONTAINER_USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/${CONTAINER_USER} \
    && chmod 0440 /etc/sudoers.d/${CONTAINER_USER}

# Pre-create Cargo directories with correct permissions
RUN mkdir -p /home/${CONTAINER_USER}/.cargo/registry \
    /home/${CONTAINER_USER}/.cargo/git \
    /home/${CONTAINER_USER}/.espressif \
    && chown -R ${CONTAINER_USER}:${CONTAINER_USER} /home/${CONTAINER_USER}

# Install Python packages for MCP servers
RUN pip3 install --no-cache-dir \
    mcp \
    pydantic \
    httpx \
    aiohttp || true

# Switch to esp user
USER ${CONTAINER_USER}
WORKDIR /home/${CONTAINER_USER}

# Install Rust for esp user
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y \
    --default-toolchain stable \
    --profile minimal \
    --component rust-src

ENV USER=${CONTAINER_USER}
ENV PATH=/home/${CONTAINER_USER}/.local/bin:/home/${CONTAINER_USER}/.cargo/bin:${PATH}

# Install Claude Code CLI (native binary)
# Native installation is preferred over Node.js version for:
# - Smaller image size (~200MB savings)
# - Faster installation and startup
# - No Node.js dependency issues
RUN curl -fsSL https://claude.ai/install.sh | bash \
    || echo "Claude Code installation failed, you can install it manually later"

# Add ~/.local/bin to PATH in .bashrc
RUN echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc

# Install espup and setup ESP Rust toolchain
# Use system OpenSSL to avoid build issues
ENV OPENSSL_NO_VENDOR=1
RUN cargo install espup --locked \
    && espup install \
    && echo '[ -f "$HOME/export-esp.sh" ] && . "$HOME/export-esp.sh"' >> ~/.bashrc

# Install cargo-espflash and other tools
RUN cargo install cargo-espflash espflash ldproxy --locked

# Setup ESP-IDF environment auto-load
RUN echo '[ -f "/opt/esp/idf/export.sh" ] && source /opt/esp/idf/export.sh > /dev/null 2>&1' >> ~/.bashrc

# Create Claude Code config directory
RUN mkdir -p /home/${CONTAINER_USER}/.claude

# Add useful aliases
RUN echo 'alias build="cargo build"' >> ~/.bashrc \
    && echo 'alias flash="cargo espflash flash"' >> ~/.bashrc \
    && echo 'alias monitor="cargo espflash monitor"' >> ~/.bashrc \
    && echo 'alias idf-build="idf.py build"' >> ~/.bashrc

# Set working directory to workspace
WORKDIR /workspace

# Expose ESP32 serial port (typical)
# Note: Actual device access is configured in devcontainer.json
