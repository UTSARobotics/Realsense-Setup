FROM python:3.11-bookworm

# 1. Install System Tools
# 'sudo' is needed because we run as a non-root user but might need root privileges
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    gcc \
    g++ \
    sudo \
    curl \
    lsb-release \
    libxml2-dev \
    libxslt-dev \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# 2. Create a Non-Root User 'rover'
ARG UID=1000
RUN useradd -m -u ${UID} -s /bin/bash rover && \
    usermod -aG sudo rover && \
    echo "rover ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/rover

# 3. Setup ArduPilot (Internal Clone)
# We clone into /opt so it doesn't conflict with your code in /workspace
WORKDIR /opt
RUN git clone --recursive --depth 1 https://github.com/ArduPilot/ardupilot.git
RUN chown -R rover:rover /opt/ardupilot

# 4. Install Build Dependencies (Global)
# We install the specific versions ArduPilot needs manually to avoid script errors
RUN pip install --no-cache-dir \
    empy==3.3.4 \
    pexpect \
    future \
    pymavlink \
    mavproxy \
    requests

# 5. Build the Rover Simulator
USER rover
WORKDIR /opt/ardupilot
# Configure for Software-In-The-Loop (SITL)
RUN ./waf configure --board sitl
# Build the Rover binary
RUN ./waf rover

# 6. Add Sim Tools to Path
ENV PATH="/opt/ardupilot/Tools/autotest:/opt/ardupilot/build/sitl/bin:${PATH}"

# 7. Setup Your Project Environment
WORKDIR /workspace
# Copy your specific project requirements
COPY --chown=rover:rover requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy your python code
COPY --chown=rover:rover . .

# Default command (can be overridden by compose)
CMD ["/bin/bash"]
