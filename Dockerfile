# Use Ubuntu 22.04 as base image
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV ROS_DISTRO=humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    curl \
    wget \
    git \
    build-essential \
    cmake \
    pkg-config \
    libeigen3-dev \
    libboost-all-dev \
    liburdfdom-dev \
    libassimp-dev \
    libhdf5-dev \
    libhdf5-serial-dev \
    libhdf5-103 \
    libhdf5-cpp-103 \
    libhdf5-dev \
    libhdf5-serial-dev \
    libhdf5-103 \
    libhdf5-cpp-103 \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && apt-get install -y \
        ros-humble-desktop \
        ros-humble-turtlebot3-gazebo \
        ros-humble-turtlebot3-navigation2 \
        ros-humble-nav2-bringup \
        python3-rosdep \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install conda for pinocchio
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh \
    && bash /tmp/miniconda.sh -b -p /opt/conda \
    && rm /tmp/miniconda.sh

# Add conda to PATH
ENV PATH="/opt/conda/bin:${PATH}"

# Install pinocchio via conda
RUN conda install -c conda-forge pinocchio -y

# Set working directory
WORKDIR /app

# Copy requirements first for better caching
COPY requirements.txt .

# Install Python dependencies
RUN pip3 install --no-cache-dir -r requirements.txt

# Copy the application code
COPY . .

# Make scripts executable
RUN chmod +x start.sh

# Create a non-root user
RUN useradd -m -s /bin/bash rcm_user && \
    chown -R rcm_user:rcm_user /app
USER rcm_user

# Expose port
EXPOSE 8000

# Set the default command
CMD ["./start.sh"]

