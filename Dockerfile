# ======== ROS/Colcon Dockerfile ========
# This sample Dockerfile will build a Docker image for AWS RoboMaker
# in any ROS workspace where all of the dependencies are managed by rosdep.
#
# Adapt the file below to include your additional dependencies/configuration outside of rosdep.
# =======================================

# ==== Arguments ====
# Override the below arguments to match your application configuration.
# ===================

# ROS Distribution (ex: melodic, foxy, etc.)
ARG ROS_DISTRO=melodic
# Application Name (ex: helloworld)
ARG APP_NAME=robomaker_app
# Path to workspace directory on the host (ex: ./robot_ws)
ARG LOCAL_WS_DIR=workspace
# User to create and use (default: robomaker)
ARG USERNAME=robomaker
# The gazebo version to use if applicable (ex: gazebo-9, gazebo-11)
ARG GAZEBO_VERSION=gazebo-9
# Where to store the built application in the runtime image.
ARG IMAGE_WS_DIR=/home/$USERNAME/workspace

# ======== ROS Build Stages ========
# ${ROS_DISTRO}-ros-base
#   -> ros-robomaker-base
#      -> ros-robomaker-application-base
#         -> ros-robomaker-build-stage
#         -> ros-robomaker-app-runtime-image
# ==================================

# ==== ROS Base Image ============
# If running in production, you may choose to build the ROS base image
# from the source instruction-set to prevent impact from upstream changes.
# ARG UBUNTU_DISTRO=focal
# FROM public.ecr.aws/lts/ubuntu:${UBUNTU_DISTRO} as ros-base
# Instruction for each ROS release maintained by OSRF can be found here: https://github.com/osrf/docker_images
# ==================================

# ==== Build Stage with AWS RoboMaker Dependencies ====
# This stage creates the robomaker user and installs dependencies required to run applications in RoboMaker.
# ==================================

FROM public.ecr.aws/docker/library/ros:${ROS_DISTRO}-ros-base AS ros-robomaker-base
ARG USERNAME
ARG IMAGE_WS_DIR

RUN apt-get clean
RUN apt-get update && apt-get install -y \
    lsb  \
    unzip \
    wget \
    curl \
    xterm \
    python3-colcon-common-extensions \
    devilspie \
    xfce4-terminal

RUN groupadd $USERNAME && \
    useradd -ms /bin/bash -g $USERNAME $USERNAME && \
    sh -c 'echo "$USERNAME ALL=(root) NOPASSWD:ALL" >> /etc/sudoers'

USER $USERNAME
WORKDIR /home/$USERNAME

RUN mkdir -p $IMAGE_WS_DIR

# ==== ROS Application Base ====
# This section installs exec dependencies for your ROS application.
# Note: Make sure you have defined 'exec' and 'build' dependencies correctly in your package.xml files.
# ========================================
FROM ros-robomaker-base as ros-robomaker-application-base
ARG LOCAL_WS_DIR
ARG IMAGE_WS_DIR
ARG ROS_DISTRO
ARG USERNAME

WORKDIR $IMAGE_WS_DIR
COPY --chown=$USERNAME:$USERNAME $LOCAL_WS_DIR/src $IMAGE_WS_DIR/src

RUN sudo apt update && \
    rosdep update && \
    rosdep fix-permissions

# Note: This will install all dependencies.
# You could further optimize this by only defining the exec dependencies.
# Then, install the build dependencies in the build image.
RUN rosdep install --from-paths src --ignore-src -r -y

# ==== ROS Workspace Build Stage ====
# In this stage, we will install copy source files, install build dependencies and run a build.
# ===================================
FROM ros-robomaker-application-base AS ros-robomaker-build-stage
LABEL build_step="${APP_NAME}Workspace_Build"
ARG APP_NAME
ARG LOCAL_WS_DIR
ARG IMAGE_WS_DIR

RUN  . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
     --install-base $IMAGE_WS_DIR/$APP_NAME

# ==== ROS Robot Runtime Image ====
# In the final stage, we will copy the staged install directory to the runtime image.
# =================================
FROM ros-robomaker-application-base AS ros-robomaker-app-runtime-image
ARG APP_NAME
ARG USERNAME
ARG GAZEBO_VERSION

ENV USERNAME=$USERNAME
ENV APP_NAME=$APP_NAME
ENV GAZEBO_VERSION=$GAZEBO_VERSION

RUN rm -rf $IMAGE_WS_DIR/src

COPY --from=ros-robomaker-build-stage $IMAGE_WS_DIR/$APP_NAME $IMAGE_WS_DIR/$APP_NAME

# Add the application source file to the entrypoint.
WORKDIR /
COPY entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh && \
    sudo chown -R $USERNAME /entrypoint.sh && \
    sudo chown -R $USERNAME $IMAGE_WS_DIR/$APP_NAME

ENTRYPOINT ["/entrypoint.sh"]