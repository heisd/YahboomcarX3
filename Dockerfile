################################################################################
# Yahboomcar X3 — ROS 2 Humble development / test image
#
# Build:
#   docker build -t yahboomcar-x3 .
#
# Run closed-circuit tests (no hardware needed):
#   docker run --rm yahboomcar-x3 pytest src/yahboomcar_linefollow/test/ -v
#
# Run interactively (ROS2 + colcon build):
#   docker run --rm -it --privileged \
#     --device /dev/video0:/dev/video0 \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     yahboomcar-x3 bash
################################################################################

FROM ros:humble-ros-base AS base

SHELL ["/bin/bash", "-c"]

# ── System packages ────────────────────────────────────────────────────────────
# python3-opencv from Ubuntu Jammy ships OpenCV 4.5.4 which includes
# cv2.QRCodeDetector — no extra contrib/zbar packages needed.
RUN apt-get update -q && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-pip \
      python3-opencv \
      python3-numpy \
      ros-humble-geometry-msgs \
      ros-humble-std-msgs \
      ros-humble-sensor-msgs \
      ros-humble-cv-bridge \
      ros-humble-ament-index-python \
      ros-humble-launch-ros \
    && rm -rf /var/lib/apt/lists/*

# ── Python test runner ─────────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir pytest

# ── Copy workspace ─────────────────────────────────────────────────────────────
WORKDIR /ws
COPY . /ws/

# ── Build with colcon (symlink-install so params/ is directly accessible) ─────
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
      --packages-select yahboomcar_linefollow \
      --event-handlers console_cohesion+ \
    && echo "source /ws/install/setup.bash" >> /root/.bashrc

# ── Default command: run the closed-circuit test suite ─────────────────────────
# Tests use a cv2 stub so they run without a physical camera or display.
CMD ["python3", "-m", "pytest", "src/yahboomcar_linefollow/test/", "-v", "--tb=short"]
