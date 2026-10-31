# IFS-DV-SANDBOX image: ROS 2 Humble + the IFSSIM bridge + Python tooling.
#
# Two workspaces:
#   /opt/ifs_ws        "underlay": fs_msgs + ifssim_bridge + the IFS09 code
#                      (ifs09_cone_detection), built once here.
#                      You never need to rebuild it.
#   /sandbox/ros2_ws   "overlay": your code (bind-mounted from the repo).
#                      Build it with `sb-build` inside the container.

FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-pip \
        python3-numpy \
        python3-scipy \
        python3-matplotlib \
        python3-pytest \
        cmake build-essential git \
        ros-humble-foxglove-bridge \
        ros-humble-tf2-ros \
        ros-humble-tf2-geometry-msgs \
        ros-humble-nav-msgs \
        ros-humble-sensor-msgs-py \
        ros-humble-visualization-msgs \
        ros-humble-rosbag2-storage-mcap \
    && ln -sf /usr/bin/pytest-3 /usr/local/bin/pytest \
    && rm -rf /var/lib/apt/lists/*

# The real pipeline's perception (perception:=ifs09) needs Numba and
# scikit-learn. Same versions as the IFS09 pipeline image (IFSSIM repo,
# docker/dv_pipeline_stack), so its code runs here exactly as there.
# Keep this list in sync with .github/workflows/ci.yml.
RUN pip install --no-cache-dir \
        "numpy==1.24.4" "scipy==1.15.3" "numba==0.67.0" "scikit-learn==1.7.2"
# Numba's compiled-code cache (a Docker volume, see docker-compose.yml), so
# the 10-20 s compile happens once, not at every start.
ENV NUMBA_CACHE_DIR=/numba_cache

# ---- Underlay: sim bridge + messages + IFS09 code (do not edit, see vendor/README.md) ----
WORKDIR /opt/ifs_ws
COPY vendor/fs_msgs       src/fs_msgs
COPY vendor/ifssim_bridge src/ifssim_bridge
COPY vendor/ifs09_cone_detection src/ifs09_cone_detection
RUN . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

COPY docker/fastdds_profile.xml /opt/fastdds_profile.xml
COPY docker/entrypoint.sh       /entrypoint.sh
# Shell helpers (sb-build, sb-launch, sb-new, sim) are read from the mounted
# repo, so editing docker/sandbox_shell.sh needs no image rebuild.
RUN chmod +x /entrypoint.sh \
    && echo '[ -f /sandbox/docker/sandbox_shell.sh ] && . /sandbox/docker/sandbox_shell.sh' >> /etc/bash.bashrc

WORKDIR /sandbox/ros2_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
