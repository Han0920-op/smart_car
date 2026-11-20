FROM osrf/ros:humble-desktop-full

# APT uses the Jammy archive. The ROS desktop underlay already exists in the base.
# An optional build secret lets APT and curl use the cloud runner's trusted CA.
RUN --mount=type=secret,id=apt_ca,mode=0444 \
    set -eu; \
    if [ -r /run/secrets/apt_ca ]; then \
        printf 'Acquire::https::CaInfo "/run/secrets/apt_ca";\n' > /etc/apt/apt.conf.d/99-build-ca; \
    fi; \
    sed -i -e 's|http://archive.ubuntu.com|https://archive.ubuntu.com|g' \
           -e 's|http://security.ubuntu.com|https://security.ubuntu.com|g' /etc/apt/sources.list; \
    apt-get -o Dir::Etc::SourceParts=- update; \
    DEBIAN_FRONTEND=noninteractive apt-get -o Dir::Etc::SourceParts=- install -y --no-install-recommends \
        gazebo libgazebo-dev; \
    rm -f /etc/apt/apt.conf.d/99-build-ca; \
    rm -rf /var/lib/apt/lists/*

# Build the upstream Gazebo Classic ROS plugins at a pinned release.
RUN --mount=type=secret,id=apt_ca,mode=0444 \
    set -eu; \
    if [ -r /run/secrets/apt_ca ]; then export CURL_CA_BUNDLE=/run/secrets/apt_ca; fi; \
    mkdir -p /tmp/classic_ws/src; \
    cd /tmp/classic_ws; \
    curl -fsSL --retry 3 -o gazebo.tar.gz https://codeload.github.com/ros-simulation/gazebo_ros_pkgs/tar.gz/refs/tags/3.9.0; \
    echo '3142775cea2c1ec16bbd840278356c570f37e42ecec4e2a254750a1959e485ee  gazebo.tar.gz' | sha256sum -c -; \
    tar -xf gazebo.tar.gz -C src; \
    set +u; . /opt/ros/humble/setup.sh; set -u; \
    MAKEFLAGS=-j4 colcon build --merge-install --install-base /opt/smartcar_classic \
        --executor sequential --packages-up-to gazebo_plugins \
        --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release; \
    rm -rf /tmp/classic_ws

ENV GAZEBO_MODEL_DATABASE_URI=""
ENV LIBGL_ALWAYS_SOFTWARE=1
