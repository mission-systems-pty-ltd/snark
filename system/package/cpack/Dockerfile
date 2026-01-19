ARG version=24.04
FROM ubuntu:${version}
ENV DEBIAN_FRONTEND="noninteractive"

ARG comma_deb
COPY ${comma_deb} /tmp/comma.deb
RUN apt-get update \
    && PACKAGES="build-essential cmake devscripts git \
      libtbb-dev libzmq3-dev libexiv2-dev libassimp-dev \
      libboost-all-dev libeigen3-dev libopencv-dev libopencv-highgui-dev python3-opencv \
      qtbase5-dev qt5-qmake qt3d5-dev libqt5opengl5-dev libqt5charts5-dev \
      /tmp/comma.deb" \
    && apt-get install -y --no-install-recommends $PACKAGES \
    && rm -rf /var/lib/apt/lists/*

# Unpack and install Vimba.
ARG vimba_pkg
COPY $vimba_pkg /tmp/vimba.tgz
RUN tar -xzf /tmp/vimba.tgz -C /opt \
    && rm /tmp/vimba.tgz \
    && cd /opt/Vimba*/VimbaGigETL/ \
    && ./Install.sh

# Make git ok with folder ownership.
RUN git config --global --add safe.directory /mnt/snark
