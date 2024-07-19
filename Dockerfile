FROM osrf/ros:noetic-desktop
LABEL authors="chlee-rdv"
ARG DEBIAN_FRONTEND=noninteractive
# ----------------------------------------------------------------------------------------------
# 지역 설정
ENV TZ=Asia/Seoul
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y --no-install-recommends \
    vim git wget tar xz-utils \
    build-essential cmake g++ net-tools
# ----------------------------------------------------------------------------------------------
# Zivid SDK 설치

RUN apt-get update && apt-get install --assume-yes \
    wget \
    ocl-icd-libopencl1

RUN mkdir -p /etc/OpenCL/vendors && \
    echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility

RUN wget --quiet \
    https://downloads.zivid.com/sdk/releases/2.12.0+6afd4961-1/u20/amd64/zivid_2.12.0+6afd4961-1_amd64.deb \
    https://downloads.zivid.com/sdk/releases/2.12.0+6afd4961-1/u20/amd64/zivid-studio_2.12.0+6afd4961-1_amd64.deb \
    https://downloads.zivid.com/sdk/releases/2.12.0+6afd4961-1/u20/amd64/zivid-tools_2.12.0+6afd4961-1_amd64.deb \
    https://downloads.zivid.com/sdk/releases/2.12.0+6afd4961-1/u20/amd64/zivid-genicam_2.12.0+6afd4961-1_amd64.deb

RUN apt-get update
RUN apt-get install ./*.deb --assume-yes && rm ./*.deb
# ----------------------------------------------------------------------------------------------
# APT 패키지 리스트 삭제(이미지 크기를 줄이기 위함)
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
# ----------------------------------------------------------------------------------------------
ARG WORKSPACE="/root/catkin_ws"
COPY install ${WORKSPACE}/install

# ----------------------------------------------------------------------------------------------
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source ${WORKSPACE}/install/setup.bash" >> ~/.bashrc

WORKDIR ${WORKSPACE}
ADD src/entrypoint.sh /root/entrypoint.sh
ENTRYPOINT [ "/root/entrypoint.sh" ]

