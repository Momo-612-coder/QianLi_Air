FROM osrf/ros:noetic-desktop-full-focal

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654 && \
    sed -i 's/archive.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list && \
    sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
    
ADD src /basic_dev/src/
ADD setup.bash /
COPY build_basic_dev_dev.sh /basic_dev/build_basic_dev_dev.sh
COPY waypoint.txt /basic_dev/waypoint.txt

RUN chmod +x /setup.bash

USER root

# RUN apt update && apt install -y wget gnupg2 software-properties-common && \
#     wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb && \
#     dpkg -i cuda-keyring_1.1-1_all.deb && \
#     sed -i 's/developer.download.nvidia.com/developer.download.nvidia.cn/g' /etc/apt/sources.list.d/*cuda*.list && \
#     apt update && \
#     apt install -y cuda-toolkit-12-2 libcudnn9-cuda-12

RUN apt update && apt install -y \
  wget tar git cmake build-essential \
  gfortran \
  libopenblas-dev liblapack-dev \
  coinor-libipopt-dev \
  python3-catkin-tools \
  ros-noetic-geographic-msgs \
  ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs \
  ros-noetic-image-transport ros-noetic-cv-bridge \
  ros-noetic-osqp-vendor \
  ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  libopencv-dev ros-noetic-libg2o libsuitesparse-dev net-tools libeigen3-dev libgoogle-glog-dev libarmadillo-dev

ENV PATH="/opt/cmake/bin:/usr/local/cuda-12.2/bin:${PATH}"

RUN cd /tmp && \
    wget -q https://github.com/Kitware/CMake/releases/download/v3.27.9/cmake-3.27.9-linux-x86_64.tar.gz && \
    tar -xzf cmake-3.27.9-linux-x86_64.tar.gz && \
    mv cmake-3.27.9-linux-x86_64 /opt/cmake && \
    rm cmake-3.27.9-linux-x86_64.tar.gz && \
    cmake --version

RUN cd /opt && \
    osqp_config_dir="$(dirname "$(find /opt/ros/noetic -name osqpConfig.cmake -print -quit)")" && \
    test -n "${osqp_config_dir}" && \
    git clone --depth 1 --branch v0.8.1 https://github.com/robotology/osqp-eigen.git && \
    cmake -S osqp-eigen -B osqp-eigen/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/osqp-eigen \
      -DCMAKE_PREFIX_PATH=/opt/ros/noetic \
      -Dosqp_ROOT=/opt/ros/noetic \
      -Dosqp_DIR="${osqp_config_dir}" && \
    cmake --build osqp-eigen/build -j$(nproc) && \
    cmake --install osqp-eigen/build && \
    rm -rf osqp-eigen-src osqp-eigen-build && \
    ldconfig

# RUN wget -q https://github.com/casadi/casadi/archive/refs/tags/3.6.7.tar.gz && \
#     tar -xzf 3.6.7.tar.gz && \
#     cmake -S casadi-3.6.7 -B casadi-3.6.7/build \
#       -DCMAKE_BUILD_TYPE=Release \
#       -DCMAKE_Fortran_COMPILER=/usr/bin/gfortran \
#       -DCMAKE_INSTALL_PREFIX=/opt/casadi \
#       -DWITH_IPOPT=ON && \
#     BUILD_JOBS="$(nproc)" && \
#     if [ "${BUILD_JOBS}" -gt 4 ]; then BUILD_JOBS=4; fi && \
#     cmake --build casadi-3.6.7/build -j"${BUILD_JOBS}" && \
#     cmake --install casadi-3.6.7/build && \
#     rm -rf 3.6.7.tar.gz casadi-3.6.7 && \
#     ldconfig

# # 下载并安装 onnxruntime-gpu v1.20.0 (支持 CUDA 12)
# RUN wget https://gh-proxy.com/https://github.com/microsoft/onnxruntime/releases/download/v1.20.0/onnxruntime-linux-x64-gpu-1.20.0.tgz && \
#     tar -zxvf onnxruntime-linux-x64-gpu-1.20.0.tgz && \
#     mv onnxruntime-linux-x64-gpu-1.20.0 /opt/onnxruntime && \
#     rm onnxruntime-linux-x64-gpu-1.20.0.tgz

RUN cd /opt && \
    git clone https://gh-proxy.com/https://github.com/Livox-SDK/Livox-SDK.git && \
    cd Livox-SDK && \
    mkdir -p build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install
    
ENV ROS_DISTRO=noetic
#ENV LD_LIBRARY_PATH="/opt/casadi/lib:/opt/onnxruntime/lib:/usr/local/cuda-12.2/lib64"
ENV LD_LIBRARY_PATH="/opt/osqp-eigen/lib:/usr/local/cuda-12.2/lib64"
ENV OsqpEigen_DIR="/opt/osqp-eigen/lib/cmake/OsqpEigen"
ENV CMAKE_PREFIX_PATH="/opt/osqp-eigen:/opt/ros/noetic"

WORKDIR /basic_dev

RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && ./build_basic_dev_dev.sh"

COPY entrypoint.sh /basic_dev/entrypoint.sh
RUN chmod +x /basic_dev/entrypoint.sh

ENTRYPOINT ["/basic_dev/entrypoint.sh"]
