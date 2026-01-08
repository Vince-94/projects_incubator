FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive


########################
# INSTALL DEPENDENCIES #
########################

# Update
RUN apt update && apt upgrade -y

RUN apt install --quiet -y \
    sudo software-properties-common lsb-release
RUN add-apt-repository universe

# Ubuntu deps
RUN apt install --quiet -y \
    apt-transport-https ca-certificates gnupg gnupg2 gnupg-agent libssl-dev \
    git curl file wget zip unzip pkg-config openssh-client openssh-server \
    xterm tmux gdb valgrind \
    htop net-tools nmap lynx \
    libx11-dev vim iputils-ping

RUN apt install --quiet -y \
    bash-completion
RUN echo "source /usr/share/bash-completion/completions/git" >> ~/.bashrc

# C++ deps
RUN apt install --quiet -y \
    build-essential clang cmake ccache ninja-build

# C++ linter
RUN apt install --quiet -y \
    clang-tidy clang-tools \
    cppcheck \
    astyle

# C++ testing deps
RUN apt install --quiet -y libgtest-dev libgmock-dev lcov

# C++ common lib
RUN apt install --quiet -y \
    libboost-all-dev \
    libeigen3-dev

# Python deps
RUN apt install --quiet -y \
    python3 python3-dev python3-pip python3-future

# Python linter
RUN apt install --quiet -y pylint

# Python test libs
RUN apt install --quiet -y python3-pytest

# Python utils libs
RUN apt install --quiet -y \
    python3-pandas

RUN apt install --quiet -y \
    python3-pandas \
    python3-jsonschema \
    python3-jinja2 \
    python3-toml \
    python3-colorlog

# Install OpenBLAS (high‑performance dense linear algebra)
RUN if dpkg --print-architecture | grep -q "arm"; then \
        apt update && apt install -y libopenblas-dev; \
    fi


# Install PROXQP
RUN git clone --recurse-submodules https://github.com/Simple-Robotics/proxsuite.git
RUN cd proxsuite
WORKDIR proxsuite
RUN mkdir build

# WORKDIR proxsuite/build
# RUN cmake .. -DCMAKE_BUILD_TYPE=Release \
#     -DBUILD_TESTING=OFF \
#     -DBUILD_PYTHON_INTERFACE=OFF
# RUN make -j$(nproc)
# RUN sudo make install




#! Tail
# Clean
USER root
RUN apt -y autoremove && apt clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# Set user
USER $CONTAINER_USR
WORKDIR /home/$CONTAINER_USR

# Set entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

ENV force_color_prompt=yes
CMD ["bash"]
