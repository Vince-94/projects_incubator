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

# C++ SDL2 for visualization
RUN apt update && apt install -y \
    libsdl2-dev \
    libsdl2-image-dev \
    libsdl2-gfx-dev \
    libsdl2-ttf-dev

# Install OpenBLAS (high‑performance dense linear algebra)
RUN if dpkg --print-architecture | grep -q "arm"; then \
        apt update && apt install -y libopenblas-dev; \
    fi


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
