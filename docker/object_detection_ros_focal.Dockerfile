FROM ubuntu:20.04

# suppress warnings during apt-get calls
ARG DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# install of apt-utils suppresses bogus warnings
RUN apt-get update \
  && apt-get install -y apt-utils 2>&1 | grep -v "debconf: delaying package configuration, since apt-utils is not installed" \
  && apt-get install -y \
    build-essential \
    git \
    lsb-release \
    sudo \
    wget \
  && rm -rf /var/lib/apt/lists/*

# suppress detached head warnings
RUN git config --global advice.detachedHead false