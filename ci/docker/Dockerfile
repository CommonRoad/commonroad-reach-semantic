FROM debian:bullseye-slim

ENV DEBIAN_FRONTEND noninteractive

# Install system dependencies
RUN apt-get update && \
    apt-get install -y \
        wget git pkg-config \
        build-essential clang ninja-build \
        clang-format clang-tidy \
        ca-certificates wget \
        gcovr doxygen graphviz cppcheck \
        libomp-dev \
        libeigen3-dev \
        libboost-all-dev \
        libcgal-dev \
        libblas3 liblapack3 liblapack-dev libblas-dev gfortran \
        libyaml-cpp-dev \
        # libspdlog-dev \ # this specific version seems to cause issues in the CI, thus, we rather build it ourselves via FetchContent
        libgtest-dev \
        librange-v3-dev \
        libgl1 \
        doctest-dev && \
    mkdir -p /etc/apt/keyrings && \
    wget -q -O - https://www.lrde.epita.fr/repo/debian.gpg | tee /etc/apt/keyrings/lrde-spot.gpg && \
    echo 'deb [signed-by=/etc/apt/keyrings/lrde-spot.gpg] http://www.lrde.epita.fr/repo/debian/ stable/' | tee -a /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y libspot-dev && \
    rm -rf /var/lib/apt/lists/*

# Install newer version of CMake than provided by the standard repos
ARG CMAKE_VERSION=3.28.3
RUN wget "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-Linux-x86_64.sh" \
    -q -O /tmp/cmake-install.sh && \
    chmod u+x /tmp/cmake-install.sh && \
    mkdir /opt/cmake && \
    /tmp/cmake-install.sh --skip-license --prefix=/opt/cmake && \
    rm /tmp/cmake-install.sh
ENV PATH="/opt/cmake/bin:${PATH}"

# Install miniconda
ARG CONDA_VERSION="py312_24.1.2-0"
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-${CONDA_VERSION}-Linux-x86_64.sh -q -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda
ENV PATH="/opt/conda/bin:${PATH}"

# Create conda environments for Python versions
ARG PY_VERSIONS="3.10 3.11"
ENV CONDA_ENV_PREFIX="commonroad-py"
RUN for PY_VERSION in ${PY_VERSIONS}; do conda create -n "${CONDA_ENV_PREFIX}${PY_VERSION}" python=${PY_VERSION}; done
