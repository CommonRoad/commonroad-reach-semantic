FROM gitlab.lrz.de:5005/cps/rapid-prototyping-setup/ci-lite:1.0

# Anaconda
# Install miniconda
ARG CONDA_DIR=/opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p $CONDA_DIR

# Add conda to PATH
ENV PATH=$CONDA_DIR/bin:$PATH

# install external dependencies
RUN mkdir -p /etc/apt/keyrings && \
    wget -q -O - https://www.lrde.epita.fr/repo/debian.gpg | tee /etc/apt/keyrings/lrde-spot.gpg && \
    echo 'deb [signed-by=/etc/apt/keyrings/lrde-spot.gpg] http://www.lrde.epita.fr/repo/debian/ stable/' | tee -a /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y \
    git \
    build-essential \
    cmake \
    clang clang-tidy clang-format \
    unzip \
    libblas3 liblapack3 liblapack-dev libblas-dev gfortran \
    libboost-dev \
    libboost-thread-dev \
    libboost-test-dev \
    libboost-filesystem-dev \
    libeigen3-dev \
    libomp-dev \
    libcgal-dev \
    libyaml-cpp-dev \
    libgl1 \
    libspot-dev \
    doctest-dev && \
    rm -rf /var/lib/apt/lists/*

# create conda environments for python versions
ARG PY_VERSIONS="3.10"
ARG CONDA_ENV_PREFIX="commonroad-py"
RUN for PY_VERSION in $PY_VERSIONS; do conda create -n "$CONDA_ENV_PREFIX$PY_VERSION" python=$PY_VERSION; done

# install commonroad-drivability-checker
ARG CRDC_DIR="/dc"
ARG LOCAL_CRDC_DIR="./commonroad-drivability-checker"
WORKDIR $CRDC_DIR
COPY $LOCAL_CRDC_DIR/requirements.txt .
RUN for PY_VERSION in $PY_VERSIONS; do conda run -n "$CONDA_ENV_PREFIX$PY_VERSION" pip install -r ./requirements.txt; done
ARG BUILD_JOBS=4
COPY $LOCAL_CRDC_DIR .
RUN mkdir build.bak && \
    for PY_VERSION in $PY_VERSIONS; do conda run -n "$CONDA_ENV_PREFIX$PY_VERSION" bash build.sh -j $BUILD_JOBS --cgal -i && mv build/*-cpython-*/ build.bak; done && \
    rm -r build && \
    mv build.bak build

# install commonoroad-reachable-set
ARG CRREACH_DIR="/reach"
ARG LOCAL_CRREACH_DIR="./commonroad-reachable-set"
WORKDIR $CRREACH_DIR
COPY $LOCAL_CRREACH_DIR/requirements.txt .
RUN for PY_VERSION in $PY_VERSIONS; do conda run -n "$CONDA_ENV_PREFIX$PY_VERSION" pip install -r ./requirements.txt; done
COPY $LOCAL_CRREACH_DIR .
RUN for PY_VERSION in $PY_VERSIONS; do conda run -n "$CONDA_ENV_PREFIX$PY_VERSION" CRDC_DIR=$CRDC_DIR pip install .; done
