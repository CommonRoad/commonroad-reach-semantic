FROM gitlab.lrz.de:5005/cps/rapid-prototyping-setup/ci-lite:1.0

# Anaconda
# Install miniconda
ARG CONDA_DIR=/opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p $CONDA_DIR

# Add conda to PATH
ENV PATH=$CONDA_DIR/bin:$PATH

RUN mkdir -p /etc/apt/keyrings && \
    wget -q -O - https://www.lrde.epita.fr/repo/debian.gpg | tee /etc/apt/keyrings/lrde-spot.gpg && \
    echo 'deb [signed-by=/etc/apt/keyrings/lrde-spot.gpg] http://www.lrde.epita.fr/repo/debian/ stable/' | tee -a /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y \
        clang-tidy clang-format \
        libblas3 liblapack3 liblapack-dev libblas-dev gfortran \
        libyaml-cpp-dev \
        libgl1 \
        libspot-dev \
        doctest-dev && \
    rm -rf /var/lib/apt/lists/*

# create conda environments for python versions
ARG PY_VERSIONS="3.10 3.11"
ENV CONDA_ENV_PREFIX="commonroad-py"
RUN for PY_VERSION in $PY_VERSIONS; do conda create -n "$CONDA_ENV_PREFIX$PY_VERSION" python=$PY_VERSION; done
