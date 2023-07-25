FROM gitlab.lrz.de:5005/cps/commonroad-reachable-set/deps:ci

RUN mkdir -p /etc/apt/keyrings && \
    wget -q -O - https://www.lrde.epita.fr/repo/debian.gpg | tee /etc/apt/keyrings/lrde-spot.gpg && \
    echo 'deb [signed-by=/etc/apt/keyrings/lrde-spot.gpg] http://www.lrde.epita.fr/repo/debian/ stable/' | tee -a /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y libspot-dev && \
    rm -rf /var/lib/apt/lists/*

# install drivability checker
ARG CRREACH_DIR="/reach"
ARG LOCAL_CRREACH_DIR="./commonroad-reachable-set"
WORKDIR $CRREACH_DIR
COPY $LOCAL_CRREACH_DIR/requirements.txt .
RUN for PY_ENV in cr310 cr39 cr38 cr37; do conda run -n $PY_ENV pip install -r ./requirements.txt; done
COPY $LOCAL_CRREACH_DIR .
RUN for PY_ENV in cr310 cr39 cr38 cr37; do conda run -n $PY_ENV CRDC_DIR="/dc" pip install .; done
