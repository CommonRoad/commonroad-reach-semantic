TAG="ci-cpp-env-model"
docker login gitlab.lrz.de:5005
git clone -b development --recurse-submodules --single-branch --depth 1 git@gitlab.lrz.de:cps/commonroad-drivability-checker.git dc
git clone -b develop --recurse-submodules --single-branch --depth 1 git@gitlab.lrz.de:cps/commonroad-reachable-set.git reach
docker build --build-arg LOCAL_CRDC_DIR="./dc" --build-arg LOCAL_CRREACH_DIR="./reach" -t gitlab.lrz.de:5005/cps/commonroad/commonroad-reach-semantic/deps:$TAG .
rm -rf reach dc
docker push gitlab.lrz.de:5005/cps/commonroad/commonroad-reach-semantic/deps:$TAG
docker logout gitlab.lrz.de:5005
