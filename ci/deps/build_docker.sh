docker login gitlab.lrz.de:5005
git clone -b develop --recurse-submodules --single-branch --depth 1 git@gitlab.lrz.de:cps/commonroad-reachable-set.git reach
docker build --build-arg LOCAL_CRREACH_DIR="./reach" -t gitlab.lrz.de:5005/cps/commonroad-reach-semantic/deps:ci .
rm -rf reach
docker push gitlab.lrz.de:5005/cps/commonroad/commonroad-reach-semantic/deps:ci
docker logout gitlab.lrz.de:5005
