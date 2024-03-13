#! /bin/bash

REGISTRY=gitlab.lrz.de:5005
IMAGE=$REGISTRY/cps/commonroad/commonroad-reach-semantic/ci

# Update this when creating a new version of the image
TAG=2.0

docker login $REGISTRY
docker build -t $IMAGE:$TAG .
docker push $IMAGE:$TAG
docker tag $IMAGE:$TAG $IMAGE:latest
docker push $IMAGE:latest
docker logout $REGISTRY
