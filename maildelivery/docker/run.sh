#work reference https://nms.kcl.ac.uk/planning/software/optic.html

#!/bin/bash
IMAGE_TAG="optic"
CONTAINER_NAME="c-optic"

docker build -t ${IMAGE_TAG} .
docker run --name ${CONTAINER_NAME} -d ${IMAGE_TAG}

docker cp ${CONTAINER_NAME}:'/usr/src/plan.txt' .
docker stop $CONTAINER_NAME
docker rm $CONTAINER_NAME
