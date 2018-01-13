#!/usr/bin/env bash

docker_id=$(docker ps | grep -E "\bcontrols_kit\b" | cut -c 1-12)
size=${#docker_id}

if [ $size -ne 12 ]; then
	printf "Error! Could not extract docker id properly from\n\n"
	echo $(docker ps | grep -E "\bcontrols_kit\b")
	printf "\nexit new terminal\n"
	exit
else
	echo "Trying to connect to container [$docker_id]..."
fi


if [ -z $docker_id ] 
then
	echo "Error! Can't connet to [controls_kit]"
	echo "Did you start this container?"
	exit
fi

docker exec -it $docker_id /bin/bash
