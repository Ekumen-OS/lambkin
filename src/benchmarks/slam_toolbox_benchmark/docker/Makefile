MAKEFILE_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

latest: noetic

noetic:
	docker build -t slam_toolbox_benchmark:noetic -f ${MAKEFILE_DIR}/images/noetic/Dockerfile ${MAKEFILE_DIR}/..
	docker tag slam_toolbox_benchmark:noetic slam_toolbox_benchmark:latest

.PHONY: noetic
