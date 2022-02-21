MAKEFILE_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

latest: noetic

noetic:
	docker build -t cartographer_ros_benchmark:noetic -f ${MAKEFILE_DIR}/images/noetic/Dockerfile ${MAKEFILE_DIR}/..
	docker tag cartographer_ros_benchmark:noetic cartographer_ros_benchmark:latest

.PHONY: noetic
