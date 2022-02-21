MAKEFILE_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

latest: noetic

noetic:
	docker build -t lambkin:noetic -f ${MAKEFILE_DIR}/images/noetic/Dockerfile ${MAKEFILE_DIR}/..
	docker tag lambkin:noetic lambkin:latest

.PHONY: noetic
