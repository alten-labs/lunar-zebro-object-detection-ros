
docker/focal/build-and-test:
	docker build . -f ./docker/object-detection-ros-focal.Dockerfile -t object-detection-ros-focal/object-detection-ros-focal:latest-focal
	docker build . -f ./docker/test-object-detection-ros-focal.Dockerfile -t test

docker/focal/run:
	docker run -it object-detection-ros/object-detection-ros:latest-focal /bin/bash
