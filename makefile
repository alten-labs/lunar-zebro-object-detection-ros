
docker/focal/build-and-test:
	docker build . -f ./docker/object-detection-ros-focal.Dockerfile -t object-detection-ros/object-detection-ros:latest-focal

docker/focal/run:
	docker run -it object-detection-ros/object-detection-ros:latest-focal /bin/bash