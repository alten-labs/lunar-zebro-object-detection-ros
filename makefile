
docker/build-and-test:
	docker build . -f ./docker/object_detection_ros_focal.Dockerfile -t object-detection-ros/object-detection-ros:latest-focal

docker/run:
	docker run -it object-detection-ros/object-detection-ros:latest-focal /bin/bash