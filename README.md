# ROS2 GPS driver in Docker [![](https://img.shields.io/docker/pulls/frankjoshua/ros2-gps)](https://hub.docker.com/r/frankjoshua/ros2-gps) [![CI](https://github.com/frankjoshua/docker-ros2-gps/workflows/CI/badge.svg)](https://github.com/frankjoshua/docker-ros2-gps/actions)

## Description

Runs the GPS receiver driver in a Docker container — reads the serial GPS (`/dev/gps`) and publishes it into the ROS 2 graph. Needs `--network=host` for ROS 2 DDS discovery.

This repo is mostly an example of how to build a multi architecture docker container with ROS (Robotic Operating System). Github Actions is used to build multi-architecture images using `docker buildx` for amd64 (x86 Desktop PC) and arm64 (Jetson). This is for the purpose of developing locally on a work pc or laptop. Then being able to transfer your work to an embedded device with a high level of confidence of success.

## Example

```
docker run -it \
    --network="host" \
    --ipc=host \
    --privileged \
    frankjoshua/ros2-gps
```
ros2 topic pub /my_topic std_msgs/String "data: Hello, ROS 2!"

## Building

Use [build.sh](build.sh) to build the docker containers.

<br>Local builds are as follows:

```
./build.sh -t frankjoshua/ros2-gps -l
```

## Template

This repo is a GitHub template. Just change the repo name in [.github/workflows/ci.yml](.github/workflows/ci.yml) and edit [Dockerfile](Dockerfile) and [README.md](README.md) to taste.

## Testing

Github Actions expects the DOCKERHUB_USERNAME and DOCKERHUB_TOKEN variables to be set in your environment.

## License

Apache 2.0

## Author Information

Joshua Frank [@frankjoshua77](https://www.twitter.com/@frankjoshua77)
<br>
[http://roboticsascode.com](http://roboticsascode.com)
