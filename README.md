### RPW_vislam

This project is divided into two main parts: the development environment setup for AMD64 architecture machines (e.g., typical desktop or laptop computers) and the deployment setup for ARM64 architecture machines (e.g., NVIDIA Jetson devices).

The development setup uses a single docker container that includes all necessary ROS2 packages and dependencies for both the camera interface and visual SLAM processing. This unified environment simplifies development and testing on AMD64 machines.

The deployment setup uses two Docker containers to run the application on the ARM64 architecture machine. One container is for the camera interface, and the other is for the visual SLAM processing. This makes it easier to manage the ROS packages inside the containers, as the camera drivers require specific configurations that are best handled in isolation.

Clone this repo (and its submodules) by running:

```bash
git clone --recurse-submodules git@github.com:Silviis/RPW_vislam.git
```

## Developing (on AMD64 architecture machine)
This repository is designed to be used with VSCode Dev Containers. Visual Studio Code will automatically detect the `devcontainer.json` file and prompt you to open the folder in a container. This allows you to develop in an environment that closely matches the target deployment environment.

## Deploying (on the ARM64 machine e.g. Jetson)
Docker compose is used to build and run the application on the target ARM64 machine. The provided `docker-compose.yml` file defines the services, networks, and volumes needed for the application.

To build and run the application, use the following commands:

```bash
docker-compose up -d
```

To stop the application, use:

```bash
docker-compose down
```

To get a bash shell inside a running container, use:

```bash
docker exec -it <container_name> /bin/bash
```

Replace `<container_name>` with the name of the container you want to access (either ros2_cam or ros2_slam).

