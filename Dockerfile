
FROM dustynv/ros:humble-desktop-l4t-r32.7.1
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Add new user as a sudoer.
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# GStreamer dependencies
RUN apt install -y gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good

USER $USERNAME
RUN echo "source /opt/ros/${ROS_DISTRO}/install/setup.bash" >> /home/$USERNAME/.bashrc  # Source ROS installation
RUN echo "FILE=/workspaces/RPW_ros2_ws/install/setup.bash && test -f \$FILE && source \$FILE" >> /home/$USERNAME/.bashrc  # Source ROS workspace
RUN echo "cd /workspaces/RPW_ros2_ws" >> /home/$USERNAME/.bashrc
RUN sudo chsh -s /bin/bash ${USERNAME}
WORKDIR /workspaces/RPW_ros2_ws
