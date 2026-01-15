
FROM osrf/ros:humble-desktop-full
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Set your ZSH theme here
ARG ZSH_THEME=agnoster

# Add new user as a sudoer.
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# General tools & libraries
RUN apt update && apt install -y nano net-tools ncat zsh git curl iproute2 python3-pip python3-wheel

# GStreamer dependencies
RUN apt install -y gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good

USER $USERNAME
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc  # Source ROS installation
RUN echo "FILE=/workspaces/RPW_vislam/cam_ws/install/setup.bash && test -f \$FILE && source \$FILE" >> /home/$USERNAME/.bashrc  # Source cam_ws
RUN echo "FILE=/workspaces/RPW_vislam/slam_ws/install/setup.bash && test -f \$FILE && source \$FILE" >> /home/$USERNAME/.bashrc  # Source slam_ws
RUN sudo chsh -s /bin/bash ${USERNAME}
