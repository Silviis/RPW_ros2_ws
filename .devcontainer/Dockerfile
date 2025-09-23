
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
RUN apt update -y && apt install -y nano net-tools ncat zsh git curl openssh-server iproute2 libqt5serialport5-dev python3-pip python3-wheel

# Robot Software dependencies
RUN apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools \
        gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio libuvc-dev \
        libgflags-dev

# ZSH
USER $USERNAME
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended
RUN sed -i "s!robbyrussell!${ZSH_THEME}!g" /home/$USERNAME/.zshrc  # Replace default theme
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.zsh" >> /home/$USERNAME/.zshrc  # Source ROS installation
RUN echo "FILE=/workspaces/RPW_ros2_ws/install/setup.zsh && test -f \$FILE && source \$FILE" >> /home/$USERNAME/.zshrc  # Source ROS workspace
RUN echo "export NO_AT_BRIDGE=1" >> /home/$USERNAME/.zshrc  
RUN echo "# argcomplete for ros2 & colcon " >> /home/$USERNAME/.zshrc  
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> /home/$USERNAME/.zshrc  
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> /home/$USERNAME/.zshrc  
RUN echo "cd /workspaces/RPW_ros2_ws" >> /home/$USERNAME/.zshrc  # Change default directory to home directory
RUN echo "alias ws='cd /workspaces/RPW_ros2_ws'" >> /home/$USERNAME/.zshrc
RUN sudo chsh -s /bin/zsh $USERNAME
