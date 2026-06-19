#!/bin/bash

# Command to be run within the "postCreateCommand" field of devcontainer.json
# Note that this script is intended to run in a humble ROS container.

set -e  # exit on first error

# Install and setup git
apt update
apt install -y git
git config --global --add safe.directory /workspace/src

# Ensure system dependencies are installed
rosdep update
rosdep install --from-paths /workspace/src --ignore-src -y

# Install the local python packages
pip install -e /workspace/src/schunk_gripper_library --upgrade
pip install -e /workspace/src/schunk_gripper_dummy --upgrade

# Install and setup pre-commit
pip install pre-commit
cd /workspace/src
pre-commit install
cd /workspace

# Build the ROS workspace
source /opt/ros/humble/setup.bash
cd /workspace
colcon build --symlink-install
# Overlay the production workspace
source /workspace/install/setup.bash
echo 'source /workspace/install/setup.bash' >> /etc/bash.bashrc

echo "post_create_command.sh completed."
