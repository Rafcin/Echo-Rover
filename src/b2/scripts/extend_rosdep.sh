#!/bin/bash
#
# This adds custom dependency instructions to rosdep. For example,
# to install a Python pip-installed package that is not currently known to rosdep
#
# This script should not need to be modified. Instead, just add the new rosdep definitions
# to a file named rosdep.yaml in the package directory (e.g. <package_dir>/rosdep.yaml)
#
# The format for the rosdep.yaml file is documented here:
#     http://docs.ros.org/independent/api/rosdep/html/rosdep_yaml_format.html
#
# Ths script is based off of the discussion found here:
#     https://github.com/ros-industrial/industrial_ci/issues/206

# Find our package directory
# Should be above the scripts/ directory where this script lives
PKG_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"/..

# Add local file to rosdep sources
echo "yaml file:///${PKG_DIR}/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/40-custom.list

# Update rosdep so it is aware of the new source
rosdep update
