#!/usr/bin/env bash

############################################################
# Custom test file for industrial_ci
#
# Should contain the following functions:
#   - prepare_${filename} --> prepare_python_format_check
#   - run_${filename} --> run_python_format_check
#
# This file is not supposed to be run as is.
############################################################

function install_python_format_requirements() {
  # install pip3 if not installed yet
  ici_exec_for_command pip3 ici_install_pkgs_for_command pip3 python3-pip

  # rosdep needs to be able to install to the system packages directory for Python versions > 3.10
  # see https://docs.ros.org/en/independent/api/rosdep/html/pip_and_pep_668.html for more details
  local p=$(python3 -c 'import sys; print(sys.version_info[0] > 2 and sys.version_info[1] > 10)'>&1)
  if [ $p == "True" ]; then
    export PIP_BREAK_SYSTEM_PACKAGES=1
  fi

  # install all requirements for linting
  ici_asroot pip3 install -r .ci/python/requirements.txt
}

function prepare_python_format_check() {
  true
}

function run_python_format_check() {
  ici_step install_python_format_requirements install_python_format_requirements
  ici_step run_python_format_check .ci/python/lint.sh
}
