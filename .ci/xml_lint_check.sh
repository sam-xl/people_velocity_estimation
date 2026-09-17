#!/usr/bin/env bash

############################################################
# Custom test file for industrial_ci
#
# Should contain the following functions:
#   - prepare_${filename} --> prepare_xml_lint_check
#   - run_${filename} --> run_xml_lint_check
#
# This file is not supposed to be run as is.
############################################################

function install_xml_lint_requirements() {
  # install xmllint and curl
  ici_install_pkgs_for_command xmllint libxml2-utils
  ici_install_pkgs_for_command curl curl
}

function prepare_xml_lint_check() {
  true
}

function run_xml_lint_check() {
  ici_step install_xml_lint_requirements install_xml_lint_requirements
  ici_step run_xml_lint_check .ci/xml/validate_xml.sh
}
