#!/usr/bin/env bash

################################################################
# CI validation helpers to run as part of the before_script.
#
# Purpose:
# - Fail fast when required CI variables are missing or unsafe.
#
# Usage (example):
#   check_non_empty BASEDIR "${BASEDIR}"
#   check_is_absolute_path BASEDIR "${BASEDIR}"
#   check_is_relative_path CI_CACHE_DIR "${CI_CACHE_DIR}"
################################################################

# Print error and stop CI immediately.
fail() { echo "ERROR: $1" >&2; exit 1; }

# Guard against non-empty or not set values.
check_non_empty() {
  local name="$1"
  local value="$2"
  [[ "$value" =~ [^[:space:]] ]] || fail "$name must be non-empty"
}

# Guard against dangerous root-directory usage.
check_not_root() {
  local name="$1"
  local value="$2"
  [ "$value" != "/" ] || fail "$name must not be '/'"
}

# Guard against unexpected relative paths.
check_is_absolute_path() {
  local name="$1"
  local value="$2"
  case "$value" in
    /*) ;;
    *) fail "$name must be an absolute path" ;;
  esac
}

# Guard against unexpected absolute paths.
check_is_relative_path() {
  local name="$1"
  local value="$2"
  case "$value" in
    /*) fail "$name must be a relative path (must not start with '/')" ;;
  esac
}

# Guard against parent-directory traversal segments.
check_no_parent_traversal() {
  local name="$1"
  local value="$2"
  case "$value" in
    *".."*) fail "$name must not contain '..'" ;;
  esac
}
