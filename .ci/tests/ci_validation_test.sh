#!/usr/bin/env bash

################################################################
# Tests for the CI validation helpers in .ci/ci_validation.sh
#
# These tests only run in the project_template repository
# itself (see template_tests.yml), not in projects created
# from the template.
#
# Usage:
#   bash .ci/tests/ci_validation_test.sh
################################################################

FAILURES=0

# Verify that a check accepts a value. The check runs in a subshell as a
# failing check calls exit.
expect_pass() {
  local check="$1"
  local value="$2"
  if ( "$check" TEST_VAR "$value" ) >/dev/null 2>&1; then
    echo "ok: $check accepts '$value'"
  else
    echo "FAIL: $check should accept '$value'" >&2
    FAILURES=$((FAILURES + 1))
  fi
}

# Verify that a check rejects a value.
expect_fail() {
  local check="$1"
  local value="$2"
  if ( "$check" TEST_VAR "$value" ) >/dev/null 2>&1; then
    echo "FAIL: $check should reject '$value'" >&2
    FAILURES=$((FAILURES + 1))
  else
    echo "ok: $check rejects '$value'"
  fi
}

. "$(dirname "$0")/../ci_validation.sh"

# check_non_empty
expect_pass check_non_empty "value"
expect_fail check_non_empty ""
expect_fail check_non_empty "   "

# check_not_root
expect_pass check_not_root "/workspaces"
expect_fail check_not_root "/"

# check_is_absolute_path
expect_pass check_is_absolute_path "/workspaces"
expect_fail check_is_absolute_path ".ci_cache"
expect_fail check_is_absolute_path ""

# check_is_relative_path
expect_pass check_is_relative_path ".ci_cache"
expect_fail check_is_relative_path "/workspaces"

# check_no_parent_traversal
expect_pass check_no_parent_traversal ".ci_cache"
expect_fail check_no_parent_traversal "../escape"
expect_fail check_no_parent_traversal "nested/../escape"

if [ "$FAILURES" -ne 0 ]; then
  echo "${FAILURES} test(s) failed" >&2
  exit 1
fi
echo "All tests passed"
