#!/usr/bin/env bash

# Use ANSI escape codes for coloring the output
RED='\033[0;31m'
GREEN='\033[0;32m'
RESET='\033[0m'

# want all the commands to run, but fail the script if any command fails
exit_code=0

# lint the code (replaces flake8, isort, etc.)
(
    set -x
    ruff check .
) || exit_code=1

# check if formatting is correct (~ equivalent of `black format --check`)
(
    set -x
    ruff format --check --diff .
) || exit_code=1

if [ $exit_code -eq 1 ]; then
    echo -e "${RED}\nlint.sh: failed${RESET}"
    exit $exit_code
fi 

echo -e "${GREEN}\nlint.sh: passed${RESET}"
exit $exit_code
