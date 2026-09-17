#!/usr/bin/env bash

set -x

# lint the code (replaces flake8, isort, etc.) and fix where possible
ruff check --select I --fix .

# format the code
ruff format .
