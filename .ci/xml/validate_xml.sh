#!/usr/bin/env bash

# Exit on any error
set -e

# Download the latest schemata
curl -s https://nobleo.github.io/ros2_launch_validation/ros2_launch.xsd > /tmp/ros2_launch.xsd
curl -s http://download.ros.org/schema/package_format3.xsd > /tmp/package_format3.xsd

# Validate launch files
# Ignores python and yaml launch files even if they have the `.launch` extension
{
  find . -path "*/launch/*.xml" -print0
  find . -path "**/*.launch" -print0
} | sort -zu | while IFS= read -r -d '' file; do
  grep -q "^def generate_launch_description" "$file" && continue   # Ignore python launch files
  grep -q "^launch:" "$file" && continue   # Ignore yaml launch files
  xmllint --noout --schema /tmp/ros2_launch.xsd "$file"
done

# Validate package manifests
find . -path "**/package.xml" -not -path "./.industrial_ci/*" -exec xmllint --noout --schema /tmp/package_format3.xsd {} +
