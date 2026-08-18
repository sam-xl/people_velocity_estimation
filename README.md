# <PACKAGE_NAME>
TODO: Add badges (build status, license)

## Overview 
TODO: Add a general description of the repository and the functionality it provides.

**Authors: [George Sotirchos](https://github.com/gsotirchos-samxl) ([SAM XL](https://github.com/sam-xl))**

**Maintainers: George Sotirchos, G.Sotirchos@tudelft.nl** 

This package hase been developed and tested under <ROS_VERSION> and <UBUNTU_VERSION>.

## Installation

### Dependencies
- TODO: List of dependencies (and version) which rosdep cannot solve. It is expected from the user to have these installed beforehand.

### Building
To build from source, clone the lastest version from this repository into your workspace:

```bash
cd ros_ws/src
git clone <PACKAGE_URL>
```

Use [`vcstool`](https://github.com/dirk-thomas/vcstool) to clone other project packages and complete the project workspace:

```bash
cd ros_ws/src
vcs import < <PACKAGE_NAME>/.repos
```

Install the dependencies of the cloned packages using `rosdep`:

```bash
cd ros_ws/src
rosdep install --from-paths . --ignore-src -y
```

Finally, build all packages in the workspace:

```bash
colcon build
```

## Usage
TODO: Add a description of a basic usage example for this package

```bash
roslaunch <package_name> sample_1.launch
```

## Launch files
* **`sample_1.launch`**: TODO: Add description ...

* **`sample_2.launch`**: TODO: Add description ...

## Nodes

### `node_name`
TODO: Add a description of the node and its functionality 

#### Parameters 
* `parameter_name` (type)\
TODO: Add description ...

#### Published Topics
* `/topic_name` ([message/type](link/to/msg/file))\
TODO: Add description ...

#### Subscribed Topics
* `/topic_name` ([message/type](link/to/msg/file))\
TODO: Add description ...

#### Services 

* `service_name` ([message/type](link/to/msg/file))\
TODO: Add description ...

#### Actions 

* `action_name` ([message/type](link/to/msg/file))\
TODO: Add description ...

## Detailed Documentation
Other than the brief introduction in this page, you can also check the detailed documentation [here](./docs/).

## Bugs & Feature Requests
Please report bugs and request features using the [Issue Tracker](<PACKAGE_ISSUE_URL>).

