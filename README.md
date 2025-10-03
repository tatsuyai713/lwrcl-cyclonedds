# LWRCL (LightWeight Rclcpp Compatible Library)

This repository provides build scripts and samples for Cyclone DDS, designed to be compatible with ROS 2 topics. It serves as a bridge for developers looking to integrate Cyclone DDS with ROS 2 ecosystems, ensuring seamless communication and interoperability between systems using these technologies.

This library provides a simplified API similar to ROS 2's rclcpp for working with Cyclone DDS, enabling easier integration and management of nodes, publishers, subscribers, and timers within the Cyclone DDS ecosystem.

And also, this library enables to implement ROS 2 compatible applications with Cyclone DDS on lightweight SBCs such as Raspberry Pi.

## Features

- **Cyclone DDS Build Scripts:** Simplify the process of installing and setting up Cyclone DDS on Ubuntu/Debian and QNX systems.
- **ROS 2 Compatible Topics:** Includes samples that demonstrate how to publish and subscribe to ROS 2 topics using Cyclone DDS, facilitating integration into existing ROS 2 projects.
- **ROS Compatible Libraries:** Offers support for building and installing libraries crucial for ROS compatibility, such as yaml-cpp, ROS data types, and tf2.
- **lwrcl:** Please read `lwrcl/README.md`.

## How to Use This Repository

### Preparation: Remove ROS 2 Environment Setup

Remove the ROS 2 environment setup line from your ~/.bashrc:

```
source /opt/ros/humble/setup.bash
```

### Clone the Repository

Clone this repository and enter the directory:

```bash
git clone --recursive https://github.com/tatsuyai713/lwrcl-cyclonedds.git
cd lwrcl-cyclonedds
```


### Install Cyclone DDS

Install Cyclone DDS and necessary DDS packages on Ubuntu/Debian:

```bash
cd scripts
./build_cyclone.sh
```

### Build and Install ROS Data Types

```bash
./build_data_types.sh
```

### Build and Install LWRCL

Build and install the lwrcl for enhanced ROS 2 compatibility:

```bash
./build_lwrcl.sh
```

### Build and Install Libraries for ROS Compatibility

```bash
./build_libraries.sh
```

### Build LWRCL Sample Applications

Compile the lwrcl sample applications:

```bash
./build_apps.sh
```

The compiled applications can be found in the apps/install folder.

## Included Open Source Projects

This workspace includes or utilizes the following open-source projects:

- ROS Data Types: https://github.com/rticommunity/ros-data-types
- yaml-cpp: https://github.com/jbeder/yaml-cpp
- Fast-DDS: https://github.com/eProsima/Fast-DDS

This guide provides a comprehensive overview of setting up and using the Cyclone DDS / ROS 2 Compatible Workspace, ensuring that users can seamlessly integrate Cyclone DDS into their ROS 2 projects for efficient and interoperable communication.

