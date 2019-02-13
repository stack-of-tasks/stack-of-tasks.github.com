---
layout: page
title: Download & Install
category: Getting Started
---

## How to install the stack of tasks on your machine?

## Prerequisites
The current software architecture has been tested entirely on Ubuntu 16.04 LTS, Ubuntu 14.04 LTS, and Ubuntu 12.04 LTS.

A large part of the basic algorithms have been tested on Windows and Mac a while go, but there is no guarantee that it is still true.

We advise the use of robotpkg to install the Stack of Tasks. You can try the install from the source code but we rather
advice its [binary repository](http://robotpkg.openrobots.org/debian.html).
You have to use also the [wip (work in progress) part of robotpkg](http://robotpkg.openrobots.org/robotpkg-wip.html).
Please follow the section entitled "Binary packages in robotpkg/wip".

### Dependencies

The needed software are:
- git, CMake, boost, lapack, blas, python 2.7
- CMake
- gcc, g++
- Doxygen (and eventually LaTeX for mathematical formula rendering)
- Boost
- Eigen
- Blas, lapack
- ROS (fuerte, groovy, hydro, indigo).

Please note that most of the packages are ROS-independent, but some packages allow to link the SOT framework to ROS (e.g. dynamic_graph_bridge, redundant_manipulator_control)

## Binary installation

We are currently providing the stack of tasks through robotpkg on Ubuntu 16.04 LTS (amd64), and Ubuntu 14.04 LTS (amd64).

Please follow the instructions given [here](http://robotpkg.openrobots.org/debian.html) to access the package repository.

To install the package sot-core-v3 you can use
```bash
apt-get install robotpkg-sot-core-v3
```

## Source installation

The source installation can be realized through robotpkg.
Please follow the instructions given [here](http://robotpkg.openrobots.org/install.html) to get the package repositories.

You will also have to install the wip (work in progress) part of robotpkg. The installation procedure is described [here](http://robotpkg.openrobots.org/robotpkg-wip.html).

To compile and install the package **sot-core-v3** then you can type:

```bash
cd robotpkg/wip/sot-core-v3
make install
```
