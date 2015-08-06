RoCKIn@Work Central Factory Hub (CFH) Examples
==============================================

This repository contains very simple examples of how a robot would communicate with the RoCKIn Central Factory Hub.

  - TODO: cpp example 
  - TODO: roscpp example

about git submodules
--------------------

!!Important!! This project uses git submodules, and those submodules contain git submodules. 
Using git 1.6.5 or later this is easy to manage with one command:

    git clone --submodule
    
or 

    git clone
    git submodule update --init --recursive

Older versions of git need to

    git clone at_work_central_factory_hub_examples
    cd at_work_central_factory_hub_examples
    git submodule init
    git submodule update
    cd at_work_central_factory_hub_comm
    git submodule init
    git submodule update

building with cmake
-------------------

    mkdir build
    cd build
    cmake ..
    make

building with ROS catkin_make
-----------------------------

TODO: this is unfinished, untested, and under development.   
This should work "off the shelf", just clone into a working catkin workspace and run catkin_make.

TODO:
Look into cmake & ros, no catkin_make solution. 
for example (again untested)

    mkdir build
    cd build
    cmake -DWithROS=True ..   
