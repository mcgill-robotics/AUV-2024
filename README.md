# AUV-2020

*This project is currently under development*

Ahoy! This project contains software intended to run on a custom built AUV (model: Bradbury) to compete at RoboSub2022 on behalf of McGill Robotics. It runs using ROS and Arduino.

This project is maintained by the McGill Robotics Club and was developed by its members - students of McGill University. 
A number of ROS packages were included as git submodules (and thus fall under their respective licenses): 
- The `hydrophones`, `dvl`, and `teledyne_navigator` packages were built by former McGill Robotics Members (largely GPLv2)
- The `pointgrey_camera_driver` package is maintained by Mike Purvis, original repo [here](https://github.com/ros-drivers/pointgrey_camera_driver) (BSD)


## License

Since we are using GPLv2 dependancies, the project as a whole falls 
under GPLv2. The full license is provided inside [LICENSE](LICENSE).

Individual ROS packages (not submodules mentioned above) are under the MIT license. Other packages are subject to licenses set 
out by their respective maintainers (including GPLv2, BSD). 


## Getting Started

ROS requires Linux to run, the target operating system is Ubuntu 20.04 (LTS). For development it is sufficient to run a VM however, 
for testing it may be easier to use a native Ubuntu installation. For this we recommend having Ubuntu on an external drive (such as a USB) 
and booting off of that to minimize the risk of screwing something up on your machine. 

ROS is a framework/bunch-o-libraries that's handy for building/running/testing robotics software, it takes care of boring-but-necessary 
things such as; providing a way to send data between running programs, and providing a common build unit (package) that allows us to 
easily integrate software other people made into our project. Make sure you are using the version of ROS compatible with your OS, for 
Ubuntu 20 this is ROS Noetic.


### Dependencies

- Ubuntu 20.04 (Focal)
- ROS Noetic

*+ package specific dependecies*


## Installation 

Installing ROS (the general-purpose version):

    sudo apt-get install ros-noetic-desktop-full

To get these project files onto your computer using git navigate to a desired folder and do:

    git clone https://github.com/mcgill-robotics/AUV-2020.git
  
Alternatively, if you set up your SSH keys with your Github account you can save yourself having to type your user/pass every 
time (**recommended**):

    git clone git@github.com:mcgill-robotics/AUV-2020.git


## Building ROS Packages

*To build/launch/test and individual package, see the README.md in the respective package.*

The build the currently functional packages:

	source /opt/ros/noetic/setup.bash
	cd <AUV-2020>/catkin_ws/src
	catkin build planner controls propulsion

After build is complete, update ROS environment so that packages are 'visible':

	source ../devel/setup.bash
  
### Flashing Firmware

If you have an Arduino you can wire it up to emmulate the thrusters on the AUV (see schematic - **TODO**). Upload the 
binary file by connecting the Arduino via USB (currently hardcoded assumption USB shows up as /dev/ttyACM0) and running 
the generated make target:

    catkin build --no-deps  propulsion --make-args propulsion_embedded_thrusters-upload


## Running (on local machine)

Due to potential issues with start-up, enter each of the following commands  individually and wait for all processes for 
the package to be running prior to entering the next command.

    roslaunch controls controls.launch & 
    roslaunch propulsion propulsion.launch & 
    roslaunch planner mission.launch &


## Running (on AUV)
 
¯\\_(ツ)_/¯


## Contributing

To learn how to contribute to the project and how to get started see CONTRIBUTING.md (**TODO**)


## Known Issues

We (try) to keep track of current existing issues in `docs/known_issues.md`.

If you find something not-quite-right please create a new Github Issue outlining the problem or 
add it directly to `docs/known_issues.md` (**TODO**) and make a pull request to `noetic` branch.
