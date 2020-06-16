# Nuclear Robotics Group (NRG)- Autonmous Mobile Survey Software

The NRG-Mobile respository contains source code for the execution and support of large area radiation survey tasks with an autonomous mobile robotics system. It is created under the copyright disclosure record C19007: "Nuclear Robotics Group: Autonomous Mobile Survey System." More information can be found in the [LICENSE.md](https://github.com/lanl/NRG-Mobile/blob/master/LICENSE.md) file.

## Contents

The NRG-Mobile repository is divided into two metapackages:
* alpha_sweep - Contains packages specific to the execution of large area alpha radiation surveys by an autonomous mobile robot.
    * robot_interface - Wrapper for robot driver commands in order to facilitate hardware agnostic operations.  
    * full_coverage - Primary operational package. First, generates a plan to navigate the robot in a way that fully covers the operation space. Second, executes coverage plan while maintaining safe conditions with respect to obstacles and radiation. Finally, outputs results of survey. 
    * sweep_gui - User facing graphical interface for interaction with the robot prior, during, and after survey task.
* navigation_tools - Contains packages that support generic navigation tasks in a semi-controlled environment.
    * rolling_map - ROS-based tool for processing 3D sensor information into maps for complex path planning and obstacle avoidance in semi-structured environments.
    * markers_to_map - Tool for the projection of 3D obstacle information into 2-dimensional images/maps for computationally efficient path planning.
    * clear_area_recovery - Modified ROS navigation stack recovery behavior. For details see [here](http://wiki.ros.org/clear_costmap_recovery?distro=melodic).
    * rotate_theta_recovery - Modified ROS navigation stack recovery behavior. For details see [here](http://wiki.ros.org/rotate_recovery?distro=melodic).
    * velodyne - Submodule for drivers to operate a Velodyne VLP-16 3D LIDAR. 
    * velodyne_orient - Tools to orient sensor information and clean the data for more efficient processing. 
* temporal_dev - Development packages for spatio-temporal world modeling.
    * bayesian_stats - Handles bayesian statistics involved with spatio-temporal mapping, including interfacing with PyMC3.
    * map_generation - Testing package for generating simulated world spaces and obstacles.
    * temporal_navigation - Main package for handling spatio-temporal map maintenance logic.
    * topo_mapping - Framework for generic topological map. Plans to develop a room based topological hierarchy for memory considerations.
* ubiquity - Packages relating to the specific hardware platform from [Ubiquity Robotics](https://www.ubiquityrobotics.com/). Includes the main software packages as a submodule from the Ubiquity Magni Github [repository](https://github.com/UbiquityRobotics). 

## Instructions for Running

The NRG-Mobile project is dependent on ROS. ROS only runs in a Linux environment, nominally Ubuntu. The current version of NRG-Mobile supports ROS melodic. The following preliminary setup is required before downloading NRG-Mobile.

### Setup

1. Install [Ubuntu 18.04](https://ubuntu.com/download/desktop). Many [guides](https://www.linuxtechi.com/ubuntu-18-04-lts-desktop-installation-guide-screenshots) [exist](https://www.forbes.com/sites/jasonevangelho/2018/08/29/beginners-guide-how-to-install-ubuntu-linux/#c9783a2951c9) on this. 
2. Install [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).
3. Run the following command in the terminal interface to install other peripheral packages.

  ```bash
  sudo apt install ros-melodic-navigation ros-melodic-octomap libcap0.8-dev git chrony
  ```
4. Download this repository by entering the command below into the terminal.
  ```bash
  git clone https://github.com/lanl/NRG-Mobile.git
  git submodule update --init
  ``` 
5. TODO: Finish section with any other package setup including scripts required

### Operation

TODO: Please be patient as we work to create comprehensive guides.

