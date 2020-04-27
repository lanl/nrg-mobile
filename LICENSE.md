# Nuclear Robotics Group (NRG)- Autonmous Mobile Survey Software

## Terms and Conditions of Use

Pleae read the following terms and conditions carefully before using this software. This project has been reviewed by the Los Alamos National Laboratory (LANL) Export Control and the Classification Teams and deemed to be Open Source Software under a 3-Clause BSD license. The copyright disclosure record is C19007: "Nuclear Robotics Group: Autonomous Mobile Survey System."

## Copyright

© (or copyright) 2020. Triad National Security, LLC. All rights reserved.
This program was produced under U.S. Government contract 89233218CNA000001 for Los Alamos
National Laboratory (LANL), which is operated by Triad National Security, LLC for the U.S.
Department of Energy/National Nuclear Security Administration. All rights in the program are
reserved by Triad National Security, LLC, and the U.S. Department of Energy/National Nuclear
Security Administration. The Government is granted for itself and others acting on its behalf a
nonexclusive, paid-up, irrevocable worldwide license in this material to reproduce, prepare
derivative works, distribute copies to the public, perform publicly and display publicly, and to permitothers to do so.

Futhermore, this open source software can be redistributed and/or modified according to the terms of the 3-Clause BSD License below. Any modified code or derived works should be cleared marked so as to be differentiated from the version available here. The full text of the 3-Clause BSD License can be found in the section below.

## 3-Clause BSD License

This program is open source under the BSD-3 License.
Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
	 
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
and the following disclaimer in the documentation and/or other materials provided with the distribution.
	 
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse
or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Addtional License Information

The NRG-Mobile software project is based in the Robot Operating System (ROS) framework. It makes extensive use of external libraries and packages. Unless otherwise stated, packages are not modified and contribute to this project through linking. Those third party packages with modified source code will include the original licenses and copyrights in their respective source files. Other copyrights and attributed packages can be found below:

1. ROS [libraries](https://www.ros.org): Framework for communication and structure of robotics software. Tools, libraries, and conventions for hardware agnostic operations of robotic systems.

2. ROS [Navigation](http://wiki.ros.org/navigation): Navigation libraries within the ROS framework for controlling robot movement and path planning. Specific recovery behavior files have been modified and include the original license and copyright in the modified source file.

3. Velodyne [drivers](http://wiki.ros.org/velodyne): Drivers for the Velodyne VLP-16 3D LIDAR sensor. This sensor generates 3D range data in a form known as a point cloud.

4. ROS [Point Cloud Library (PCL)](http://wiki.ros.org/pcl): Library for handling operations with 3D data in point clouds. 

5. [PyMC3](https://docs.pymc.io): Statistics library for Monte Carlo Estimation written in Python.

6. [Qt5](https://www.qt.io/): Graphical framework for creating user interfaces. Libraries dynamically linked.

7. ROS [Octomap](http://wiki.ros.org/octomap): Mapping system within the ROS framework for handling 3D maps. No files directly used, but some methods provided inspiration for original code. 