# OBLAM Assignment: Loop Closure and Pose-Graph Optimization
<!-- via Continuous-time Optimization -->

# Course page

The course materials and instructions can be found at [KTH Canvas](https://canvas.kth.se/courses/40649) .

# Prerequisite

The software was developed on the following dependancies
1. [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
2. [ROS Noetic](http://wiki.ros.org/noetic/Installation)
3. [Ceres 2.1.0](http://ceres-solver.org/installation.html) (do checkout the branch 2.1.0 after git clone)
4. PCL libary (built-in of Ubuntu 20.04)

The code was editted on VS Code with [#region folding add-on](https://marketplace.visualstudio.com/items?itemName=maptz.regionfolder) for tidier view.

# Installation
Please install all dependencies first. Afterwards, create a ros workspace, clone the package to the workspace, and build by `catkin build` or `catkin_make`, for e.g.:

```
mkdir catkin_ws/src
cd catkin_ws/src
git clone https://github.com/brytsknguyen/oblam_pgo
cd ..; catkin build
```
# Download Data
Please download pointcloud data [here](https://drive.google.com/drive/folders/1zwZQPBPeBefeB_StdvPEzp0Cp8pmhcP8?usp=share_link)

Declare the path to the data in the launch file run_pgo.launch.

# Assignment
Go to the function OptimmizePoseGraph() and create a ceres problem and solve it. If it works you should see a loop closure event like this.

<p align="center">
    <img src="docs/loop2.gif" alt="mcd ntu daytime 04" width="49%"/>
    <img src="docs/loop1.gif" alt="mcd ntu daytime 01" width="49%"/>
</p>

# Enjoy Studying!
<img src="docs/thinkingfoot.png" alt="drawing" width="300"/>
