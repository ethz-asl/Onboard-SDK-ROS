#DJI Onboard SDK ROS (3.2) Packages
## We modified this package based on DJI Onboard SDK ROS (3.2) in ways that

1. Increasing Baudrate from 230400 to 921600 bps in order to cope with increasted IMU publishing and sending virtual command rates (100Hz). (They were 50Hz).

2. Directly sending a control command (roll, pitch angles, yaw rate and vertical velocity) via serial port to N1 autopilot (original DJI M100 autopilot) in order to avoid using ROS service call for this. Note that ROS service call should not be used for continuous command data streams as suggested by [ROS communication patterns](http://wiki.ros.org/ROS/Patterns/Communication). (There are reasons behind this but most importantly it's a blocking call!).

3. Publishing IMU and odomtery message topics (using ROS topic) using [ROS standard coordinate system](http://www.ros.org/reps/rep-0103.html) and [this](http://www.ros.org/reps/rep-0105.html).

4. Fixing some buffer overflow error.

5. Some extra features such as deadzone recovery and auto-trim compensation. 

Installation instructions
------
(If you already installed ROS on your system ([ROS installation](http://wiki.ros.org/indigo/Installation/Ubuntu), please skip step 1 and 2).
1 Install and initialize ROS indigo desktop full, additional ROS packages, catkin-tools:
  
```sh
  $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
  $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
  $ sudo apt-get update
  $ sudo apt-get install ros-indigo-desktop-full ros-indigo-joy ros-indigo-octomap-ros python-wstool python-catkin-tools
  $ sudo rosdep init
  $ rosdep update
  $ source /opt/ros/indigo/setup.bash
```
2 Initialize catkin workspace:
```sh
  $ mkdir -p ~/catkin_ws/src
  $ cd ~/catkin_ws
  $ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
  $ catkin init  # initialize your catkin workspace
```

## We are preparing the complate documentation, please bear with us :)

https://arxiv.org/abs/1701.08623

When using these software packages in your research, it would be great if you cite us!!

```bibtex
@ARTICLE{2017M100Ctrl,
          author = {{Sa}, I. and {Kamel}, M. and {Khanna}, R. and {Popovic}, M. and {Nieto}, J. {Siegwart}, R.},
          title = "{Dynamic System Identification, and Control for a cost effective open-source VTOL MAV}",
          archivePrefix = "arXiv",
          eprint = {1701.08623},
          primaryClass = "cs.RO",
          keywords = {Computer Science - Robotics},
          year = 2017,
          month = Jan
}
```


## More documentation can be found from http://goo.gl/sgh5C0

####Please refer to <https://developer.dji.com/onboard-sdk/documentation/github-platform-docs/ROS/README.html> in DJI Developer Website.
