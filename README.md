DJI Onboard SDK ROS (3.2) Packages
------
We modified this package based on DJI Onboard SDK ROS (3.2) in ways that:

1. Increasing Baudrate from 230400 to 921600 bps in order to cope with increasted IMU publishing and sending virtual command rates (100Hz). (They were 50Hz).

2. Directly sending a control command (roll, pitch angles, yaw rate and vertical velocity) via serial port to N1 autopilot (original DJI M100 autopilot) in order to avoid using ROS service call for this. Note that ROS service call should not be used for continuous command data streams as suggested by [ROS communication patterns](http://wiki.ros.org/ROS/Patterns/Communication). (There are reasons behind this but most importantly it's a blocking call!).

3. Publishing IMU and odomtery message topics (using ROS topic) using [ROS standard coordinate system](http://www.ros.org/reps/rep-0103.html) and [this](http://www.ros.org/reps/rep-0105.html).

4. Fixing some buffer overflow error.

5. Some extra features such as deadzone recovery and auto-trim compensation. 

Overview
------
This repository contains the modified version of DJI Onboard SDK ROS (3.2) that interfaces with ETH ASL software packages such as [MPC controller for MAV](https://github.com/ethz-asl/mav_control_rw/tree/devel/dji_m100_linear) and [Multi-Sensor Fusion framework](https://github.com/ethz-asl/ethzasl_msf).

What you can achieve with these packages is your DJI M100 platform can follow your position commands or ~~trajectory~~. State estimation of MAV can be provided from any frameworks that you have, but here we used Motion capture device (Vicon) measurement for simplicity.

![alt text](https://drive.google.com/file/d/0B-0CTsFowMRVODNDWDNRQnNWR3M/view?usp=sharing)


More technical details can be found from following [relevant publication section](https://github.com/ethz-asl/dji_onboard_sdk_ros/blob/3.2/README.md#relevant-publications-documentataion-and-citation).

Installation instructions
------
(If you already installed ROS on your system ([ROS installation](http://wiki.ros.org/indigo/Installation/Ubuntu), please skip step 1 and 2).
We need two computers; an ordinary laptop (desktop) and onboard computer.

1 Install and initialize ROS indigo desktop full, additional ROS packages, catkin-tools:
```sh
  $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
  $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
  $ sudo apt-get update
  $ sudo apt-get install ros-indigo-desktop-full ros-indigo-joy python-wstool python-catkin-tools
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
3.1 Laptop installation instructions
```sh
  $ git clone https://github.com/ethz-asl/ros_vrpn_client
  $ git clone https://github.com/lrse/ros-keyboard
```

3.2 Onboard computer installation instructions
```sh
  $ sudo apt-get install liblapacke-dev
  $ git clone https://github.com/ethz-asl/dji_onboard_sdk_ros
  $ git clone https://github.com/ethz-asl/mav_control_rw -b devel/dji_m100_linear
  $ git clone https://github.com/ethz-asl/catkin_simple.git
  $ git clone https://github.com/ethz-asl/rotors_simulator -b feature/dji_m100_joy
  $ git clone https://github.com/ethz-asl/mav_comm.git
  $ git clone https://github.com/ethz-asl/eigen_catkin.git
  $ git clone https://github.com/ethz-asl/ethzasl_msf.git
```

* Build the workspace
Run following command at your catkin workspace root, '~/catkin_ws'
```sh
  $ catkin build
```

Relevant publications, documentataion, and citation
------

https://arxiv.org/abs/1701.08623

When using these software packages in your research or help you somehow, we would be very happy and looking forward to hearing from your experiences. It would be nice for us if you could cite us as well!!

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
