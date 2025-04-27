# SLAM Workshop

<p align="center">
<img src = "doc/imgs/slamworkshop.PNG?raw=true" center=true width="85%"/>
</p>


The slam workshop is at is, a 2 day intensive workshop on slam for building a robot from the scratch (simulation)


<div align="center">
<video width="640" height="480" controls loop>
<source src="doc/vids/slam.mp4" type="video/mp4">
</video>
</div>

<details open>
<summary> <b>Brief Review<b></summary>

- We made and standarized the packages to use rviz2, gazebo, slam_toolbox to achieve a 4-wheeled robot to navigate an environment (cafeteria)
  - First, configured the state publisher
  - Next, loaded the URDF model into gazebo and test the joints
  - Incorporated (in a non-standard way) 2 differential drive controllers into gazebo without using skid steer controller.
  - Constructed a world in gazebo
  - Made a launch file to load the world
  - Spawned the robot into the world with the controllers
  - Later incorporated the slam_toolbox package (or you could also use gmapping)
  - After using, in this case the slam_toolbox we can see the full environment
  - Finally saved the map.
- Because this seminar was only on SLAM, we only focused on creating packages for SLAM only and not autonomous navigation.

</details>

<details close>
<summary> <b>Project Tree<b></summary>

```sh
.
├── doc
│   ├── imgs
│   │   ├── frames.PNG
│   │   ├── graph.PNG
│   │   └── slamworkshop.PNG
│   ├── pdf
│   │   └── frames_2025-04-27_12.50.19.pdf
│   └── vids
│       ├── bringup.mp4
│       ├── gazebo.mp4
│       ├── rviz2.mp4
│       └── slam.mp4
├── README.md
├── slam_gmapping
│   ├── openslam_gmapping
│   │   ├── build_tools
│   │   │   ├── generate_shared_object
│   │   │   ├── Makefile.app
│   │   │   ├── Makefile.generic-shared-object
│   │   │   ├── Makefile.subdirs
│   │   │   ├── message
│   │   │   ├── pretty_compiler
│   │   │   └── testlib
│   │   ├── carmenwrapper
│   │   │   ├── carmenwrapper.cpp
│   │   │   └── carmenwrapper.h
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── gfs-carmen
│   │   │   ├── gfs-carmen.cpp
│   │   │   └── Makefile
│   │   ├── grid
│   │   │   ├── graphmap.cpp
│   │   │   ├── Makefile
│   │   │   └── map_test.cpp
│   │   ├── gridfastslam
│   │   │   ├── CMakeLists.txt
│   │   │   ├── gfs2log.cpp
│   │   │   ├── gfs2neff.cpp
│   │   │   ├── gfs2rec.cpp
│   │   │   ├── gfs2stat.cpp
│   │   │   ├── gfs2stream.cpp
│   │   │   ├── gfsreader.cpp
│   │   │   ├── gfsreader.h
│   │   │   ├── gridslamprocessor.cpp
│   │   │   ├── gridslamprocessor_tree.cpp
│   │   │   └── motionmodel.cpp
│   │   ├── gui
│   │   │   ├── CMakeLists.txt
│   │   │   ├── gfs2img.cpp
│   │   │   ├── gfs_logplayer.cpp
│   │   │   ├── gfs_nogui.cpp
│   │   │   ├── gfs_simplegui.cpp
│   │   │   ├── gsp_thread.cpp
│   │   │   ├── gsp_thread.h
│   │   │   ├── qgraphpainter.cpp
│   │   │   ├── qgraphpainter.h
│   │   │   ├── qmappainter.cpp
│   │   │   ├── qmappainter.h
│   │   │   ├── qnavigatorwidget.cpp
│   │   │   ├── qnavigatorwidget.h
│   │   │   ├── qparticleviewer.cpp
│   │   │   ├── qparticleviewer.h
│   │   │   ├── qpixmapdumper.cpp
│   │   │   ├── qpixmapdumper.h
│   │   │   ├── qslamandnavwidget.cpp
│   │   │   └── qslamandnavwidget.h
│   │   ├── include
│   │   │   └── gmapping
│   │   │       ├── grid
│   │   │       │   ├── accessstate.h
│   │   │       │   ├── array2d.h
│   │   │       │   ├── harray2d.h
│   │   │       │   └── map.h
│   │   │       ├── gridfastslam
│   │   │       │   ├── gridslamprocessor.h
│   │   │       │   ├── gridslamprocessor.hxx
│   │   │       │   └── motionmodel.h
│   │   │       ├── log
│   │   │       │   ├── configuration.h
│   │   │       │   └── sensorlog.h
│   │   │       ├── particlefilter
│   │   │       │   └── particlefilter.h
│   │   │       ├── scanmatcher
│   │   │       │   ├── icp.h
│   │   │       │   ├── scanmatcher.h
│   │   │       │   └── smmap.h
│   │   │       ├── sensor
│   │   │       │   ├── sensor_base
│   │   │       │   │   ├── sensor.h
│   │   │       │   │   └── sensorreading.h
│   │   │       │   ├── sensor_odometry
│   │   │       │   │   ├── odometryreading.h
│   │   │       │   │   └── odometrysensor.h
│   │   │       │   └── sensor_range
│   │   │       │       ├── rangereading.h
│   │   │       │       └── rangesensor.h
│   │   │       └── utils
│   │   │           ├── autoptr.h
│   │   │           ├── commandline.h
│   │   │           ├── gvalues.h
│   │   │           ├── macro_params.h
│   │   │           ├── point.h
│   │   │           └── stat.h
│   │   ├── ini
│   │   │   ├── gfs-LMS-10cm.ini
│   │   │   ├── gfs-LMS-20cm.ini
│   │   │   ├── gfs-LMS-5cm.ini
│   │   │   ├── gfs-PLS-10cm.ini
│   │   │   └── gfs-PLS-5cm.ini
│   │   ├── log
│   │   │   ├── carmenconfiguration.cpp
│   │   │   ├── carmenconfiguration.h
│   │   │   ├── configuration.cpp
│   │   │   ├── configuration.h
│   │   │   ├── log_plot.cpp
│   │   │   ├── log_test.cpp
│   │   │   ├── Makefile
│   │   │   ├── rdk2carmen.cpp
│   │   │   ├── scanstudio2carmen.cpp
│   │   │   ├── sensorlog.cpp
│   │   │   ├── sensorlog.h
│   │   │   ├── sensorstream.cpp
│   │   │   └── sensorstream.h
│   │   ├── package.xml
│   │   ├── particlefilter
│   │   │   ├── particlefilter.cpp
│   │   │   ├── particlefilter_test.cpp
│   │   │   ├── pf.h
│   │   │   └── range_bearing.cpp
│   │   ├── README
│   │   ├── scanmatcher
│   │   │   ├── CMakeLists.txt
│   │   │   ├── eig3.cpp
│   │   │   ├── eig3.h
│   │   │   ├── gridlinetraversal.h
│   │   │   ├── icptest.cpp
│   │   │   ├── lumiles.h
│   │   │   ├── scanmatcher.cpp
│   │   │   ├── scanmatcher.new.cpp
│   │   │   ├── scanmatcherprocessor.cpp
│   │   │   ├── scanmatcherprocessor.h
│   │   │   ├── scanmatch_test.cpp
│   │   │   └── smmap.cpp
│   │   ├── sensor
│   │   │   ├── CMakeLists.txt
│   │   │   ├── Makefile
│   │   │   ├── sensor_base
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── sensor.cpp
│   │   │   │   ├── sensoreading.h
│   │   │   │   └── sensorreading.cpp
│   │   │   ├── sensor_odometry
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── odometryreading.cpp
│   │   │   │   └── odometrysensor.cpp
│   │   │   └── sensor_range
│   │   │       ├── CMakeLists.txt
│   │   │       ├── rangereading.cpp
│   │   │       └── rangesensor.cpp
│   │   └── utils
│   │       ├── autoptr_test.cpp
│   │       ├── CMakeLists.txt
│   │       ├── datasmoother.h
│   │       ├── dmatrix.h
│   │       ├── movement.cpp
│   │       ├── movement.h
│   │       ├── optimizer.h
│   │       ├── orientedboundingbox.h
│   │       ├── orientedboundingbox.hxx
│   │       ├── printmemusage.cpp
│   │       ├── printmemusage.h
│   │       ├── printpgm.h
│   │       ├── stat.cpp
│   │       └── stat_test.cpp
│   ├── README.md
│   └── slam_gmapping
│       ├── CMakeLists.txt
│       ├── include
│       │   └── slam_gmapping
│       │       └── slam_gmapping.h
│       ├── launch
│       │   └── slam_gmapping.launch.py
│       ├── package.xml
│       └── src
│           └── slam_gmapping.cpp
├── slam-workshop-bringup
│   ├── CMakeLists.txt
│   ├── launch
│   │   └── main.launch.py
│   ├── package.xml
│   └── README.md
├── slam-workshop-description
│   ├── CMakeLists.txt
│   ├── config
│   │   └── display.rviz
│   ├── launch
│   │   ├── main.launch.py
│   │   ├── rviz2.launch.py
│   │   └── state_publisher.launch.py
│   ├── LICENSE
│   ├── package.xml
│   ├── README.md
│   └── urdf
│       ├── mobRob_diff_drive.urdf
│       ├── mobRob_skid_steer.urdf
│       └── mobRob.urdf
├── slam-workshop-gazebo
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── gazebo.launch.py
│   │   └── main.launch.py
│   ├── package.xml
│   ├── README.md
│   └── worlds
│       └── cafe-world.world
└── slam-workshop-mapping
    ├── CMakeLists.txt
    ├── config
    │   └── mapper_params_online_async.yaml
    ├── launch
    │   └── slam.launch.py
    ├── maps
    │   ├── cafe-map.pgm
    │   └── cafe-map.yaml
    ├── package.xml
    └── README.md

51 directories, 176 files
```
</details>

<details open>
<summary> <b>Computational Graph of Nodes Launched<b></summary>

<p align="center">
<img src = "doc/imgs/graph.PNG?raw=true" center=true width="85%"/>
</p>

</details>


<details open>
<summary> <b>Frames view<b></summary>

<p align="center">
<img src = "doc/imgs/frames.PNG?raw=true" center=true width="85%"/>
</p>

</details>


<details open>
<summary> <b>Step 1 - Robot URDF and Visualzation<b></summary>

Created an URDF file and test the robot in rviz2

We create two robots, one based on the [Mobile URDF Maker](https://github.com/ali-pahlevani/Mobile_Robot_URDF_Maker.git) by [Ali Pahlevani](https://www.linkedin.com/in/ali-pahlevani/), and other urdf based on a differential drive robot from the scratch using [this repository](https://github.com/gurselturkeri/ros2_diff_drive_robot).

<div align="center">
<video width="640" height="480" controls loop>
<source src="doc/vids/rviz2.mp4" type="video/mp4">
</video>
</div>

</details>


<details open>
<summary> <b>Step 2 - Gazebo (classic) Simulation<b></summary>

Later we also defined a gazebo enviroment and spawned the robot for testing purposes.

<div align="center">
<video width="640" height="480" controls loop>
<source src="doc/vids/gazebo.mp4" type="video/mp4">
</video>
</div>

</details>

<details open>
<summary> <b>Step 3 and 4 - Mapping using slam_toolbox<b></summary>

Finally we managed to do 2 packages, one for mapping (used slam_toolbox for make the robot map the environment) and the other that brings up all the system.  Later when finished to map you can save the environment.


<div align="center">
<video width="640" height="480" controls loop>
<source src="doc/vids/bringup.mp4" type="video/mp4">
</video>
</div>


</details>



<details open>
<summary> <b>Using The Package <b></summary>

- Some before proceed:
  - we used the models: 
    - groud_plane
    - sun
    - cafe
    - cafe_table
  - If you don not have this models, you could [download from here](https://github.com/osrf/gazebo_models) include on the folders:
    - **~/.gazebo/models** or in
    - **/usr/share/gazebo-X/models** where X is the gazebo version
  - Watch that in .pgm files origin must be float, not integers. Otherwise you could have an error like: 
  - [ERROR] [ros2-4]: process has died [pid 18959, exit code 1, cmd 'ros2 lifecycle set /map_server activate'].

- Also in the config file for the slam_toolbox configuration, if you use an initial pose of the robot, make sure to be floats too.
```sh
      map_start_pose: [0.0, 0.0, 0.0]
```

- Next, follow the next steps to replicate the outcome...

- Install eigen, ros2 controllers and other libraries using your ros distribution if you dont have it installed (including map_saver)
```sh
    sudo apt-get update
    sudo apt-get install ros-<ros-distro>-slam-toolbox
    sudo apt install ros-<ros-distro>-gazebo-ros-pkgs
    sudo apt install ros-<ros-distro>-ros2-control 
    sudo apt install ros-<ros-distro>-ros2-controllers 
    sudo apt install ros-<ros-distro>-diff-drive-controller
    sudo apt install ros-<ros-distro>-joint-state-broadcaster
    sudo apt install ros-<ros-distro>-effort-controllers
    sudo apt install ros-<ros-distro>-velocity-controllers
    sudo apt install ros-<ros-distro>-position-controllers 
    sudo apt install ros-<ros-distro>-teleop-twist-keyboard 
    sudo apt install ros-<ros-distro>-rviz2 
    sudo apt install ros-<ros-distro>-robot-state-publisher    
```

- Create the workspace
```sh
    cd ~
    mkdir -p ros2_slam_worshop/src
    cd ros2_slam_worshop/src
```
- Fork (or clone) this repo in the `~/ros2_slam_worshop/src` folder by typing:
```sh 
    git clone https://github.com/issaiass/SLAM-Workshop.git --recursive

```

- Later compile the SLAM Workshop repository and source it
```sh
    cd ~/ros2_slam_workshop
    colcon build --packages-select openslam_gmapping slam_gmapping
    colcon build
    source install/setup.bash
```

- Run the main demo for SLAM Workshop with slam_toolbox:
- On terminal 1 (for launch everything and slam_toolbox)
```sh
    ros2 launch slam-workshop-bringup main.launch.py
```
- On terminal 2 (for navigating)
```sh
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- Use the keyboard to navigate (see the console output for keys)
- When you are done, press Ctrl+C or in a third terminal type
```sh
    mkdir -p ~/ros2_slam_workshop/src/slam-workshop-mapping/maps
    ros2 run nav2_map_server map_saver_cli -f <map_name_without_extension>
```


- On terminal 2
```sh
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- Navigate with the keys to construct the map (see rviz how is going)

- When you are satisfied with the contruscted map input the following commands on a new terminal to save the map.
```sh
    ros2 run nav2_map_server map_saver_cli -f 
    module_7_assignment/mapping/map/corridor
```

- On terminal 3, optionally load and visualize the map
```sh
    ros2 launch mapping map_loading_2d.launch.py map:=corridor.yaml
```

</details>

<details open>
<summary> <b>Results <b></summary>


<p align="center">
  <a href="https://youtu.be/6sfYys5ov4k">
    <img src="https://img.youtube.com/vi/6sfYys5ov4k/0.jpg" alt="Yet Another SLAM (only) video">
  </a>
</p>

</details>


<details open>
<summary> <b>Issues<b></summary>

- Mobile_Robot_URDF_Maker URDF for SLAM is not currently is not working for me and i did not found the solution, but the error is in the URDF file somewhere:
```sh
[async_slam_toolbox_node-1] [WARN] [1702066299.830374687] [slam_toolbox]: Failed to compute odom pose. 
```

</details>

<details open>
<summary> <b>Future Work<b></summary>

- Add navigation if required

</details>

<details open>
<summary> <b>Contributing<b></summary>

Your contributions are always welcome! Please feel free to fork and modify the content but remember to finally do a pull request.

</details>

<details open>
<summary> :iphone: <b>Having Problems?<b></summary>

<p align = "center">

[<img src="https://img.shields.io/badge/linkedin-%230077B5.svg?&style=for-the-badge&logo=linkedin&logoColor=white" />](https://www.linkedin.com/in/riawa)
[<img src="https://img.shields.io/badge/telegram-2CA5E0?style=for-the-badge&logo=telegram&logoColor=white"/>](https://t.me/issaiass)
[<img src="https://img.shields.io/badge/instagram-%23E4405F.svg?&style=for-the-badge&logo=instagram&logoColor=white">](https://www.instagram.com/daqsyspty/)
[<img src="https://img.shields.io/badge/twitter-%231DA1F2.svg?&style=for-the-badge&logo=twitter&logoColor=white" />](https://twitter.com/daqsyspty) 
[<img src ="https://img.shields.io/badge/facebook-%233b5998.svg?&style=for-the-badge&logo=facebook&logoColor=white%22">](https://www.facebook.com/daqsyspty)
[<img src="https://img.shields.io/badge/linkedin-%230077B5.svg?&style=for-the-badge&logo=linkedin&logoColor=white" />](https://www.linkedin.com/in/riawe)
[<img src="https://img.shields.io/badge/tiktok-%23000000.svg?&style=for-the-badge&logo=tiktok&logoColor=white" />](https://www.linkedin.com/in/riawe)
[<img src="https://img.shields.io/badge/whatsapp-%23075e54.svg?&style=for-the-badge&logo=whatsapp&logoColor=white" />](https://wa.me/50766168542?text=Hello%20Rangel)
[<img src="https://img.shields.io/badge/hotmail-%23ffbb00.svg?&style=for-the-badge&logo=hotmail&logoColor=white" />](mailto:issaiass@hotmail.com)
[<img src="https://img.shields.io/badge/gmail-%23D14836.svg?&style=for-the-badge&logo=gmail&logoColor=white" />](mailto:riawalles@gmail.com)

</p

</details>

<details open>
<summary> <b>License<b></summary>
<p align = "center">
<img src= "https://mirrors.creativecommons.org/presskit/buttons/88x31/svg/by-sa.svg" />
</p>
</details>