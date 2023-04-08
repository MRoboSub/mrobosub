# mrobosub

Consolidated repo for the mrobosub ros network

## Install
```console
$ rosdep install --from-paths src --ignore-src -r -y
```
Be in `~/catkin_ws` and installs all dependencies for ros

## Docker

After installing Docker Desktop, run these commands in this folder to start the container

```console
$ docker compose up -d
$ docker compose exec mrobosub bash
```

## Package Structure

- `mrobosub` - The metapackage which depends on all other packages. Only need to modify this when creating a new package
- `mrobosub_fcu` - The flight controller unit (FCU) package. Handles mixing together thrusters and sensors to determine how to move the submarine in the desired direction.
- `mrobosub_gnc` - The guidance navigation and control (GNC) package. Handles higher level control of the submarine, such heading control and depth control, via feedback loops.
- `mrobosub_hal` - The hardware abstraction layer (HAL) package. Contains code that directly interfaces with the hardware devices.
- `mrobosub_localization` - The localization package. Estimates the submarine's location relative to its enviornment.
- `mrobosub_msgs` - The messages package. Contains all the custom ROS message types created for the `mrobosub` project. Most packages depend on `mrobosub_msgs`.
- `mrobosub_perception` - The perception package. Handles retrieving images from the cameras and processing them via classical CV (computer vision) and ML (machine learning) to determine where objects are located relative to the submarine.
- `mrobosub_planning` - Contains the high level state machine that determines what the submarine should be doing at all times.

## Coordinate System

![image.png](./docs/img/coords.png)


## Computer Setup

### Bringup

Use the `robot_upstart` package to run a launch file on boot

```bash
$ sudo apt install ros-noetic-robot-upstart
$ rosrun robot_upstart install <pkg>/launch/<file>.launch --job <service_name> --symlink
$ sudo systemctl daemon-reload
$ sudo systemctl start <service_name>
``` 

### GPIO

Bringup as currently written relies on GPIO. In order to run GPIO code, regular users need read/write access to `/dev/gpiomem`

```bash
$ chmod o+rw /dev/gpiomem
```


## Messages

### Wildcards

The string `*6dof` in a topic name indicates that there exists 6 topics, one for each degree of freedom (DOF). The DOF replaces `*6dof` in each topic name. The DOF's are:
- `surge`
- `sway`
- `heave`
- `roll`
- `pitch`
- `yaw`

The string `*object`  in a topic name indicates that there exists a topic for each type of object that we are interested in. The object names are currently:
- `gate`

### Topics

- `/raw_imu (sensor_msgs/Imu)`
    - raw IMU data (oritentation, etc)
- `/raw_depth (std_msgs/Float64)`
    - raw depth (m) (TODO is this positive or negative when descending?)
- `/target_pose/*6dof (std_msgs/Float64)`
    - desired position
- `/target_twist/*6dof (std_msgs/Float64)`
    - desired speeds
- `/output_wrench/*6dof (std_msgs/Float64)`
    - force on each DOF to send to the FCU
- `/pose/*6dof`
    - current pose
- `/obj_psn/*object (mrobosub_msgs/ObjectPosition)`
    - information about location of object on the screen and distance from the UAV

### Services
- `/obj_enable/*object (std_srvs/SetBool)`
    - determines if an object's information should be published or not. If set to `false`, nothing should be published to `/obj_psn/*object`
