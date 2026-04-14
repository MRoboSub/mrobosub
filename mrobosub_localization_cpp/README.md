# mrobosub_localization_cpp

For [Zayn Baig](https://github.com/imzaynb)'s Winter 2026 Multidisciplinary Design Project, he was tasked with migrating the [Field Robotics Group](https://fieldrobotics.engin.umich.edu/)'s seminal Autonomous Underwater Veheicle (AUV) localization paper [TURTLMap](https://umfieldrobotics.github.io/TURTLMap/) on the Sub.

This document will serve as a development diary until I complete the entire project, in which case this will turn into more of a mix of rationale and documentation for the localization code.

One thing I strongly believe is important to the longevity of the codebase is thorough, accurate documentation. I hope my efforts here will be a good first step towards accomplishing this.

## TODOs
[] I should probably make a Github Project to keep me organized

[] Figure out how to add downloading GTSAM to the Docker image

[] Figure out how to add all of the other C++ requirements to the docker image
  - could this be added to the CMakeLists.txt file? Not sure how ament works

[] I have a feeling the state code listed in `PosegraphNode.h` could definitely be split into state classes for `KeyframeState`, `DvlState`, `ImuState`, and `BarometerState`.

## Development Diary

### [03/17] Cloning the repository
I realized pretty late that our codebase (as of October 2025) is fully in ROS2 while TURTLMap was written completely in ROS1. As such, this is going to be both a mix of making their code work on our submarine along with migrating their code to ROS2. 

I created this package using 
```bash
ros2 pkg create --license Apache-2.0 --build-type ament-cmake mrobosub_localization_cpp
```

I then updated the `package.xml` to reflect relavent changes that needed to be made to the file.

Across the codebase, we use the BSD-2.0 license, but since I am heavily using the [TURTLMap Github repository](https://github.com/umfieldrobotics/TURTLMap) which has this Apache 2.0 license, I thought it best to adhere to the one they use.

### [03/17-03/18] Porting all of the Header files
I thought a good place to start would be porting over all of the headerfiles under `./include/mrobosub_localization_cpp` into this repository. 

I chose to have member variables for most classes have an `_` prefix instead of an `_` suffix. I the place where I chose not to adhere to this rule is for the structs in which all members are public anyways and there is really no more elaborate code than using the struct to hold related data together.

I also found it fit to change the variable names and organize the code a little bit. I have a feeling that I will be majorly refactoring some of the code, especially the ROS specific code, later on, but I think gradual change is the best. I think my idea for the time being will be to keep it relatively similar to the source material for now until I get it running in which case I can majorly refactor it later.

I also had the header files only define all of the functions, not actually implement them. They had a healthy mix of some getters/setters being implemented in the header file, but I prefer consistency, so all function implementations will be in the source files.

### [03/18] Loading the GTSAM library onto this Docker container
The easiest way that I have seen to load GTSAM in for a ROS environment is installing the package via apt.

```bash
sudo apt install ros-humble-gtsam
```

I was considering adding the GTSAM Github as a submodule, but this would drastically increase compilation times which is really not worth it given how frequently we build.

This line was added to the Dockerfile.

Additionally, you also need to reflect this dependency in the `package.xml` with the line `<depend>gtsam</depend>`.

Also, you need to reflect these changes in the CMakeLists.txt.

#### 1. Find the package

We first need to have CMake find the package
```
find_package(GTSAM required)
```

#### 2. Link with the library
```
target_link_libraries(localization
    gtsam
)
```

#### 3. (Optional) Expose GTSAM to other modules

If you need other packages in the workspace need to use this package (`mrobosub_localization_cpp`), they will need GTSAM too. We can expose this to them using ament.
```
ament_export_dependencies(gtsam)
```

After adding GTSAM, I rebuilt the container, which successfully worked.

Additionally, I had to add a couple of VSCode extensions for C++ and CMake add better linting and intellisense. With this, I caught some small typos with my code.

### [03/18] Fixing the variables to use the GTSAM library.

Before, I was not using the `gtsam::` variables because I had not included `gtsam` into the project until after I finished all of the header files. I wanted to get a good bit of coding done, just as a productivity boost, before I get bogged down with downloading and debugging install GTSAM.

After downloading the library, as I document above, I fully converted all of the commented out gtsam objects into the actual (uncommented out) versions.

In the existing GTSAM code, there is a mix of smart pointers (e.g. `std::shared_ptr` or `std::unique_ptr`) between both the standard library `std` and the `boost` library. However, GTSAM heavily relies on the `boost` library. As such, for consistency sake, I will consistently use `boost`.

In order to do so, I have to add `Boost` to the CMakeLists.txt in 2 ways:

#### 1. Add the package
```
find_package(Boost REQUIRED COMPONENTS thread)
```

#### 2. Link the libraries
```
target_link_libraries(localization PRIVATE
${BOOST_LIBRARIES}
...
)
```

### [03/18] Migrating the Source Files
Before I get to ironing out the specific differences between ROS1 and ROS2 between the two C++ codebases, I want to migrate over all of the ROS-agnostic source code. 

So far, I have migrated `BluerovBarometerFactor`.

### [03/24] Continuing to Migrate Source Files
I have worked through migrating `DvlOnlyFactor`, `PreintegratedVelocityHelpers`, and started `Posegraph`.

As I began to work through `Posegraph`, one big thing I encountered was the loading of parameters. The existing solution used by the TURTLMap authors is to use YAML files loaded in by an external YAML C++ library. ROS2 has pretty robust parameter loading (not sure about ROS1 which may have motivated the authors to use the external YAML package). As such, I worked to convert the existing method of loading parameters to the ROS2 method.

#### About Parameters
While refactoring the parameter code, I returned back to the ROS2 Parameter documentation. Here are the relevant pages.
- [Parameter Documentation](https://docs.ros.org/en/kilted/p/rclcpp/generated/classrclcpp_1_1Parameter.html)
- [Using Parameters in a C++ Class](https://docs.ros.org/en/kilted/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
- [Understanding ROS2 Parameters](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [Using ROS2 Launch For Large Projects](https://docs.ros.org/en/kilted/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#loadingparametersfromyamlfile)

Here is the general format of a ROS2 parameter file:
```yaml
<node name here>:
  ros__parameters:
    param_one:
    parent_one:
      child_one:
    # ...
```

What I didn't understand until just recently is that in order to access _nested_ parameters, you can use `.` syntax. E.g. to access `child_one`, you would access it via `parent_one.child_one`.

In order to access a node's parameters, you have to have a pointer to the node in question. For me, I wanted to separate the initial parameter loading logic into its own class (see `Parameter.h` and `Parameter.cpp`). Thus, in order to accomplish this, you need to pass a pointer to the node into the `Parameter` class. 

If you want to access a shared_ptr to `this` you can use `this->shared_from_this`. However, you can't pass a `shared_ptr` of a class into any other class or for any other use-case (as it does not exist!!!) until **AFTER THE CONSTRUCTOR RUNS**. I did not know this! As such, you must have a separate `initialize_parameters` method that you will run to populate the parameters after 

### [4/13] Determining Parameter Values
To determine the various parameter values, I started off with the datasheets for each of the sensors. 

#### Inertial Sense IMX-5
The Inertial Sense IMX-5 (IMU)'s datasheet is [here](https://docs.inertialsense.com/datasheets/IMX-5_IMU_AHRS_GNSS-INS_Datasheet.pdf). From the datasheet, I was able to get the arr/gyro maximum values, noise density, and random walk values. 

According to the datasheet, apparently I am able to get readings at a rate of 1kHz which is really good (4x the rate that the TURTLMap IMU was able to get).

Since I am having this node run in it's own package, I might be able to bump up the ROS publishing rate. Right now I am having the node sleep for 5ms before continuing --> this maxes out my maximum rate to 200kHz.

I also changed the message type from our custom type to the `sensor_msgs::msg::Imu` type to be more in line with the traditional ROS2 sensor types.

### Tracker 650 DVL
I primarily just need the extrinsic transformations from the CAD. Will ask Mechanical for those numbers ASAP.

I needed to change the data published by the DVL because according to the datasheet [here](https://docs.ceruleansonar.com/c/tracker-650/communicating-with-the-tracker-650/outgoing-message-formats-tracker-650-to-host/usddvkfc-kalman-filter-raw-data-support-message) the DVL actually publishes it's confidence in each of it's axes. This can be used to provide a covariance value.

Apparently the typical ROS method for publishing messages parsed from a DVL is the `TwistWithCovarianceStamped` (which makes sense since the DVL provides the sub an estimate of it's twist).

Right now, we were extracting the raw beam velocities in the DVL publisher node as opposed to publishing the vx, vy, vz. I had to use a transformation matrix to turn beam velocities into robot frame velocities.

The sensor is mounted via a NED format. Using right hand rule, point your index finger away from the data cable, your middle finger towards the right, and your thumb pointed down. 

See the following reference image.
![image](https://docs.ceruleansonar.com/c/~gitbook/image?url=https%3A%2F%2F977450193-files.gitbook.io%2F%7E%2Ffiles%2Fv0%2Fb%2Fgitbook-x-prod.appspot.com%2Fo%2Fspaces%252FldEroQctKFErSiZvuhJk%252Fuploads%252FxuL7u4W2BE68qUs2ovN7%252Fimage.png%3Falt%3Dmedia%26token%3D669440b3-49cd-4d2f-a5d5-f8bcedde80e0&width=768&dpr=3&quality=100&sign=d4de0e74&sv=2)

Note that because of the beam orientations, the A beam is aligned with the x direction. 

## Citation

### Acknowledgements
I give a huge thanks to [Dr. Katie Skinner](https://robotics.umich.edu/people/faculty/katie-skinner/) for guiding me with this project. She's such an awesome person!

### TURTLMap Paper Citation

```bibtex
@inproceedings{song2024turtlmap,
  title={TURTLMap: Real-time Localization and Dense Mapping of Low-texture Underwater Environments with a Low-cost Unmanned Underwater Vehicle},
  author={Song, Jingyu and Bagoren, Onur and Andigani, Razan and Sethuraman, Advaith and Skinner, Katherine A},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={1191--1198},
  year={2024},
  organization={IEEE}
}
```

### TURTLMap Github
You can see the source code provided by the authors for TURTLMap at the [TURTLMap Github repository](https://github.com/umfieldrobotics/TURTLMap).