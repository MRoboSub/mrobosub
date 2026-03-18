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