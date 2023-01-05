# mrobosub

consolidated repo for the mrobosub ros network

## Package Structure

- `mrobosub` - The metapackage which depends on all other packages. Only need to modify this when creating a new package
- `mrobosub_fcu` - The flight controller unit (FCU) package. Handles mixing together thrusters and sensors to determine how to move the submarine in the desired direction.
- `mrobosub_gnc` - The guidance navigation and control (GNC) package. Handles higher level control of the submarine, such heading control and depth control, via feedback loops.
- `mrobosub_hal` - The hardware abstraction layer (HAL) package. Contains code that directly interfaces with the hardware devices.
- `mrobosub_localization` - The localization package. Estimates the submarine's location relative to its enviornment.
- `mrobosub_msgs` - The messages package. Contains all the custom ROS message types created for the `mrobosub` project. Most packages depend on `mrobosub_msgs`.
- `mrobosub_perception` - The perception package. Handles retrieving images from the cameras and processing them via classical CV (computer vision) and ML (machine learning) to determine where objects are located relative to the submarine.
- `mrobosub_planning` - Contains the high level state machine that determines what the submarine should be doing at all times.

## Docker

```
docker compose up -d
docker compose exec mrobosub bash
```

## Coordinate System

![image.png](./docs/img/coords.png)

