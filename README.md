# Drone Navigation Capstone

My CS BSc drone-navigation capstone project, developed with Itzchak Harel and Roy Naor.

## Project Summary

This project is a ROS-based drone navigation and mapping system for a DJI Tello-style workflow. The recovered source combines drone control, AprilTag tracking, ORB-SLAM integration, trajectory loading, point-cloud conversion, and a Tkinter-based operator UI.

## Tech Stack

- Python 2.7
- ROS, `rospy`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf`
- Tello/Flock ROS messages and driver components
- OpenCV, cv_bridge, Pillow, Tkinter, pygame
- Open3D and point-cloud conversion helpers
- AprilTag and ORB-SLAM integration

## Repository Layout

- `src/` contains the recovered Python source and small trajectory CSV/text files.
- `src/project_gui/` contains the GUI assets required by the interface.
- `docs/ROS_Diagram.png` and `docs/ROS_Diagram.drawio` describe the ROS component flow.
- `docs/preview_image.jpeg` is a small recovered drone preview image.
- `docs/OMITTED_ARTIFACTS.md` lists recovered artifacts intentionally left out of this repository.

## Validate

Run:

```sh
make check
```

The check is static. It verifies the expected source/docs are present and scans tracked text for privacy and machine-path markers. The original ROS stack, drone hardware, and Python 2 runtime were not available during validation.
