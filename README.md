# Drone Navigation Capstone

My CS BSc drone-navigation capstone project, developed with Itzchak Harel and Roy Naor.

## Project Summary

This project is a ROS-based drone navigation and mapping system for a DJI Tello-style workflow. The source combines drone control, AprilTag tracking, ORB-SLAM integration, trajectory loading, point-cloud conversion, and a Tkinter-based operator UI.

## Tech Stack

- Python 2.7
- ROS, `rospy`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf`
- Tello/Flock ROS messages and driver components
- OpenCV, cv_bridge, Pillow, Tkinter, pygame
- Open3D and point-cloud conversion helpers
- AprilTag and ORB-SLAM integration

## Repository Layout

- `src/` contains the Python source and small trajectory CSV/text files.
- `src/project_gui/` contains the GUI assets required by the interface.
- `docs/ROS_Diagram.png` and `docs/ROS_Diagram.drawio` describe the ROS component flow.
- `docs/preview_image.jpeg` is a small drone preview image.

- `docs/project-report.pdf` contains the project report.
- `docs/final-presentation.pptx` contains the final presentation.
- `docs/demos/` contains flight and tag-tracking demonstrations; `docs/screenshots/` shows the operator interface.
- `tests/` and `docker/` provide the ROS simulation tests.

## Run the simulation tests

Install Docker, then run:

```sh
make test-simulation
```

The image provides ROS Melodic, Python 2.7, OpenCV, Tkinter, and the upstream Flock messages. It runs the project's own Tello simulator and controller on a real ROS graph, with networking disabled outside the container. The tests exercise takeoff, movement, closed-loop position control, trajectory loading, and the operator widgets on a virtual display. They do not send commands to a physical drone.

Physical flight, the live camera/AprilTag pipeline, and ORB-SLAM mapping require their corresponding hardware and integrations; these are not covered by the simulation suite.

For a quick source/document check, run `make check` (requires `uv`).
