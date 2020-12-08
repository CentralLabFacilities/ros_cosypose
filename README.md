# ros_cosypose

ROS interface to the cosypose library.

# Installation

- install [cosypose](https://github.com/ylabbe/cosypose). It creates an anaconda environment during the installation.
- install additional dependencies inside the anaconda environment:

```
anaconda activate cosypose
pip install transforms3d, trimesh
```

- clone the ros_cosypose package into your catkin workspace, rebuild the workspace.

```
conda deactivate
cd CATKIN_WORKSPACE/src
git clone https://github.com/ikalevatykh/ros_cosypose
cd ..
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
catkin build
```

Tested on ROS Noetic, Ubuntu 20.04 and ROS Melodic, Ubuntu 18.04.
Package has minimum dependencies and should work in previous ROS versions also.

# Nodes

## object_recognition.py

Node starts `/recognize_objects` action server.
Publish:

- `/recognize_object_array`: recognition results
- `/recognized_object_marker_array`: debug MarkerArray to visualize recognized objects in rviz
- `/recognized_object_overlay/image`: debug image with detection bounding boxes
  Parameters:
- `image`: camera image topic.
- `one_instance_per_class`: default True
- `detection_threshold`: default 0.5
- `object_coarse_run_id`: default 'coarse-bop-ycbv-synt+real--822463'
- `object_refiner_run_id`: default 'refiner-bop-ycbv-synt+real--631598'
- `object_detector_run_id`: default 'detector-bop-ycbv-synt+real--292971'

## object_information.py

Provides the `/get_object_information` service to retrieve an object information from datasets.

## scene_update.py

Subscribes to the `/recognize_object_array` topic and updating MoveIt planning scene according to the recognition results.

# Launch

Start nodes from the cosypose anaconda environment:

```
anaconda activate cosypose
roslaunch ros_cosypose object_recognition.launch image:=/usb_cam/image_raw config:=ycbv scene_update:=True
```

Arguments:

- `image`: camera image topic.
- `config`: name of a yaml configuration file from the package /config folder
- `config_file`: path to an abitrary config file (overrides `config`)
- `scene_update`: start `scene_update` node or not.

You have to provide tf transformation for the camera frame.

# Test from rviz

Add the MotionPlanning plugin to rviz. Use the Detect button in the Manipulation tab to trigger object recognition.

If you started `scene_update` node, recognized objects will be added to the MoveIt planning scene.

Add the MarkerArray plugin to rviz and subscribe to the `/recognized_object_marker_array` topic to see recognition results independently of MoveIt.

Add the Image plugin to rviz and subscribe to the `/recognized_object_overlay/image` topic to see debug overlay.

# Test from console

```
rosrun ros_cosypose test_object_recognition
rosrun ros_cosypose test_object_information
```

# Debug

If you do not know the camera transformation run dummy publisher:

```
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 usb_cam world 100
```

You can change logger level using `set_logger_level` service:

```
rosservice call --wait /object_recognition/set_logger_level 'rosout' 'debug'
rosservice call --wait /object_information/set_logger_level 'rosout' 'debug'
rosservice call --wait /scene_update/set_logger_level 'rosout' 'debug'
```
