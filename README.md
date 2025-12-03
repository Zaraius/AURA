# AURA

# Development Notes

Make sure to build the program at `./ros2_ws/`by running:
- `colcon build --symlink-install`
- `source install/setup.bash`
So it automatically builds when there are file updates. However, you need to redo all these when creating/deleting files.

To run the setup test launch script: `ros2 launch aura test_launch.py`

UWB modules have to be plugged into the RPi in order everytime! 

# Setup Procedure

Make sure all devices on `OLIN-ROBOTICS` WIFI

For docker setup on ros2, make sure to run `docker run -it     --env="DISPLAY=$DISPLAY"     --env="QT_X11_NO_MITSHM=1"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"     -v $(pwd):/root/workspace     --name ros2-mavros    --network host --device /dev/input/js0:/dev/input/js0 --privileged  ros2-mavros`

Make sure:
- `--network host` ensuring docker inheriting computer's IP subnet connection to ROS
- `--device /dev/input/js0:/dev/input/js0` **plug in the joystick before running this commad** to bring the `/dev/input/js0` to the docker

To run another docker instance at the same time: `docker exec -it ros2-mavros bash`

Once entered docker image `ros2-mavros`, run `export ROS_DOMAIN_ID=31` to ensure all programs are sharing the same `ROS_DOMAIN_ID`

Then run `ros2 run joy joy_node` to check joystick setup. If setup properly, it should print some log messages instead of printing nothing

Then the program should be setup!
