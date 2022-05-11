Example Applications for FogROS2
## Running Examples:

#### To run gqcnn
```
ros2 launch fogros2_examples gqcnn_docker.launch.py
```
and run gqcnn's client:
```
docker run --net=host --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --env CYCLONEDDS_URI=file:///tmp/cyclonedds.xml -v $(pwd)/install/share/fogros2/configs/cyclonedds.xml:/tmp/cyclonedds.xml --rm -it keplerc/gqcnn_ros:pj ros2 launch gqcnn_ros client.launch.py
```
in ros workspace.

#### To run vslam

See tutorial walkthrough [here](https://github.com/SimeonOA/orb_slam_2_ros/blob/fogros2/TUTORIAL.md)

Run VSLAM:
```
ros2 launch fogros2_examples vslam.launch.py
```
and run vslam's client:
```
docker run --net=host --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --env CYCLONEDDS_URI=file:///tmp/cyclonedds.xml -v $(pwd)/install/share/fogros2/configs/cyclonedds.xml:/tmp/cyclonedds.xml --rm -it -v /home/gdpmobile7/rgbd_dataset_freiburg1_xyz:/dataset -v $(pwd)/output:/output mjd3/orbslam-ros ros2 launch orb_slam2_ros orb_slam2_d435_rgbd_client_launch.py dataset:=/dataset compress:=0
```
in ros workspace.
