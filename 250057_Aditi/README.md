# Multi-Robot SLAM, Autonomous Exploration & Map Merging

## How I Built the Solution

The system was built following a modular architecture, splitting the monolithic ROS 2 package into six separate packages:

1. `simulation`: Contains the Gazebo world and launch files for spawning both TurtleBot3 robots with their respective namespaces (`robot1`, `robot2`).

2. `robot1_slam` & `robot2_slam`: Independent SLAM Toolbox instances configured for each robot's namespace and TF frame. Both also contain isolated Nav2 navigation stacks for local motion planning.

3. `exploration`: A custom frontier-based autonomous exploration node using OpenCV.

4. `map_merger`: A node responsible for receiving both local occupancy grids, performing feature matching using ORB, aligning the maps, and stitching them into a unified global map.

5. `bringup`: A master launch package that orchestrates the entire pipeline through `master.launch.py`.

## SLAM & Exploration Approach

Each robot runs an instance of `async_slam_toolbox_node` within its isolated namespace (`/robot1` and `/robot2`), ensuring independent map frames (`robot1/map` and `robot2/map`).

For exploration, a custom Python node (`frontier_explorer.py`) subscribes to the local occupancy grid map. It processes the grid into an image format and uses morphological operations in OpenCV to identify "frontiers" — boundaries separating free space from unknown space.

It filters out small or unsuitable boundaries using contour-area filtering and selects a viable frontier for exploration. A `nav2_msgs/action/NavigateToPose` goal is then dispatched to the robot's local Nav2 action server to travel toward the selected frontier and expand the map.

This process repeats as the robots discover new unexplored regions.

## Map Merging Approach

The Map Merger node subscribes to the `/robot1/map` and `/robot2/map` topics.

Because the maps are dynamically built and may have different dimensions, the node first converts the ROS 2 `OccupancyGrid` values (`-1`, `0`, `100`) into an image-compatible representation and prepares the maps for alignment.

OpenCV's ORB (Oriented FAST and Rotated BRIEF) feature detector is used to extract features from the environmental structure. Features from both local maps are matched using a Brute Force matcher with Hamming distance.

A homography matrix is then estimated using RANSAC to determine the relative transformation between the two map grids. The second map is transformed into the coordinate space of the first map.

The aligned maps are then merged while preserving known occupancy information and prioritizing occupied cells where appropriate. The resulting unified occupancy grid is periodically published to the `/global_map` topic.

## How to Run It

### Running the Simulation

To launch the complete multi-robot simulation, navigate to the workspace and source the required ROS 2 environments:

```bash
cd ~/ros-bootcamp-task-2/250057_Aditi

source /opt/ros/humble/setup.bash
source install/setup.bash

export TURTLEBOT3_MODEL=waffle

ros2 launch bringup master.launch.py \
robot1_x:=-2.0 robot1_y:=-0.5 \
robot2_x:=2.0 robot2_y:=0.5