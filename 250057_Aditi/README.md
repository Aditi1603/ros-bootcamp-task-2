# Multi-Robot SLAM, Autonomous Exploration & Map Merging

## How I built the solution
The system was built following a modular architecture, splitting the monolithic ROS 2 package into six separate packages:
1. `simulation`: Contains the Gazebo world and launch files for spawning both TurtleBot3 robots with their respective namespaces (`robot1`, `robot2`).
2. `robot1_slam` & `robot2_slam`: Independent SLAM toolbox instances configured for each robot's namespace and TF frame. Both also contain isolated Nav2 navigation stacks for local motion planning.
3. `exploration`: A custom frontier-based autonomous exploration node using OpenCV.
4. `map_merger`: A node responsible for receiving both local grids, performing feature matching using ORB, and stitching them into a unified global map.
5. `bringup`: A master launch package to orchestrate the entire pipeline via `master.launch.py`.

## SLAM & Exploration Approach
Each robot runs an instance of `async_slam_toolbox_node` within its isolated namespace (`/robot1` and `/robot2`), ensuring independent map frames (`robot1/map` and `robot2/map`).
For exploration, a custom Python node (`frontier_explorer.py`) subscribes to the local occupancy grid map. It processes the grid into an image format and uses morphological operations in OpenCV to identify "frontiers"—boundaries separating free space and unknown space. It filters out small/unreachable boundaries using contour areas, and then selects the most viable frontier. A `nav2_msgs/action/NavigateToPose` goal is dispatched to the robot's local Nav2 action server to travel to the frontier and expand the map. The process repeats until no valid frontiers remain.

## Map Merging Approach
The Map Merging node subscribes to the `/robot1/map` and `/robot2/map` topics. 
Because the maps are dynamically built, it converts the ROS 2 OccupancyGrid `-1, 0, 100` values into grayscale OpenCV images. It uses OpenCV's ORB (Oriented FAST and Rotated BRIEF) feature detector to extract keypoints from the environmental structure. The keypoints from both local maps are then matched using a Brute Force matcher with Hamming distance. A homography matrix is computed using RANSAC to calculate the relative 2D transform between the two grids. The transformed maps are stitched together using `np.maximum()` and published continuously to the `/global_map` topic.

## How to run it
1. Clone the repository and build the workspace:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```
2. Export the TurtleBot3 model:
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ```
3. Run the master launch file (you can supply different spawn coordinates if desired):
   ```bash
   ros2 launch bringup master.launch.py robot1_x:=-2.0 robot1_y:=-0.5 robot2_x:=2.0 robot2_y:=0.5
   ```
4. Observe the robots autonomously exploring and the merged global map on the `/global_map` topic in RViz.
