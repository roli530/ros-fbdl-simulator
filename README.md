# Installation steps
## Required Environment:
- ROS 2 Humble/Foxy
- ROS Bridge Server (`sudo apt install ros-humble-rosbridge-server`)
- NodeJS

## Installation:

1. Create a ROS workspace and clone this repository into the `src` directory.
2. Navigate to the `web_ros2` directory and run the command `npm install`.
3. Navigate to the workspace directory and run the command `colcon build`.
4. Use the command `source install/setup.bash`.
5. Start the background system with the command `ros2 launch ros2_fuzzy simulation_controller`.
6. Navigate to the `src/web_ros2/src` directory and start the web interface with the command `node server.js`.
7. Access the simulator in your browser at `localhost:3000`.
