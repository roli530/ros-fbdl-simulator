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

# Simulator Controls

- **Compile**: Loads the behavioral pattern designed in the rule editor onto the selected agent.
- **Start**: Starts the simulation.
- **Stop**: Pauses the simulation.
- **New Special Object**: Adds a numbered circular object. After clicking the button, click on the point in the simulation where you want to place the object.
- **New Obstacle**: Similar to the special object, a new obstacle can be added by clicking the button and selecting a point in the simulation.
- **Stop Adding**: This button is hidden by default. It allows continuous creation of new obstacles by clicking, and the button stops this process.
- **Change Color**: Changes the color of the selected obstacle or special object. Select the object, click the button, and choose a new color from the list.
- **Load FBDL**: Loads the FBDL behavioral pattern of the selected agent into the rule editor.
- **Display Universes**: Displays the universes and rule bases used for editing the behavioral pattern.
- **Save**: Saves the current state of the simulation. The system will prompt you for a name and location to save the state. It is important to save it to the `~/web_ros2/src/public/maps` folder; otherwise, the server won't be able to load it later.
- **New Agent**: Creates a new agent with the name specified in the text field next to the button.
- **Change Dimension**: Changes the width and height of the selected obstacle object based on the numeric values in the Width and Height fields.
- **Load**: After pressing the **Stop** button, previously saved simulation states will appear in the list next to the button. The selected state can be loaded by pressing **Load**.
- **Remove**: Allows the deletion of any object in the simulation environment by selecting it and pressing the **Remove** button.
  ![Simulator](simulator.png)
