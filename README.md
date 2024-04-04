# Machina_Labs_Robotic_HW
This is my submission for the Machina Labs robotic homework.

## Overview
My solution contains two packages, `robotic_hw_interfaces` and `robotic_hw`.

`robotic_hw_interfaces` contains the custom interfaces (services and messages) needed for my solution.
* `srv/GetLoadCellData.srv` contains the definition of the service for the 3-DOF sensor.
* `msg/LoadCellData.msg` defines a message containing information about a 3-DOF sensor measurement.
* `msg/LoadCellDataArray.msg` defines the message published by the publisher node. It is an array because the publisher requests data from multiple servers and aggregates them into one topic.

`robotic_hw` contains the implementation of my server and publisher nodes, as well as launch files for convenience.
* `robotic_hw/load_cell_data_server.py` is a node which requests data from a load cell through a socket. It exposes this data through a ROS2 service.
  * The server is robust and will attempt to periodically reconnect if the load cell socket disconnects.
  * The socket IP, port and number of samples to request are parameters of the node.
  * Since the assignment mentions filtered data, a simple lowpass filter is applied by averaging the received samples into one filtered sample.
* `robotic_hw/load_cell_data_publisher.py` is a node which requests load cell data from multiple services and combines them into an array which is published at 500 Hz.
  * The publisher takes a list of services as parameter, so it can easily scale to any number of sensors and services.
  * Publishing frequency (500 Hz) and service request frequency are also parameters of the node.
  * The aggregated data is published as a LoadCellDataArray message to the `/load_cells` topic.

## Usage
This repository can be cloned directly into a ROS2 workspace. 
In addition to standard ROS2 libraries, the only additional dependency is `numpy`.
To build, you can run `colcon build` in the workspace root. The package then must be sourced from the install directory.
To run the code, you can use the launch files defined in the `robotic_hw` package.

Launch Files
  * `launch/two_sensors.launch.py` launches two servers and one publisher.
  * `launch/simulator.launch.py` launches the sensor simulator script, `sensor.py`.
  * `launch/two_sensors_simulated.launch.py` launches both of those launch files, starting the simulator and the nodes.

# Original Assignment

## Context
The design of our cells in Machina Labs has evolved over the past years. Currently, each of our cells has two articulated industrial robots on rails (a total of 7 axes) and a frame with hydraulic clamps. For the parts to form correctly, we must exert and maintain a dynamic force during the forming in a very accurate location in space. Currently, each robot is equipped with a load cell. See a quick video about our process [here](https://www.youtube.com/watch?v=iqYMprTEXRI). We are using ROS2 to collect the data from the network and control the robots in real-time. As a robotic engineer, we keep developing different modules for our network to add features to the system.  
 
## Objective
The goal of This project is to build a ROS2 network that collects data from 3-DOF sensors and makes the filtered data available as a ROS service and topic. Since we cannot send a real sensor to all of our applicants, we made a simple simulator (sensor.py) that mimics the behavior of a real sensor but with random data. 
- The first task is to make a custom service for 3-DOF sensor 
- The second task is to make a ROS2 service server that continuously reads data from the sensor and has the latest filter data available for the client service that you make. 
- Finally, please make a simple client that calls two of these services and publishes them to a topic at 500Hz. Please keep in mind that your service servers can run slower than 500Hz. 
- You can define a second server in the simulator to modify the code and run two at the same time.
- You can check the example.py to see how to make calls to the sensor

## Grading Criteria
- We’re looking for code that is clean, readable, performant, and maintainable.
- The developer must think through the process of deploying and using the solution and provide the necessary documentation. 
- The sensor samples with 2000Hz, and you can request a specific number of samples in each call. Each call also has a ~1ms delay on top of the sampling time. We would like to hear your thought on picking the number of samples that you read in each call. 

## Submission
To submit the assignment, do the following:

1. Navigate to GitHub's project import page: [https://github.com/new/import](https://github.com/new/import)

2. In the box titled "Your old repository's clone URL", paste the homework repository's link: [https://github.com/Machina-Labs/robotic_hw](https://github.com/Machina-Labs/robotic_hw)

3. In the box titled "Repository Name", add a name for your local homework (ex. `Robotic_soln`)

4. Set the privacy level to "Public", then click "Begin Import" button at bottom of the page.

5. Develop your homework solution in the cloned repository and push it to GitHub when you're done. Extra points for good Git hygiene.

6. Send us the link to your repository.

## ROS2
Install instructions (Ubuntu): [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

ROS2 tutorials: [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)

