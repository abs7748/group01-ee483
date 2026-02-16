**Autonomous Duckiebot:**
This Duckietown Taxi project was a group project which focuses on building an autonomous driving pipeline for the Duckiebot that can reliably follow lanes, detect intersections, and execute turns.
The goal is to combine the individual components from previous labs, including lane following and driver control, into a single closed-loop navigation system capable of getting from specified point A to point B.

1. Lane Following and PID Node: Processes camera images to detect the yellow and white
lane markings, computes the robot’s lateral and angular error, and outputs velocity
commands through a PID controller.

2. Intersection Detection Node: Analyzes the cropped bottom region of the frame in HSV
color space to detect red stop-line markers. When enough of a red pattern is identified, it
publishes an intersection event signal to a topic (either STOP_1 or STOP_2).

3. Driver (FSM) Node: Serves as the central coordinator. It subscribes to both the PID lane
following commands and the intersection event topic. Based on incoming events, it
switches between driving modes: lane following, stopping, turning, and the final straight
section, while publishing wheel commands to the robot.

<img width="478" height="510" alt="image" src="https://github.com/user-attachments/assets/726e6f01-9b46-4f26-8066-e5c9cf590a05" />

**How to run the program:**
Lab devel -> src- -> Running the driver.py node to run the program for the Autonomous Duckiebot Taxi.

<img width="427" height="374" alt="image" src="https://github.com/user-attachments/assets/2b225137-477f-43b2-b03f-0051ae004a56" />


