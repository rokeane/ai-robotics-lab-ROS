# ai-robotics-lab-ROS
This project sets out to develop a 3-layer architecture for an autonomous ground drone. As part of its mission, the ground drone robot aims to navigate around a structured environment, capture images of objects of interest, and follow a moving target. 

* The 3-layer architecture is composed of 3 components: 
  * A controller responsible for coupling sensors to the actuators with feedback control loops
  * A sequencer that computes action sequences to be taken by the robot at a given time
  * A deliberator that determines the path of executable tasks that will fulfil the mission when executed
