## Assignment 4: Learning-based control of a robotic arm

## Overview:  
This assignment focuses on implementing learning-based control strategies for robotic manipulators. You will train an agent to perform target-reaching and obstacle avoidance using either: Reinforcement Learning (RL) when an agent learns through trial-and-error interactions and Imitation Learning (IL) when an agent learns from demonstrations provided by an expert controller.

## Simulation Environment  

You will work with a custom 3-link planar robotic arm environment (Fig. 1). 

<p align="center">
  <img src="env.png" alt="Simulation Environment" width="600"/>
</p>

### Exercises  
The assignment has 4 exercises:

* **Part 1**: Train an IL agent
* **Part 2**: Train an RL agent
* **Part 3**: Analysis and Comparison
* **Part 4**: Evaluation on unseen environment
	
For **Part 1 and Part 2**, you can use any of the algorithms discussed in the class. You are expected to deliver the following:
* Code implementation 
* A short supporting video [like https://www.youtube.com/watch?v=yembjBOEUOI]

**Note**: Please do not include the video inside the ZIP file. Instead, provide a YouTube or Google Drive link to the video in your report.

For **part 3**, prepare a comparative analysis between imitation learning and reinforcement learning agent. Discuss their advantages, limitations, and suitable application domains. Additionally, you can also use any metrics to compare them based on your understanding.

For **part 4**, your planner will be tested on a different environment, where obstacle/goal position/size will be varied. The goal is to evaluate adaptability and robustness of your implementation.

#### Extra Credits
* Refine the IL model using RL model
* Extend your implementation to handle moving obstacles
 
## Setup Instructions  

### Requirements  
- Python 3.8+  
- NumPy 
- If using OpenAI Gym for environment, follow instructions here - https://gymnasium.farama.org/
