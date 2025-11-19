## Assignment 2: Force control of a robotic arm


### Testing Instructions

1. Download `box_slanted.obj` from assests folder and copy the `box_slanted.obj` in this directory in your local repository: `'/assets/box_slanted.obj'`.

2. Comment this line in your code in the `create_world` function: 
`load_box(position=np.array([0.6, 0., 0.25]), dimensions=(0.7, 1, 0.5), mass=0)`

3. Add this line in your code in the `create_world` function: `load_mesh(filename=os.path.join(asset_dir, 'box_slanted.obj'), position=np.array([0.3, 0, 0.25]), orientation=p.getQuaternionFromEuler([0, 0, np.pi/2]), mass=0)`

4. Add this function in `main.py`
```python
def load_mesh(filename, position, orientation=(0, 0, 0, 1), mass=1., scale=(1., 1., 1.),
              color=None, with_collision=True, flags=None, *args, **kwargs):
    kwargs = {}
    if flags is not None:
        kwargs['flags'] = flags
    # create collision shape if specified
    collision_shape = None
    if with_collision:
        collision_shape = p.createCollisionShape(p.GEOM_MESH, fileName=filename, meshScale=scale,
                                                        **kwargs)
    if color is not None:
        kwargs['rgbaColor'] = color
    # create visual shape
    visual_shape = p.createVisualShape(p.GEOM_MESH, fileName=filename, meshScale=scale, **kwargs)
    # create body
    if with_collision:
        mesh = p.createMultiBody(baseMass=mass,
                                        baseCollisionShapeIndex=collision_shape,
                                        baseVisualShapeIndex=visual_shape,
                                        basePosition=position,
                                        baseOrientation=orientation)
    else:
        mesh = p.createMultiBody(baseMass=mass,
                                        baseVisualShapeIndex=visual_shape,
                                        basePosition=position,
                                        baseOrientation=orientation)
    return mesh`
```


## Overview:  
This assignment focuses on implementing and analyzing force control strategies for robotic manipulators. You will explore admittance and impedance control schemes.

## Simulation Environment  

The simulation environment is PyBullet, which provides realistic kinematics and dynamic utilities for a robotic system. 
<p align="center">
  <img src="env.png" alt="Simulation Environment" width="600"/>
</p>

### Exercises  
The assignment has 6 exercises:

* **Part 1**: Admittance control for static interaction
* **Part 2**: Admittance control for dynamic interaction
* **Part 3**: Impedance control for static interaction
* **Part 4**: Impedance control for dynamic interaction
* **Part 5**: Analysis and Comparison
* **Part 6**: Evaluation on unseen environment
	
***Static Interaction***: The end-effector interacts with the environment at a fixed point (e.g., maintaining contact force on a stationary surface). 

***Dynamic Interaction***: The end-effector moves along a continuous path (e.g., circular motion) while maintaining a desired contact force with the environment. 	

For **parts 1-4**, you are expected to deliver the following:
* Code implementation for both the controllers and the interactions
* Plots of motion trajectories and contact forces (real-time plotting preferred)

For **part 5**, prepare a comparative analysis between admittance and impedance control schemes. Discuss their advantages, limitations, and suitable application domains. Additionally, you can also use any metrics to compare them based on your understanding. Some are provided here for reference: Force tracking error, Settling time, Motion smoothness

For **part 6**, your planner will be tested on a different environment, where surface topology  will be varied. The goal is to evaluate adaptability and robustness of your implementation.

#### Extra Credits
* Extend your implementation to handle interactions with non-planar surfaces.
* Implement online adaptation of virtual mass, damping, or stiffness based on contact conditions.
 
## Setup Instructions  

### Requirements  
- Python 3.8+  
- [PyBullet](https://pybullet.org): Install using `pip install pybullet`
- NumPy 
