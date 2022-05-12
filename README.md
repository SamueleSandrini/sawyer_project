# Sawyer project
There are three main nodes:
- **startup.py** is the node that allows the positions corresponding to the objects to be picked up. The positions correspond to "approach_pick", "pick", "approach_leave" and "leave". After running the script, the positions are saved in a configuration file, and it is no longer  necessary to take positions in the future, unless you want to change them.
- **pick_place** is the node that handles the pick&place according to the configuration file containing the object locations stored previously.
- **velocityScaling.py** is the node that is executed in parallel with the movement that allows to scale the robot's speed according to the human-person distance.
