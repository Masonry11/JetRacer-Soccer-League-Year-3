## Introduction

The simulation environment will be used for training our rc car ai agents with reinforcement learning. Unity provides an [ml-agents library](https://github.com/Unity-Technologies/ml-agents) that can be used for training our agents. The [ml-agents library](https://github.com/Unity-Technologies/ml-agents) conveniently contains an example soccer twos simulation that we can adapt to our needs. Ideally, we would like to gather predictions on the throttle and steering angle that should be applied to our rc cars based on their camera feeds.

### Replicating physicality of RC cars

Current State:

The dimensions of the agent's game object has been modified to roughly rectangularly approximate the rc car. Instructions for doing this can be found in [SETUP.md](./SETUP.md).

Future Work:

The size and weight of the cars, friction of the wheels, etc. must also be replicated. This can be done by modifying the game objects of the agents in Unity.

### Observation Space

The observation space of the agents in the simulation are rays that come out of the agent's game object in equally spaced intervals. These rays detect the class of an object (ball, ally car, enemy car, ally goal, enemy goal, boundary) and its respective distance.

Current State:

2V2 fields and 1v1 fields have been created to simulate the movement further. Movement parameters have been adjusted.

Future Work:

A middleware must be developed to translate the object detection model's output to input for the simulation's observation space. This middleware should take the bounding boxes and class of the object from the object detection model, calculate the distance of the object, then assign the class and distance of the object to the respective observation ray.

### Known Issues

After the modifications in [SETUP.md](./SETUP.md) are made and the simulation is training, the frame rate is extremely low. The ball and agents are observed to be moving, but sometimes the ball moves without any visible interaction with the agents.
